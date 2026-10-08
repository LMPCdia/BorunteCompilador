"""
Backend `.act`: traduce el AST al formato nativo de programa del robot Borunte.

Es un backend ALTERNATIVO a `compiler/codegen.py`, no un reemplazo del archivo.
Los dos parten del mismo AST y emiten cosas distintas:

    compiler/codegen.py    AST -> bytecode para una VM en el PLC
    compiler/act_backend.py  AST -> programa .act que el robot importa y corre

La diferencia de fondo es dónde vive la lógica. Con el backend del PLC, el robot
es un ejecutor de posiciones y toda la secuencia la lleva el PLC. Con este
backend, la secuencia la lleva **el robot mismo**, en su formato nativo, y no
hace falta escribir ninguna VM en ladder.

## De dónde sale el formato

De ingeniería inversa sobre un export real del robot
(`AberturaSanLorenzo2026.act`, 338 KB, 1032 acciones, 9 subprogramas). No hay
documentación de Borunte sobre este formato. `compiler/act_reader.py` tiene la
descripción de la estructura y un round-trip byte a byte que la valida.

Lo que está **confirmado por observación** (la forma del registro no deja lugar
a dudas): la estructura del archivo, `SET_OUT` (200), `BASE` (800), `TOOL`
(801), el `IF` sobre entrada (10001) con etiquetas (59999), la llamada a
subprograma (20000), los finales (20001 / 60000).

Lo que es **hipótesis y está marcado como tal**: cuál de las dos acciones de
movimiento es `MOVEJ` y cuál `MOVEL` (ver `MOVE_ACTIONS`), y que la acción 100
sea la espera temporizada (ver `ACTION_WAIT`).

## Control de flujo: por qué no hay ELSE

En las 1032 acciones del export real **no aparece ni un solo salto
incondicional**. El `IF` nativo (10001) es siempre "si la condición se cumple,
saltar adelante a una etiqueta", y el cuerpo entre el `IF` y su etiqueta corre
cuando la condición es FALSA.

Eso alcanza para compilar `IF ... THEN cuerpo ENDIF` (se invierte la condición y
se salta por encima del cuerpo), pero **no** para un `ELSE`: para saltear la
segunda rama haría falta un salto incondicional que no existe entre los opcodes
confirmados. Por eso `ELSE` levanta `ActCompileError` en vez de generar algo
inventado, que es la regla 1 del proyecto.

(El programa real sí encadena ramas, pero lo hace aprovechando que el `flag` de
una llamada a subprograma actúa como "saltar a esta etiqueta al volver". Eso
solo sirve si la rama termina en una llamada, así que no es un salto
incondicional de propósito general.)
"""

from __future__ import annotations

from dataclasses import dataclass, field

from comms.robot_client import Pose
from compiler.act_reader import ActFile, ActSubprogram, dumps_act
from compiler.ast_builder import build_ast
from compiler.codegen import io_number
from compiler.ast_nodes import (
    Assignment,
    BaseStmt,
    BinOp,
    CallStmt,
    Const,
    IfStmt,
    InputRef,
    MoveStmt,
    PointDecl,
    PointLiteral,
    PointName,
    PointOffset,
    ProcDecl,
    SetOutStmt,
    StateConst,
    TimerDecl,
    ToolStmt,
    VarDecl,
    WaitInStmt,
    WaitTimeStmt,
)

# --- códigos de acción (catálogo observado en el export real) ----------------

ACTION_MOVEJ = 4
ACTION_MOVEL = 10
ACTION_WAIT = 100
ACTION_SET_OUT = 200
ACTION_BASE = 800
ACTION_TOOL = 801
ACTION_IF_INPUT = 10001
ACTION_CALL = 20000
ACTION_END_SUB = 20001
ACTION_COMMENT = 50000
ACTION_LABEL = 59999
ACTION_END_MAIN = 60000

#: ⚠️ HIPÓTESIS SIN CONFIRMAR, y la más importante de todas.
#:
#: En el export real conviven las acciones 4 y 10 con EXACTAMENTE los mismos
#: campos, así que por la forma del registro no se puede saber cuál es cuál. Este
#: mapeo se decidió mirando el uso (la 10 es 21 veces más frecuente y usa
#: velocidades bajas, lo típico del trabajo lineal fino; la 4 aparece con
#: velocidades altas, lo típico de un reposicionamiento articular).
#:
#: Si está al revés, el robot hace trayectorias distintas de las escritas — un
#: movimiento articular donde se esperaba uno lineal puede meter la herramienta
#: donde no va. VALIDAR CONTRA EL PAD ANTES DE MANDAR UN PROGRAMA A PRODUCCIÓN.
#: Se corrige acá, en un solo lugar.
MOVE_ACTIONS = {
    "MOVEJ": ACTION_MOVEJ,
    "MOVEL": ACTION_MOVEL,
}

#: Valores por defecto de cada acción, tomados de los campos que en el export
#: real son constantes en todas sus apariciones.
DEFAULTS_MOVE = {
    "bindIOInfo": 0,
    "ckStatus": "63",
    "customName": "",
    "delay": "0.000",
    "distance": "0.000",
    "passTrans": 0,
    "quotePoint": [0, 0, 0],
    "relativeType": 0,
    "smooth": 0,
}

#: Coordenada de base y herramienta por defecto, hasta que el programa use
#: BASE/TOOL. 0 es el valor que aparece en el export para movimientos sin
#: herramienta asignada.
DEFAULT_COORD_ID = 0
DEFAULT_TOOL_ID = 0


def pack_tool_coord(tool_id: int, coord_id: int) -> int:
    """Empaqueta herramienta y base en el campo `toolCoord` de un movimiento.

    ⚠️ HIPÓTESIS. En el export real `toolCoord` vale 131072, 131073 y 131074,
    que son `2<<16 | 0`, `2<<16 | 1` y `2<<16 | 2`; y justo en ese programa
    `toolID` vale 2 y `coordID` vale 1 o 2. El encaje es demasiado exacto para
    ser casualidad, pero no está confirmado.
    """
    return ((tool_id & 0xFFFF) << 16) | (coord_id & 0xFFFF)


def format_fixed(value: float) -> str:
    """Los números van como string con 3 decimales: "0.000", "147.236"."""
    return f"{float(value):.3f}"


def format_speed(value: float) -> str:
    """La velocidad va con UN decimal: "100.0", "50.0", "2.0"."""
    return f"{float(value):.1f}"


def pose_to_points(pose: Pose) -> list[dict]:
    """Una pose como la lista `points` de un movimiento.

    `m6` y `m7` van siempre en "0.000": en las 556 acciones de movimiento del
    export real no hay una sola excepción. Son los dos ejes externos, que este
    robot no tiene.
    """
    valores = (pose.a, pose.b, pose.c, pose.d, pose.e, pose.f)
    pos = {f"m{i}": format_fixed(v) for i, v in enumerate(valores)}
    pos["m6"] = "0.000"
    pos["m7"] = "0.000"
    return [{"pointName": "", "pos": pos}]


class ActCompileError(Exception):
    """Algo del DSL no se puede expresar en el formato .act.

    Se levanta en vez de generar algo aproximado: un .act mal formado lo abre un
    robot real. Siempre explica POR QUÉ no se puede, no solo que no se puede.
    """


@dataclass
class _Emitter:
    """Emite la lista de acciones de un programa (principal o subprograma)."""

    acciones: list[dict] = field(default_factory=list)
    #: Etiquetas usadas en ESTE programa. En el export real los ids arrancan de
    #: 0 en cada subprograma, así que el alcance es por programa.
    proximo_label: int = 0

    def emit(self, accion: dict) -> int:
        # insertedIndex = la posición. En el export real no coincide con el
        # orden de ejecución (es el orden en que el operario insertó las líneas
        # en el pad), pero generando de cero lo más honesto es que coincidan.
        accion["insertedIndex"] = len(self.acciones)
        self.acciones.append(accion)
        return len(self.acciones) - 1

    def nuevo_label(self) -> int:
        label = self.proximo_label
        self.proximo_label += 1
        return label


class ActBackend:
    def __init__(self) -> None:
        self.points: dict[str, Pose] = {}
        self.procs: dict[str, ProcDecl] = {}
        self.coord_id = DEFAULT_COORD_ID
        self.tool_id = DEFAULT_TOOL_ID

    # -- entrada principal ---------------------------------------------------

    def compile(self, statements: list) -> ActFile:
        self._collect(statements)

        principal = _Emitter()
        for stmt in statements:
            if isinstance(stmt, (ProcDecl, PointDecl)):
                continue  # ya recogidos; no emiten nada en línea
            self.emit_stmt(stmt, principal)
        principal.emit({"action": ACTION_END_MAIN})

        biblioteca = []
        for proc in self.procs.values():
            sub = _Emitter()
            # Cada subprograma arranca con su propio estado de base/herramienta
            # heredado del principal: el robot no lo resetea solo.
            for stmt in proc.body:
                self.emit_stmt(stmt, sub)
            sub.emit({"action": ACTION_END_SUB})
            biblioteca.append(
                ActSubprogram(id=proc.proc_id, name=proc.name, actions=sub.acciones)
            )

        return self._build_file(principal.acciones, biblioteca)

    def _collect(self, statements: list) -> None:
        """Recoge puntos y subprogramas antes de emitir, para que una llamada
        hacia adelante también resuelva (mismo motivo que la pasada 0 de
        `compiler/codegen.py`)."""
        for stmt in statements:
            if isinstance(stmt, PointDecl):
                self.points[stmt.name] = self.resolve_point(stmt.expr)
            elif isinstance(stmt, ProcDecl):
                # El orden importa: quien escribe PROC HOME(altura) tiene un
                # problema más de fondo que "te falta el id", así que ese caso
                # se reporta primero.
                if stmt.params:
                    raise ActCompileError(
                        f"El PROC {stmt.name!r} tiene parámetros, y el formato .act "
                        f"no los soporta: una llamada nativa (acción "
                        f"{ACTION_CALL}) solo lleva el número de subprograma. "
                        f"Usá entradas o salidas para pasar información, y "
                        f"declaralo como PROC {stmt.name}(id=<n>)."
                    )
                if stmt.proc_id is None:
                    raise ActCompileError(
                        f"El PROC {stmt.name!r} necesita un id explícito para el "
                        f"backend .act: escribilo como PROC {stmt.name}(id=<n>). "
                        f"Ese número es la entrada en la biblioteca de "
                        f"subprogramas del robot, y se asigna a mano."
                    )
                if stmt.name in self.procs:
                    raise ActCompileError(f"PROC {stmt.name!r} declarado más de una vez")
                usados = {p.proc_id: n for n, p in self.procs.items()}
                if stmt.proc_id in usados:
                    raise ActCompileError(
                        f"El id {stmt.proc_id} ya lo usa el PROC "
                        f"{usados[stmt.proc_id]!r}: dos subprogramas no pueden "
                        f"compartir entrada en la biblioteca del robot."
                    )
                self.procs[stmt.name] = stmt

    def _build_file(self, principal: list[dict], biblioteca: list[ActSubprogram]) -> ActFile:
        """Arma el archivo con la misma disposición de líneas que el export real.

        Las líneas de relleno (`[{"action":60000,...}]` y `{}`) se reproducen
        porque están en el export del robot y no sabemos qué las lee. Omitirlas
        sería apostar a que no hacen falta.
        """
        relleno = [{"action": ACTION_END_MAIN, "insertedIndex": 0}]
        documentos: list = [principal]
        documentos += [list(relleno) for _ in range(8)]
        documentos.append({})
        documentos.append([
            {"id": s.id, "name": s.name, "program": dumps_act(s.actions)}
            for s in biblioteca
        ])
        documentos += [list(relleno) for _ in range(8)]
        return ActFile(documents=documentos, main_line=0, library_line=10)

    # -- puntos ---------------------------------------------------------------

    def resolve_point(self, node) -> Pose:
        if isinstance(node, PointLiteral):
            return node.pose
        if isinstance(node, PointName):
            if node.name not in self.points:
                raise ActCompileError(f"Punto no definido: {node.name!r}")
            return self.points[node.name]
        if isinstance(node, PointOffset):
            base = self.resolve_point(node.base)
            off = node.offset
            return Pose(*(
                b + o
                for b, o in zip(
                    (base.a, base.b, base.c, base.d, base.e, base.f),
                    (off.a, off.b, off.c, off.d, off.e, off.f),
                )
            ))
        raise ActCompileError(f"Nodo de punto desconocido: {node!r}")

    # -- sentencias -------------------------------------------------------------

    def emit_stmt(self, stmt, em: _Emitter) -> None:
        if isinstance(stmt, MoveStmt):
            self._emit_move(stmt, em)
        elif isinstance(stmt, SetOutStmt):
            self._emit_set_out(stmt, em)
        elif isinstance(stmt, BaseStmt):
            self.coord_id = stmt.coord_id
            em.emit({"action": ACTION_BASE, "coordID": stmt.coord_id})
        elif isinstance(stmt, ToolStmt):
            self.tool_id = stmt.tool_id
            em.emit({"action": ACTION_TOOL, "toolID": stmt.tool_id})
        elif isinstance(stmt, WaitTimeStmt):
            self._emit_wait(stmt, em)
        elif isinstance(stmt, IfStmt):
            self._emit_if(stmt, em)
        elif isinstance(stmt, CallStmt):
            self._emit_call(stmt, em)
        elif isinstance(stmt, PointDecl):
            self.points[stmt.name] = self.resolve_point(stmt.expr)
        elif isinstance(stmt, ProcDecl):
            pass  # las declaraciones anidadas ya se recogieron
        elif isinstance(stmt, WaitInStmt):
            raise ActCompileError(
                "WAIT_IN no está soportado por el backend .act: no hay ninguna "
                "acción confirmada que bloquee esperando una entrada. La acción "
                f"{ACTION_WAIT} espera por TIEMPO, no por entrada. Si necesitás "
                "esperar un sensor, usá IF INPUT(...) para ramificar."
            )
        elif isinstance(stmt, (VarDecl, Assignment)):
            nombre = getattr(stmt, "name", "?")
            raise ActCompileError(
                f"Las variables internas ({nombre!r}) no están soportadas por el "
                f"backend .act: no hay opcode confirmado para leerlas ni "
                f"escribirlas. El estado del programa se lleva con entradas y "
                f"salidas físicas."
            )
        elif isinstance(stmt, TimerDecl):
            raise ActCompileError(
                f"TIMER ({stmt.name!r}) no está soportado por el backend .act. "
                f"Usá WAIT <n>s, que emite la acción {ACTION_WAIT}."
            )
        else:
            raise ActCompileError(f"Sentencia no soportada por el backend .act: {stmt!r}")

    def _emit_move(self, stmt: MoveStmt, em: _Emitter) -> None:
        if stmt.kind not in MOVE_ACTIONS:
            raise ActCompileError(f"Tipo de movimiento desconocido: {stmt.kind!r}")
        accion = dict(DEFAULTS_MOVE)
        accion["action"] = MOVE_ACTIONS[stmt.kind]
        accion["points"] = pose_to_points(self.resolve_point(stmt.point))
        accion["speed"] = format_speed(stmt.speed)
        accion["toolCoord"] = pack_tool_coord(self.tool_id, self.coord_id)
        em.emit(accion)

    def _emit_set_out(self, stmt: SetOutStmt, em: _Emitter) -> None:
        numero = io_number(stmt.io_name)
        em.emit({
            "action": ACTION_SET_OUT,
            "delay": "0.000",
            "isWaitInput": False,
            # En el export real `point` y `valveID` son siempre el mismo número.
            "point": numero,
            "valveID": numero,
            "pointStatus": bool(stmt.state),
            "type": 0,
        })

    def _emit_wait(self, stmt: WaitTimeStmt, em: _Emitter) -> None:
        if stmt.until_move_done:
            raise ActCompileError(
                "WAIT UNTIL MOVE_DONE no hace falta en el backend .act: los "
                "movimientos nativos del robot ya son bloqueantes. Sacalo."
            )
        em.emit({
            "action": ACTION_WAIT,
            "customName": "",
            "isUnlimit": False,
            "limit": format_fixed(stmt.seconds or 0.0),
            "point": 0,
            "pointStatus": 0,
            "type": ACTION_WAIT,
        })

    def _emit_call(self, stmt: CallStmt, em: _Emitter) -> None:
        if stmt.name not in self.procs:
            raise ActCompileError(f"Llamada a procedimiento no definido: {stmt.name!r}")
        if stmt.args:
            raise ActCompileError(
                f"La llamada a {stmt.name}() lleva argumentos, y la llamada nativa "
                f"del robot (acción {ACTION_CALL}) solo lleva el número de "
                f"subprograma."
            )
        em.emit({
            "action": ACTION_CALL,
            # flag -1 = seguir en la línea siguiente al volver. Un valor de
            # etiqueta acá haría saltar al volver, que es como el programa real
            # encadena ramas; el compilador no lo usa.
            "flag": -1,
            "module": str(self.procs[stmt.name].proc_id),
        })

    # -- IF -----------------------------------------------------------------------

    def _emit_if(self, stmt: IfStmt, em: _Emitter) -> None:
        if stmt.else_body:
            raise ActCompileError(
                "ELSE no está soportado por el backend .act. Para saltear la "
                "segunda rama haría falta un salto incondicional, y en el export "
                "real del robot (1032 acciones) no aparece ninguno: el IF nativo "
                f"(acción {ACTION_IF_INPUT}) solo sabe saltar hacia adelante "
                "cuando la condición se cumple. Escribilo como dos IF con las "
                "condiciones opuestas."
            )

        entrada, encendido = self._condicion_de_entrada(stmt.cond)

        # La condición se INVIERTE: el IF nativo salta cuando se cumple, y lo que
        # queremos es saltear el cuerpo cuando NO se cumple. Con la condición dada
        # vuelta, el salto se lleva el caso contrario y el cuerpo queda en el
        # camino correcto.
        label = em.nuevo_label()
        em.emit({
            "action": ACTION_IF_INPUT,
            "flag": label,
            "inout": 0,
            "limit": "0.000",
            "point": entrada,
            "pointStatus": 0 if encendido else 1,
            "type": 0,
        })
        for s in stmt.then_body:
            self.emit_stmt(s, em)
        em.emit({"action": ACTION_LABEL, "comment": f"fin_if_{label}", "flag": label})

    @staticmethod
    def _condicion_de_entrada(cond) -> tuple[int, bool]:
        """Valida la condición de un IF y devuelve (número de entrada, estado).

        El backend .act SOLO acepta `INPUT(<entrada>) == ON/OFF`. La comparación
        entre variables internas sigue existiendo en la gramática y la soporta
        `compiler/codegen.py`, pero acá no hay opcode nativo confirmado que
        compare variables.
        """
        ayuda = (
            "El backend .act solo soporta condiciones de la forma "
            "IF INPUT(<entrada>) == ON/OFF."
        )
        if not isinstance(cond, BinOp) or cond.op != "==":
            raise ActCompileError(f"Condición no soportada. {ayuda}")

        izq, der = cond.left, cond.right
        if isinstance(der, InputRef) and isinstance(izq, StateConst):
            izq, der = der, izq
        if not isinstance(izq, InputRef):
            raise ActCompileError(
                f"Condición sobre algo que no es una entrada física: {izq!r}. "
                f"{ayuda} Las variables internas no tienen opcode nativo "
                f"confirmado que las compare."
            )
        if not isinstance(der, StateConst):
            raise ActCompileError(f"El lado derecho tiene que ser ON u OFF. {ayuda}")

        return io_number(izq.name), der.on


def compile_to_act(source: str) -> ActFile:
    """DSL -> `.act`. La entrada principal de este backend."""
    ast = build_ast(source)
    return ActBackend().compile(ast.statements)


def compile_to_act_text(source: str) -> str:
    return compile_to_act(source).to_text()
