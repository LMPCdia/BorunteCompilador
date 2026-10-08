"""
Segunda salida del compilador: AST -> respaldo del pad (`HCBackupRobot_*.zip`)
que el Borunte importa por pendrive y ejecuta él solo, sin PC ni PLC.

El formato está deducido de UN respaldo real (`docs/PAD_FORMAT.md`) y **no se
probó todavía importando en el pad**. Por eso el generador es conservador:

- Lo que no sabemos expresar es un `CompileError` claro, nunca una adivinanza.
- Lo que sabemos expresar pero el respaldo real NUNCA mostró ("sin
  confirmar") también es un error, salvo con `PadOptions.allow_unverified`.
  El simulador lo habilita (ahí no hay riesgo); para exportar al robot hay que
  pedirlo a propósito.
- Lo que se descarta o se supone sin cambiar el resultado va como aviso
  (`compile_to_pad_report` los devuelve).

| DSL                         | Pad                                  | Confianza |
|-----------------------------|--------------------------------------|-----------|
| `MOVEJ <JOINT> SPEED n`     | `4`, ángulos de eje                  | Deducido  |
| `MOVEL <WORLD> SPEED n`     | `10`, X,Y,Z,U,V,W                    | Deducido  |
| `SET_OUT(Ynnn, ON/OFF)`     | `200`                                | Deducido  |
| `WAIT n s`                  | `100` con `type: 100`                | Hipótesis |
| `PROC` / llamada desde MAIN | módulo + `20000`                     | Deducido  |
| `IF Xnnn == 0 THEN`         | `10001` "si X ON saltar al fin" (`pointStatus: 1`, visto 44 veces) | Deducido |
| `TOOL n` / `COORD n`        | `801` / `800` + `toolCoord` de cada movimiento | Deducido |
| `IF Xnnn == 1 THEN`         | `10001` con `pointStatus: 0`: nunca visto | **Sin confirmar** |
| llamada desde un PROC       | `20000` dentro de un módulo: nunca visto | **Sin confirmar** |

`TOOL` y `COORD` valen para los movimientos que siguen, hasta el final del
programa o del PROC donde están; cada PROC arranca con los valores de
`PadOptions` (no hereda los del que lo llama: en el pad cada movimiento lleva
su herramienta adentro, `toolCoord`, y en el respaldo real 11 movimientos no
coinciden con el último 800/801, así que lo que manda parece ser el
`toolCoord`). Si `PadOptions` trae herramienta o sistema distinto de 0, cada
módulo arranca con 800/801, como los módulos del respaldo real; el principal
real no los tiene, así que no se emiten ahí.

Lo que NO se traduce (error de compilación, con el motivo):

- `MOVEJ` a un punto `WORLD`, `MOVEL` a un punto `JOINT`, y `OFFSET` sobre un
  punto `JOINT` (sumaría grados a los ejes, no milímetros).
- `VAR`, asignaciones e `IF` sobre variables (el `.variables` real estaba vacío).
- `ELSE`: no conocemos un salto incondicional.
- `WAIT_IN`: no hay ningún ejemplo de espera de entrada.
- `PROC` con parámetros, `PROC` dentro de un IF o de otro PROC, y llamadas
  recursivas.
- `TOOL`/`COORD` dentro de un IF.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Any

from comms.robot_client import Pose
from compiler.ast_builder import build_ast
from compiler.ast_nodes import (
    Assignment,
    BinOp,
    CallStmt,
    Const,
    IfStmt,
    MoveStmt,
    PointDecl,
    PointLiteral,
    PointName,
    PointOffset,
    ProcDecl,
    SetOutStmt,
    TimerDecl,
    ToolStmt,
    CoordStmt,
    VarDecl,
    VarRef,
    WaitInStmt,
    WaitTimeStmt,
)
from compiler.codegen import CompileError, with_line
from pad.backup import (
    MAIN_LINE,
    MODULES_LINE,
    TEMPLATE_FNC,
    Action,
    ActFile,
    Module,
    PadBackup,
)

# Líneas del .act que no entendemos: se escriben como en el respaldo real.
_EMPTY_PROGRAM = [{"action": 60000, "insertedIndex": 0}]
_ACT_LINE_COUNT = 19
_UNKNOWN_LINE_10 = 9  # base 0: la línea 10 del archivo, que vale `{}`
_EMPTY_FILES = ("counters", "palletStyle", "timers", "variables")

_PROGRAM_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.\-]{0,39}$")

# Rangos conservadores (hipótesis: el pad real puede aceptar más). En el
# respaldo real se vieron herramientas 0 y 2, sistemas 0, 1 y 2, velocidades
# de 2 a 100 %, y E/S hasta X031/Y034.
MAX_TOOL_OR_COORD = 15
MAX_IO_POINT = 63          # X010..X107 / Y010..Y107
MAX_ABS_POSITION = 10000   # mm o grados
MIN_SPEED, MAX_SPEED = 0.1, 100.0
MIN_WAIT, MAX_WAIT = 0.001, 3600.0


@dataclass
class PadOptions:
    """Lo que el DSL todavía no expresa y hay que decidir al exportar."""

    program_name: str = "BorunteDSL"
    # Herramienta y sistema de coordenadas con que arranca cada programa (los
    # cambia `TOOL n` / `COORD n`). 0/0 es brida y base.
    tool: int = 0
    coord: int = 0
    # Respaldo exportado del MISMO robot para copiar lo que no sabemos generar
    # (el `.fnc`). Sin él se usa el del respaldo real que se analizó.
    template: PadBackup | None = None
    # Permite lo que el respaldo real nunca mostró (ver la tabla de arriba).
    allow_unverified: bool = False


def _check_tool_coord(value: int, word: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= MAX_TOOL_OR_COORD:
        raise CompileError(
            f"{word} {value}: tiene que ser un número entero entre 0 y {MAX_TOOL_OR_COORD} "
            f"(rango conservador: en el pad se vieron hasta el 2)"
        )
    return value


def _finite(value: float, what: str, limit: float = MAX_ABS_POSITION) -> float:
    if not math.isfinite(value) or abs(value) > limit:
        raise CompileError(f"{what} = {value:g}: fuera de rango (máximo ±{limit:g})")
    return value


def _fmt3(value: float) -> str:
    # `or 0.0` evita "-0.000", que el pad nunca escribe.
    return f"{(round(value, 3) or 0.0):.3f}"


def io_point(name: str, prefix: str) -> int:
    """`Y034` -> 20. Se escriben como los muestra el pad: X o Y mayúscula y tres
    dígitos octales desde 010 (hipótesis de `docs/PAD_FORMAT.md`, de 3 casos
    vistos). Es la inversa de `pad.listing.io_name`."""
    kind = "una entrada (X)" if prefix == "X" else "una salida (Y)"
    match = re.fullmatch(r"([XY])([0-7]{3})", name)
    if not match or match.group(1) != prefix:
        raise CompileError(
            f"{name!r} no es {kind} del pad. Se escriben como en el pad: {prefix} "
            f"mayúscula y tres dígitos octales ({prefix}010, {prefix}011, … {prefix}017, "
            f"{prefix}020, …)"
        )
    point = int(match.group(2), 8) - 8
    if not 0 <= point <= MAX_IO_POINT:
        raise CompileError(
            f"{name!r}: las E/S del pad van de {prefix}010 a {prefix}{MAX_IO_POINT + 8:03o} "
            f"(rango conservador)"
        )
    return point


def _sorted(action: dict[str, Any]) -> Action:
    # El pad escribe las claves en orden alfabético.
    return dict(sorted(action.items()))


class _Builder:
    """Acciones de UN programa (el principal o un módulo), con sus etiquetas."""

    def __init__(self, name: str, tool: int = 0, coord: int = 0) -> None:
        self.name = name
        self.actions: list[Action] = []
        self._next_flag = 0
        self.tool = tool
        self.coord = coord
        self.if_depth = 0
        self.set_tool_coord = False
        self.has_moves = False

    @property
    def is_main(self) -> bool:
        return self.name == "MAIN"

    def add(self, **fields: Any) -> Action:
        action = _sorted({**fields, "insertedIndex": len(self.actions)})
        self.actions.append(action)
        return action

    def new_flag(self) -> int:
        flag = self._next_flag
        self._next_flag += 1
        return flag

    def place_label(self, flag: int, name: str) -> None:
        self.add(action=59999, comment=name, flag=flag)


@dataclass
class _Point:
    pose: Pose
    frame: str  # "WORLD" | "JOINT"


@dataclass
class _Compiler:
    options: PadOptions
    modules: dict[str, int] = field(default_factory=dict)  # nombre -> id
    points: dict[str, _Point] = field(default_factory=dict)
    bodies: dict[str, list[Action]] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    calls: dict[str, set[str]] = field(default_factory=dict)  # quién llama a quién
    proc_has_own_frames: dict[str, bool] = field(default_factory=dict)
    # (llamado, herramienta, sistema) vigentes en cada llamada desde el principal
    main_calls: list[tuple[str, int, int]] = field(default_factory=list)
    main_warned = False

    def warn(self, stmt, message: str) -> None:
        line = getattr(stmt, "line", None)
        self.warnings.append(f"Línea {line}: {message}" if line else message)

    def unverified(self, what: str) -> None:
        if not self.options.allow_unverified:
            raise CompileError(
                f"{what} Nunca lo vimos en un respaldo real del pad, así que no se exporta "
                f"salvo que lo habilites a propósito (Programa → Permitir instrucciones sin "
                f"confirmar) y lo pruebes a velocidad baja."
            )

    # -- puntos ---------------------------------------------------------------

    def resolve_point(self, node) -> _Point:
        if isinstance(node, PointLiteral):
            return _Point(node.pose, node.frame)
        if isinstance(node, PointName):
            if node.name not in self.points:
                raise CompileError(
                    f"Punto no definido: {node.name!r} (los POINT se declaran antes de "
                    f"usarlos, también antes del PROC que los usa)"
                )
            return self.points[node.name]
        if isinstance(node, PointOffset):
            base = self.resolve_point(node.base)
            if base.frame == "JOINT":
                raise CompileError(
                    "OFFSET sobre un punto JOINT sumaría grados a los ejes, no milímetros. "
                    "Usá un punto WORLD(...) para desplazar en X, Y, Z."
                )
            b, o = base.pose, node.offset
            return _Point(
                Pose(b.a + o.a, b.b + o.b, b.c + o.c, b.d + o.d, b.e + o.e, b.f + o.f),
                base.frame,
            )
        raise CompileError(f"Nodo de punto desconocido: {node!r}")

    # -- sentencias -------------------------------------------------------------

    def emit_block(self, stmts: list, out: _Builder) -> None:
        for stmt in stmts:
            self.emit(stmt, out)

    def emit(self, stmt, out: _Builder) -> None:
        try:
            self._emit(stmt, out)
        except CompileError as e:
            raise with_line(e, stmt) from None

    def _emit(self, stmt, out: _Builder) -> None:
        if isinstance(stmt, PointDecl):
            if stmt.name in self.points:
                raise CompileError(f"POINT {stmt.name!r} ya estaba declarado")
            self.points[stmt.name] = self.resolve_point(stmt.expr)
        elif isinstance(stmt, TimerDecl):
            self.warn(stmt, f"TIMER {stmt.name} no se exporta al pad (no hace nada todavía).")
        elif isinstance(stmt, (ToolStmt, CoordStmt)):
            self._emit_tool_coord(stmt, out)
        elif isinstance(stmt, MoveStmt):
            self._warn_main_action(stmt, out)
            self._emit_move(stmt, out)
        elif isinstance(stmt, SetOutStmt):
            self._warn_main_action(stmt, out)
            point = io_point(stmt.io_name, "Y")
            out.add(action=200, delay="0.000", isWaitInput=False, point=point,
                    pointStatus=stmt.state, type=0, valveID=point)
        elif isinstance(stmt, WaitTimeStmt):
            if stmt.until_move_done or stmt.seconds is None:
                self.warn(stmt, "WAIT UNTIL MOVE_DONE no se exporta: suponemos que el pad "
                                "termina cada movimiento antes de seguir (sin confirmar, y "
                                "con suavizado podría no ser así).")
                return
            seconds = stmt.seconds
            if not math.isfinite(seconds) or not MIN_WAIT <= seconds <= MAX_WAIT:
                raise CompileError(f"WAIT {seconds:g}s: tiene que estar entre "
                                   f"{MIN_WAIT:g} y {MAX_WAIT:g} segundos")
            out.add(action=100, customName="", isUnlimit=False, limit=_fmt3(seconds),
                    point=0, pointStatus=0, type=100)
        elif isinstance(stmt, IfStmt):
            self._emit_if(stmt, out)
        elif isinstance(stmt, CallStmt):
            self._emit_call(stmt, out)
        elif isinstance(stmt, ProcDecl):
            if not out.is_main or out.if_depth:
                raise CompileError(
                    f"PROC {stmt.name}: los PROC se declaran en el nivel superior, no "
                    f"dentro de un IF ni de otro PROC (en el pad cada uno es un módulo)."
                )
            self._emit_proc(stmt)
        elif isinstance(stmt, (VarDecl, Assignment)):
            raise CompileError(
                "Las variables (VAR y asignaciones) todavía no se pueden exportar al "
                "pad: no sabemos cómo las guarda. Sirven solo para simular en la PC."
            )
        elif isinstance(stmt, WaitInStmt):
            raise CompileError(
                f"WAIT_IN({stmt.io_name}) todavía no se puede exportar al pad: no "
                "tenemos ningún ejemplo de espera de entrada. Hace falta un respaldo "
                "del pad con una (ver docs/PAD_FORMAT.md, 'Para confirmar')."
            )
        else:
            raise CompileError(f"Sentencia no soportada por el pad: {stmt!r}")

    def _warn_main_action(self, stmt, out: _Builder) -> None:
        if out.is_main and not self.main_warned:
            self.main_warned = True
            self.warn(stmt, "El principal del respaldo real solo llama módulos (no tiene "
                            "movimientos ni salidas propias). Debería andar igual, pero si el pad "
                            "lo rechaza, poné estas instrucciones dentro de un PROC.")

    def _emit_tool_coord(self, stmt, out: _Builder) -> None:
        word = "TOOL" if isinstance(stmt, ToolStmt) else "COORD"
        if out.if_depth:
            raise CompileError(
                f"{word} dentro de un IF no se puede exportar: ponelo antes del IF "
                f"(o dentro de un PROC que llame el IF)."
            )
        number = _check_tool_coord(stmt.number, word)
        out.set_tool_coord = True
        if isinstance(stmt, ToolStmt):
            out.tool = number
            out.add(action=801, toolID=number)
        else:
            out.coord = number
            out.add(action=800, coordID=number)

    def _emit_move(self, stmt: MoveStmt, out: _Builder) -> None:
        point = self.resolve_point(stmt.point)
        expected = "JOINT" if stmt.kind == "MOVEJ" else "WORLD"
        if point.frame != expected:
            what = "ángulos de eje" if expected == "JOINT" else "X, Y, Z, U, V, W"
            raise CompileError(
                f"{stmt.kind} necesita un punto {expected}(...) ({what}), y "
                f"recibió uno {point.frame}. En el pad, MOVEJ guarda ángulos de eje "
                f"y MOVEL coordenadas cartesianas: mezclarlos movería el robot a "
                f"cualquier lado."
            )
        speed = stmt.speed
        if not math.isfinite(speed) or not MIN_SPEED <= round(speed, 1) <= MAX_SPEED:
            raise CompileError(f"SPEED {speed:g}: en el pad es un % entre {MIN_SPEED:g} y "
                               f"{MAX_SPEED:g}")
        p = point.pose
        names = ("J1", "J2", "J3", "J4", "J5", "J6") if expected == "JOINT" else "XYZUVW"
        values = [_finite(v, n) for v, n in zip((p.a, p.b, p.c, p.d, p.e, p.f), names)]
        pos = {f"m{i}": _fmt3(v) for i, v in enumerate(values)}
        pos.update(m6="0.000", m7="0.000")
        name = stmt.point.name if isinstance(stmt.point, PointName) else ""
        out.has_moves = True
        out.add(
            action=4 if stmt.kind == "MOVEJ" else 10,
            bindIOInfo=0, ckStatus="63", customName=name, delay="0.000", distance="0.000",
            passTrans=0, points=[{"pointName": "", "pos": pos}], quotePoint=[0, 0, 0],
            relativeType=0, smooth=0, speed=f"{speed:.1f}",
            toolCoord=(out.tool << 16) | out.coord,
        )

    def _emit_if(self, stmt: IfStmt, out: _Builder) -> None:
        if stmt.else_body:
            raise CompileError(
                "ELSE todavía no se puede exportar al pad: no conocemos un salto "
                "incondicional en su formato."
            )
        cond = stmt.cond
        if isinstance(cond, BinOp) and cond.op == "==" and isinstance(cond.right, VarRef):
            cond = BinOp("==", cond.right, cond.left)
        if not (
            isinstance(cond, BinOp) and cond.op == "=="
            and isinstance(cond.left, VarRef) and isinstance(cond.right, Const)
            and cond.right.value in (0, 1)
        ):
            raise CompileError(
                "En el pad, IF solo puede preguntar por una entrada: "
                "IF X010 == 0 THEN ... (o == 1, sin confirmar)"
            )
        point = io_point(cond.left.name, "X")
        # Se saltea el bloque cuando la condición es FALSA: si se pregunta
        # "== 0", el salto es con la entrada en ON (la forma vista en el pad).
        skip_when_on = cond.right.value == 0
        if not skip_when_on:
            self.unverified(
                f"IF {cond.left.name} == 1 necesita «si la entrada está en OFF, saltar» "
                f"(pointStatus 0). Alternativa confirmada: IF {cond.left.name} == 0."
            )
        end = out.new_flag()
        out.add(action=10001, flag=end, inout=0, limit="0.000", point=point,
                pointStatus=1 if skip_when_on else 0, type=0)
        out.if_depth += 1
        try:
            self.emit_block(stmt.then_body, out)
        finally:
            out.if_depth -= 1
        out.place_label(end, f"FinIf{end}")

    def _emit_call(self, stmt: CallStmt, out: _Builder) -> None:
        if stmt.name not in self.modules:
            raise CompileError(f"Llamada a procedimiento no definido: {stmt.name!r}")
        if stmt.args:
            raise CompileError(f"{stmt.name}(): los módulos del pad no reciben parámetros")
        if not out.is_main:
            self.unverified(f"{out.name} llama a {stmt.name}(): en el respaldo real solo el "
                            f"programa principal llama módulos.")
        self.calls.setdefault(out.name, set()).add(stmt.name)
        if out.is_main:
            self.main_calls.append((stmt.name, out.tool, out.coord))
        out.add(action=20000, flag=-1, module=str(self.modules[stmt.name]))

    def _emit_proc(self, stmt: ProcDecl) -> None:
        if stmt.params:
            raise CompileError(
                f"PROC {stmt.name}({', '.join(stmt.params)}): los módulos del pad no "
                f"reciben parámetros"
            )
        # Los POINT declarados adentro del PROC son locales: se restauran al salir.
        outer_points = dict(self.points)
        body = _Builder(stmt.name, self.options.tool, self.options.coord)
        if self.options.tool or self.options.coord:
            # Como los módulos del respaldo real que usan otro sistema.
            body.add(action=800, coordID=self.options.coord)
            body.add(action=801, toolID=self.options.tool)
        try:
            self.emit_block(stmt.body, body)
        finally:
            self.points = outer_points
        body.add(action=20001)
        self.bodies[stmt.name] = body.actions
        self.proc_has_own_frames[stmt.name] = body.set_tool_coord or not body.has_moves

    def check_calls(self, main: _Builder) -> None:
        # Recursión: un ciclo en el grafo de llamadas no termina nunca.
        def visit(name: str, path: list[str]) -> None:
            for callee in sorted(self.calls.get(name, ())):
                if callee in path:
                    cycle = " → ".join(path[path.index(callee):] + [callee])
                    raise CompileError(f"Llamada recursiva: {cycle}. El pad no tiene pila de "
                                       f"llamadas.")
                visit(callee, path + [callee])

        for name in self.modules:
            visit(name, [name])
        # PROC con movimientos que no fija su herramienta, llamado en un momento
        # en que el principal usaba otra: probablemente se esperaba que la herede.
        defaults = (self.options.tool, self.options.coord)
        warned = set()
        for callee, tool, coord in self.main_calls:
            if (tool, coord) != defaults and callee not in warned \
                    and not self.proc_has_own_frames.get(callee, True):
                warned.add(callee)
                self.warnings.append(
                    f"{callee}() no fija TOOL/COORD y arranca con los de la exportación "
                    f"({defaults[0]}/{defaults[1]}), no con los que el principal tenía al "
                    f"llamarlo ({tool}/{coord}). Si tiene que usar otros, poné TOOL/COORD "
                    f"adentro del PROC.")


def _collect_modules(statements: list) -> dict[str, int]:
    """Pasada 0: id de módulo para cada PROC, en orden de declaración, para
    que las llamadas hacia adelante ya sepan a qué módulo ir."""
    modules: dict[str, int] = {}
    for stmt in statements:
        if isinstance(stmt, ProcDecl):
            if stmt.name in modules:
                raise with_line(CompileError(f"PROC {stmt.name!r} declarado más de una vez"), stmt)
            modules[stmt.name] = len(modules)
    return modules


def _check_template(template: PadBackup) -> None:
    """Solo se copia de la plantilla lo que en el respaldo real era "vacío":
    si ahí hay algo, sería código o configuración ajena al programa nuevo."""
    def is_empty(i: int, line) -> bool:
        if i == _UNKNOWN_LINE_10:
            return line == {}
        return (isinstance(line, list) and len(line) == 1 and isinstance(line[0], dict)
                and line[0].get("action") == 60000)

    for i, line in enumerate(template.act.lines):
        if i in (MAIN_LINE, MODULES_LINE):
            continue
        if not is_empty(i, line):
            raise CompileError(
                f"La plantilla tiene contenido en la línea {i + 1} del .act, que en el "
                f"respaldo analizado estaba vacía. Exportá del pad un programa vacío y "
                f"usalo de plantilla.")
    for ext in _EMPTY_FILES:
        if template.others.get(ext):
            raise CompileError(f"La plantilla tiene datos en .{ext}: usá un programa vacío.")


def compile_to_pad_report(source: str, options: PadOptions | None = None) -> tuple[PadBackup, list[str]]:
    """Compila y devuelve el respaldo y los avisos (lo descartado o supuesto)."""
    options = options or PadOptions()
    if not _PROGRAM_NAME_RE.match(options.program_name):
        raise CompileError(
            f"Nombre de programa {options.program_name!r}: hasta 40 letras sin acentos, "
            f"números, '_', '-' y '.', empezando con letra o número"
        )
    _check_tool_coord(options.tool, "TOOL")
    _check_tool_coord(options.coord, "COORD")
    if options.template is not None:
        _check_template(options.template)

    ast = build_ast(source)
    compiler = _Compiler(options, modules=_collect_modules(ast.statements))
    main = _Builder("MAIN", options.tool, options.coord)
    main.add(action=50000, comment="Generado por Borunte DSL")
    compiler.emit_block(ast.statements, main)
    main.add(action=60000)
    compiler.check_calls(main)

    modules = [Module(id=i, name=name, actions=compiler.bodies[name])
               for name, i in compiler.modules.items()]

    lines = [list(_EMPTY_PROGRAM) for _ in range(_ACT_LINE_COUNT)]
    lines[_UNKNOWN_LINE_10] = {}
    others = {ext: b"" for ext in _EMPTY_FILES}
    others["fnc"] = (options.template.others.get("fnc") if options.template is not None
                     and options.template.others.get("fnc") else TEMPLATE_FNC.read_bytes())

    act = ActFile(lines)
    act.lines[MAIN_LINE] = main.actions
    act.lines[MODULES_LINE] = []
    act.modules = modules
    return PadBackup(name=options.program_name, act=act, others=others), compiler.warnings


def compile_to_pad(source: str, options: PadOptions | None = None) -> PadBackup:
    return compile_to_pad_report(source, options)[0]
