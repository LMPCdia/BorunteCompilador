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
| `SMOOTH n`, `DELAY s`, `TOOL n`/`COORD n` en el movimiento | `smooth`, `delay`, `toolCoord` del movimiento | Visto en el listado del pad |
| `NEAR_END`/`AWAY_START Ynnn ON/OFF mm` | `bindIOInfo` + `distance` (`pad/bindio.py`) | Visto en el listado del pad |
| `LABEL n [núm]`, `IF Xnnn == 1 GOTO n`, `P() THEN GOTO n` | `59999`, `10001` (`pointStatus: 1`), `20000` con `flag` | Visto (44 / 45 / 3 casos) |
| `IF Xnnn == 0 GOTO n` | `10001` con `pointStatus: 0`: nunca visto | **Sin confirmar** |
| `DISABLED SET_OUT / IF…GOTO` | `50000` con `commentAction` y el texto del pad | Visto (167 casos) |
| `COMMENT "texto"`, `CMD STOP` | `50000`, `53000` (`Cmd Set:Stop`) | Visto |

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
    CmdStopStmt,
    CommentStmt,
    Const,
    DisabledStmt,
    GotoIfStmt,
    IfStmt,
    LabelStmt,
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
from pad import bindio
from pad.listing import io_name
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
    # `.fnc` (configuración del programa) a escribir en el respaldo: el del
    # respaldo original de un programa importado. Sin él, el de `template`.
    fnc: bytes | None = None


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
        self.label_flags: dict[str, int] = {}     # etiquetas del usuario -> número
        self.label_text: dict[str, str] = {}      # y el nombre que muestra el pad
        self.reserved_flags: set[int] = set()
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
        while self._next_flag in self.reserved_flags:
            self._next_flag += 1
        flag = self._next_flag
        self._next_flag += 1
        return flag

    def flag_of(self, label: str) -> int:
        if label not in self.label_flags:
            where = "el programa principal" if self.is_main else f"el PROC {self.name}"
            raise CompileError(f"La etiqueta {label!r} no está definida en {where} "
                               f"(LABEL {label}). Las etiquetas valen dentro de cada PROC.")
        return self.label_flags[label]

    def add_disabled(self, fields: Action, text: str) -> Action:
        """Una acción desactivada: el pad la guarda como un comentario con la
        acción original adentro y, en `comment`, el texto con que la mostraba."""
        index = len(self.actions)
        inner = _sorted({**fields, "insertedIndex": index})
        return self.add(action=50000, comment=_NBSP * 8 + text, commentAction=inner)

    def place_label(self, flag: int, name: str) -> None:
        self.add(action=59999, comment=name, flag=flag)


@dataclass
class _Point:
    pose: Pose
    frame: str  # "WORLD" | "JOINT"
    label: str | None = None  # `AS "..."`: el nombre que muestra el pad


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
            point = self.resolve_point(stmt.expr)
            self.points[stmt.name] = _Point(point.pose, point.frame, stmt.pad_name)
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
        elif isinstance(stmt, LabelStmt):
            out.place_label(out.flag_of(stmt.name),
                            stmt.pad_name if stmt.pad_name is not None else stmt.name)
        elif isinstance(stmt, GotoIfStmt):
            self._emit_goto_if(stmt, out)
        elif isinstance(stmt, DisabledStmt):
            self._emit_disabled(stmt, out)
        elif isinstance(stmt, CommentStmt):
            out.add(action=50000, comment=stmt.text)
        elif isinstance(stmt, CmdStopStmt):
            out.add(action=53000, **_CMD_STOP)
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
        name = ""
        if isinstance(stmt.point, PointName):
            name = stmt.point.name
            if point.label is not None:
                name = point.label
        tool = _check_tool_coord(stmt.tool if stmt.tool is not None else out.tool, "TOOL")
        coord = _check_tool_coord(stmt.coord if stmt.coord is not None else out.coord, "COORD")
        bind_info, distance = 0, "0.000"
        if stmt.bind is not None:
            bind_info, distance = self._bind(stmt.bind)
        out.has_moves = True
        out.add(
            action=4 if stmt.kind == "MOVEJ" else 10,
            bindIOInfo=bind_info, ckStatus="63", customName=name,
            delay=_fmt3(stmt.delay_s or 0.0), distance=distance,
            passTrans=0, points=[{"pointName": "", "pos": pos}], quotePoint=[0, 0, 0],
            relativeType=0, smooth=stmt.smooth or 0, speed=f"{speed:.1f}",
            toolCoord=(tool << 16) | coord,
        )

    @staticmethod
    def _bind(bind) -> tuple[int, str]:
        """`NEAR_END`/`AWAY_START` -> (bindIOInfo, distance). Ver pad/bindio.py."""
        point = io_point(bind.io_name, "Y")
        if not math.isfinite(bind.distance_mm) or not 0 <= bind.distance_mm <= MAX_ABS_POSITION:
            raise CompileError(f"distancia {bind.distance_mm:g} mm: fuera de rango")
        info = bindio.BindIO(point=point, end=bind.end, on=bind.on, enabled=bind.enabled)
        return info.encode(), _fmt3(bind.distance_mm)

    def _goto_if_fields(self, stmt: GotoIfStmt, out: _Builder) -> tuple[Action, str]:
        point = io_point(stmt.io_name, "X")
        flag = out.flag_of(stmt.label)
        status = 1 if stmt.value == 1 else 0
        if status == 0:
            self.unverified(
                f"IF {stmt.io_name} == 0 GOTO {stmt.label} necesita «si la entrada está en OFF, "
                f"saltar» (pointStatus 0). Alternativa confirmada: IF {stmt.io_name} == 1 GOTO.")
        fields = dict(action=10001, flag=flag, inout=0, limit="0.000", point=point,
                      pointStatus=status, type=0)
        text = (f"IF:{io_name('X', point)}{'ON' if status else 'OFF'} Lim:0.000 "
                f"Go to Flag[{flag}]:{out.label_text.get(stmt.label, stmt.label)}")
        return fields, text

    def _emit_goto_if(self, stmt: GotoIfStmt, out: _Builder) -> None:
        fields, _text = self._goto_if_fields(stmt, out)
        out.add(**fields)

    def _emit_disabled(self, stmt: DisabledStmt, out: _Builder) -> None:
        inner = stmt.inner
        if isinstance(inner, SetOutStmt):
            point = io_point(inner.io_name, "Y")
            fields = dict(action=200, delay="0.000", isWaitInput=False, point=point,
                          pointStatus=inner.state, type=0, valveID=point)
            text = f"Output:{io_name('Y', point)}{'ON' if inner.state else 'OFF'} DLY0.000"
        elif isinstance(inner, GotoIfStmt):
            fields, text = self._goto_if_fields(inner, out)
        else:
            raise CompileError("DISABLED solo vale para SET_OUT e IF … GOTO: es lo único que "
                               "se vio desactivado en un respaldo real.")
        out.add_disabled(fields, text)

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
        flag = out.flag_of(stmt.goto) if stmt.goto is not None else -1
        # Con salto, el pad guarda el número de la etiqueta como texto
        # (`"0"`, visto 3 veces con la etiqueta 0); sin salto, -1 (entero).
        out.add(action=20000, flag=str(flag) if stmt.goto is not None else -1,
                module=str(self.modules[stmt.name]))

    def _emit_proc(self, stmt: ProcDecl) -> None:
        if stmt.params:
            raise CompileError(
                f"PROC {stmt.name}({', '.join(stmt.params)}): los módulos del pad no "
                f"reciben parámetros"
            )
        # Los POINT declarados adentro del PROC son locales: se restauran al salir.
        outer_points = dict(self.points)
        body = _Builder(stmt.name, self.options.tool, self.options.coord)
        _prescan_labels(stmt.body, body)
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


_NBSP = "&nbsp;"
# `Cmd Set:Stop`: lo único del 53000 que se vio (una vez, al principio del principal).
_CMD_STOP = dict(addr=98304, data=2561, op=0, specialType=1, type=0, typeSel=1)


def _prescan_labels(statements: list, out: _Builder) -> None:
    """Pasada previa: número de cada etiqueta del usuario, para que un salto
    hacia adelante (y el texto de un IF desactivado) ya sepan adónde van.
    Las que traen número lo conservan; las otras toman uno libre."""
    found: list[LabelStmt] = []

    def walk(stmts: list) -> None:
        for stmt in stmts:
            if isinstance(stmt, LabelStmt):
                found.append(stmt)
            elif isinstance(stmt, IfStmt):
                walk(stmt.then_body)
                walk(stmt.else_body)

    walk(statements)
    used: set[int] = set()
    for stmt in found:
        if stmt.name in out.label_flags or (stmt.number is not None and stmt.number in used):
            what = (f"la etiqueta {stmt.name!r} está declarada más de una vez"
                    if stmt.name in out.label_flags
                    else f"el número {stmt.number} de la etiqueta {stmt.name!r} ya lo usa otra")
            raise with_line(CompileError(what), stmt)
        out.label_text[stmt.name] = stmt.pad_name if stmt.pad_name is not None else stmt.name
        if stmt.number is not None:
            used.add(stmt.number)
            out.label_flags[stmt.name] = stmt.number
        else:
            out.label_flags[stmt.name] = -1
    free = 0
    for stmt in found:
        if out.label_flags[stmt.name] == -1:
            while free in used:
                free += 1
            used.add(free)
            out.label_flags[stmt.name] = free
    out.reserved_flags = used


def _collect_pad_names(statements: list) -> dict[str, str]:
    return {s.name: (s.pad_name if s.pad_name is not None else s.name)
            for s in statements if isinstance(s, ProcDecl)}


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
    _prescan_labels(ast.statements, main)
    # El comentario de arriba del principal es nuestro, salvo que el programa
    # traiga el suyo (uno importado de un respaldo del pad).
    first = next((st for st in ast.statements if not isinstance(st, (PointDecl, ProcDecl))), None)
    if not isinstance(first, CommentStmt):
        main.add(action=50000, comment="Generado por Borunte DSL")
    compiler.emit_block(ast.statements, main)
    main.add(action=60000)
    compiler.check_calls(main)

    pad_names = _collect_pad_names(ast.statements)
    modules = [Module(id=i, name=pad_names[name], actions=compiler.bodies[name])
               for name, i in compiler.modules.items()]

    lines = [list(_EMPTY_PROGRAM) for _ in range(_ACT_LINE_COUNT)]
    lines[_UNKNOWN_LINE_10] = {}
    others = {ext: b"" for ext in _EMPTY_FILES}
    others["fnc"] = (options.fnc if options.fnc is not None
                     else options.template.others.get("fnc") if options.template is not None
                     and options.template.others.get("fnc") else TEMPLATE_FNC.read_bytes())

    act = ActFile(lines)
    act.lines[MAIN_LINE] = main.actions
    act.lines[MODULES_LINE] = []
    act.modules = modules
    return PadBackup(name=options.program_name, act=act, others=others), compiler.warnings


def compile_to_pad(source: str, options: PadOptions | None = None) -> PadBackup:
    return compile_to_pad_report(source, options)[0]
