"""
Segunda salida del compilador: AST -> respaldo del pad (`HCBackupRobot_*.zip`)
que el Borunte importa por pendrive y ejecuta él solo, sin PC ni PLC.

El formato está deducido de un respaldo real (`docs/PAD_FORMAT.md`) y **no se
probó todavía importando en el pad**. Por eso este generador es conservador:
lo que no sabemos expresar en el formato del pad es un `CompileError` claro,
nunca una adivinanza que el robot ejecute.

Qué se traduce y a qué acción del pad:

| DSL                         | Pad                                  | Confianza |
|-----------------------------|--------------------------------------|-----------|
| `MOVEJ <JOINT> SPEED n`     | `4`, ángulos de eje                  | Deducido  |
| `MOVEL <WORLD> SPEED n`     | `10`, X,Y,Z,U,V,W                    | Deducido  |
| `SET_OUT(Ynnn, ON/OFF)`     | `200`                                | Deducido  |
| `WAIT n s`                  | `100` con `type: 100`                | Hipótesis |
| `PROC` / llamada            | módulo + `20000`                     | Deducido  |
| `IF Xnnn == 0 THEN`         | `10001` "si X ON saltar al fin"      | Deducido  |
| `IF Xnnn == 1 THEN`         | `10001` con `pointStatus: 0` (OFF)   | Hipótesis |
| `TOOL n` / `COORD n`        | `801` / `800` + `toolCoord` de cada movimiento | Deducido |

`TOOL` y `COORD` valen para los movimientos que siguen, hasta el final del
programa o del PROC donde están; cada PROC arranca con los valores de
`PadOptions`. No se permiten dentro de un IF: el valor de los movimientos que
siguen al IF dependería de si se entró o no, y eso se resuelve al compilar.

Lo que NO se traduce (error de compilación, con el motivo):

- `MOVEJ` a un punto `WORLD`, o `MOVEL` a un punto `JOINT`. El MOVEJ del pad
  guarda ángulos de eje: mandarle X/Y/Z lo haría moverse a cualquier lado.
- `VAR`, asignaciones e `IF` sobre variables: no sabemos cómo guarda el pad
  sus variables (el `.variables` del respaldo estaba vacío).
- `ELSE`: el pad no tiene un salto incondicional que hayamos visto.
- `WAIT_IN`: no hay ningún ejemplo de espera de entrada en el respaldo.
- `PROC` con parámetros: los módulos del pad no tienen.
"""

from __future__ import annotations

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
from compiler.codegen import CompileError
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

_PROGRAM_NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]+$")


@dataclass
class PadOptions:
    """Lo que el DSL todavía no expresa y hay que decidir al exportar."""

    program_name: str = "BorunteDSL"
    # Herramienta y sistema de coordenadas de todos los movimientos. 0/0 es
    # brida y base (hipótesis: es lo que usa el pad cuando no se eligió nada).
    tool: int = 0
    coord: int = 0
    # Respaldo exportado del MISMO robot para copiar lo que no sabemos generar
    # (el `.fnc` y las líneas desconocidas del `.act`). Sin él se usa el del
    # respaldo real que se analizó.
    template: PadBackup | None = None


def _fmt3(value: float) -> str:
    # `or 0.0` evita "-0.000", que el pad nunca escribe.
    return f"{(round(value, 3) or 0.0):.3f}"


def io_point(name: str, prefix: str) -> int:
    """`Y034` -> 20. Numeración octal que empieza en `010` (hipótesis de
    `docs/PAD_FORMAT.md`). Es la inversa de `pad.listing.io_name`."""
    match = re.fullmatch(r"([XYxy])([0-7]+)", name)
    if not match or match.group(1).upper() != prefix:
        kind = "una entrada (X)" if prefix == "X" else "una salida (Y)"
        raise CompileError(
            f"{name!r} no es {kind} del pad. Se escriben como en el pad, con "
            f"dígitos octales: {prefix}010, {prefix}011, ... {prefix}017, {prefix}020, ..."
        )
    point = int(match.group(2), 8) - 8
    if point < 0:
        raise CompileError(f"{name!r}: las E/S del pad empiezan en {prefix}010")
    return point


def _sorted(action: dict[str, Any]) -> Action:
    # El pad escribe las claves en orden alfabético.
    return dict(sorted(action.items()))


class _Builder:
    """Acciones de UN programa (el principal o un módulo), con sus etiquetas."""

    def __init__(self, tool: int = 0, coord: int = 0) -> None:
        self.actions: list[Action] = []
        self._next_flag = 0
        self.tool = tool
        self.coord = coord
        self.if_depth = 0

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

    # -- puntos ---------------------------------------------------------------

    def resolve_point(self, node) -> _Point:
        if isinstance(node, PointLiteral):
            return _Point(node.pose, node.frame)
        if isinstance(node, PointName):
            if node.name not in self.points:
                raise CompileError(f"Punto no definido: {node.name!r}")
            return self.points[node.name]
        if isinstance(node, PointOffset):
            base = self.resolve_point(node.base)
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
        if isinstance(stmt, PointDecl):
            self.points[stmt.name] = self.resolve_point(stmt.expr)
        elif isinstance(stmt, TimerDecl):
            pass  # igual que en el bytecode: es metadata
        elif isinstance(stmt, (ToolStmt, CoordStmt)):
            word = "TOOL" if isinstance(stmt, ToolStmt) else "COORD"
            if out.if_depth:
                raise CompileError(
                    f"{word} dentro de un IF no se puede exportar: ponelo antes del IF "
                    f"(o dentro de un PROC que llame el IF)."
                )
            if isinstance(stmt, ToolStmt):
                out.tool = stmt.number
                out.add(action=801, toolID=stmt.number)
            else:
                out.coord = stmt.number
                out.add(action=800, coordID=stmt.number)
        elif isinstance(stmt, MoveStmt):
            self._emit_move(stmt, out)
        elif isinstance(stmt, SetOutStmt):
            point = io_point(stmt.io_name, "Y")
            out.add(action=200, delay="0.000", isWaitInput=False, point=point,
                    pointStatus=stmt.state, type=0, valveID=point)
        elif isinstance(stmt, WaitTimeStmt):
            if stmt.until_move_done or stmt.seconds is None:
                return  # el pad no avanza hasta terminar cada movimiento
            if stmt.seconds <= 0:
                raise CompileError(f"WAIT {stmt.seconds}s: el tiempo tiene que ser positivo")
            out.add(action=100, customName="", isUnlimit=False, limit=_fmt3(stmt.seconds),
                    point=0, pointStatus=0, type=100)
        elif isinstance(stmt, IfStmt):
            self._emit_if(stmt, out)
        elif isinstance(stmt, CallStmt):
            self._emit_call(stmt, out)
        elif isinstance(stmt, ProcDecl):
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
        if not 0 < stmt.speed <= 100:
            raise CompileError(f"SPEED {stmt.speed:g}: en el pad es un % entre 0 y 100")
        p = point.pose
        pos = {f"m{i}": _fmt3(v) for i, v in enumerate((p.a, p.b, p.c, p.d, p.e, p.f))}
        pos.update(m6="0.000", m7="0.000")
        name = stmt.point.name if isinstance(stmt.point, PointName) else ""
        out.add(
            action=4 if stmt.kind == "MOVEJ" else 10,
            bindIOInfo=0, ckStatus="63", customName=name, delay="0.000", distance="0.000",
            passTrans=0, points=[{"pointName": "", "pos": pos}], quotePoint=[0, 0, 0],
            relativeType=0, smooth=0, speed=f"{stmt.speed:.1f}",
            toolCoord=(out.tool << 16) | out.coord,
        )

    def _emit_if(self, stmt: IfStmt, out: _Builder) -> None:
        if stmt.else_body:
            raise CompileError(
                "ELSE todavía no se puede exportar al pad: no conocemos un salto "
                "incondicional en su formato. Usá dos IF (X == 1 y X == 0)."
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
                "IF X010 == 1 THEN ... o IF X010 == 0 THEN ..."
            )
        point = io_point(cond.left.name, "X")
        # Se saltea el bloque cuando la condición es FALSA: si se pregunta
        # "== 1", el salto es con la entrada en OFF, y al revés.
        skip_when_on = cond.right.value == 0
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
        out.add(action=20000, flag=-1, module=str(self.modules[stmt.name]))

    def _emit_proc(self, stmt: ProcDecl) -> None:
        if stmt.params:
            raise CompileError(
                f"PROC {stmt.name}({', '.join(stmt.params)}): los módulos del pad no "
                f"reciben parámetros"
            )
        body = _Builder(self.options.tool, self.options.coord)
        self._emit_preamble(body)
        self.emit_block(stmt.body, body)
        body.add(action=20001)
        self.bodies[stmt.name] = body.actions

    def _emit_preamble(self, out: _Builder) -> None:
        out.add(action=800, coordID=self.options.coord)
        out.add(action=801, toolID=self.options.tool)


def _collect_modules(statements: list) -> dict[str, int]:
    """Pasada 0: id de módulo para cada PROC, en orden de declaración, para
    que las llamadas hacia adelante ya sepan a qué módulo ir."""
    modules: dict[str, int] = {}

    def walk(stmts: list) -> None:
        for stmt in stmts:
            if isinstance(stmt, ProcDecl):
                if stmt.name in modules:
                    raise CompileError(f"PROC {stmt.name!r} declarado más de una vez")
                modules[stmt.name] = len(modules)
                walk(stmt.body)
            elif isinstance(stmt, IfStmt):
                walk(stmt.then_body)
                walk(stmt.else_body)

    walk(statements)
    return modules


def compile_to_pad(source: str, options: PadOptions | None = None) -> PadBackup:
    options = options or PadOptions()
    if not _PROGRAM_NAME_RE.match(options.program_name):
        raise CompileError(
            f"Nombre de programa {options.program_name!r}: usá solo letras sin "
            f"acentos, números, '_', '-' y '.'"
        )

    ast = build_ast(source)
    compiler = _Compiler(options, modules=_collect_modules(ast.statements))
    main = _Builder(options.tool, options.coord)
    main.add(action=50000, comment="Generado por Borunte DSL")
    compiler._emit_preamble(main)
    compiler.emit_block(ast.statements, main)
    main.add(action=60000)

    modules = [Module(id=i, name=name, actions=compiler.bodies[name])
               for name, i in compiler.modules.items()]

    if options.template is not None:
        lines = list(options.template.act.lines)
        others = dict(options.template.others)
    else:
        lines = [list(_EMPTY_PROGRAM) for _ in range(_ACT_LINE_COUNT)]
        lines[_UNKNOWN_LINE_10] = {}
        others = {"fnc": TEMPLATE_FNC.read_bytes()}
        others.update({ext: b"" for ext in ("counters", "palletStyle", "timers", "variables")})

    act = ActFile(lines)
    act.lines[MAIN_LINE] = main.actions
    act.lines[MODULES_LINE] = []
    act.modules = modules
    return PadBackup(name=options.program_name, act=act, others=others)
