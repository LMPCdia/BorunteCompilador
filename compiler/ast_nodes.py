"""
AST propio del DSL. Estas son estructuras de datos puras — no hay lógica de
emisión de bytecode acá (eso vive en compiler/codegen.py). Separar esto de
Lark permite que el segundo paso (emisión) recorra el árbol en el orden que
necesite, sabiendo de antemano toda la estructura — lo que resuelve el
problema de saltos hacia adelante que tenía el Transformer de una sola
pasada (ver README.md, sección "Limitación conocida").
"""

from __future__ import annotations

from dataclasses import dataclass, field

from comms.robot_client import Pose

# --- Expresiones -----------------------------------------------------------


@dataclass
class Const:
    value: float


@dataclass
class VarRef:
    name: str


@dataclass
class BinOp:
    op: str  # "==", "!=", "+", "-"
    left: "Expr"
    right: "Expr"


Expr = Const | VarRef | BinOp

# --- Puntos ------------------------------------------------------------------


@dataclass
class PointLiteral:
    pose: Pose
    # "WORLD" (X,Y,Z,U,V,W) o "JOINT" (J1..J6). La VM de referencia no los
    # distingue; el generador del pad sí, porque MOVEJ y MOVEL esperan cosas
    # distintas (ver compiler/pad_codegen.py).
    frame: str = "WORLD"


@dataclass
class PointName:
    name: str


@dataclass
class PointOffset:
    base: "PointExpr"
    offset: Pose


PointExpr = PointLiteral | PointName | PointOffset

# --- Sentencias --------------------------------------------------------------


@dataclass
class PointDecl:
    name: str
    expr: PointExpr
    pad_name: str | None = None     # `AS "..."`: el nombre que muestra el pad


@dataclass
class VarDecl:
    name: str
    type: str
    init: Expr | None = None


@dataclass
class TimerDecl:
    name: str
    seconds: float


@dataclass
class Assignment:
    name: str
    expr: Expr


@dataclass
class BindOpt:
    """Salida que cambia según la distancia al punto (pad/bindio.py)."""
    end: bool            # NEAR_END (punto final) / AWAY_START (punto inicial)
    io_name: str
    on: bool
    distance_mm: float
    enabled: bool = True


@dataclass
class MoveStmt:
    kind: str  # "MOVEJ" | "MOVEL"
    point: PointExpr
    speed: float
    # Opciones del movimiento; None = no se dijo (valen los del estado).
    smooth: int | None = None
    delay_s: float | None = None
    tool: int | None = None
    coord: int | None = None
    bind: BindOpt | None = None


@dataclass
class WaitInStmt:
    io_name: str
    timeout_s: float | None = None


@dataclass
class WaitTimeStmt:
    seconds: float | None = None
    until_move_done: bool = False


@dataclass
class SetOutStmt:
    io_name: str
    state: bool


@dataclass
class ToolStmt:
    """`TOOL n`: herramienta del pad para los movimientos que siguen."""
    number: int


@dataclass
class CoordStmt:
    """`COORD n`: sistema de coordenadas del pad para los movimientos que siguen."""
    number: int


@dataclass
class IfStmt:
    cond: Expr
    then_body: list["Stmt"]
    else_body: list["Stmt"] = field(default_factory=list)


@dataclass
class CallStmt:
    name: str
    args: list[Expr] = field(default_factory=list)
    goto: str | None = None         # `THEN GOTO etiqueta`: llamar y después saltar


@dataclass
class LabelStmt:
    name: str
    number: int | None = None       # el que guarda el pad (None: uno libre)
    pad_name: str | None = None


@dataclass
class GotoIfStmt:
    """`IF Xnnn == 1 GOTO etiqueta`: salta si la entrada está en `value`."""
    io_name: str
    value: int
    label: str


@dataclass
class DisabledStmt:
    """Una acción desactivada en el pad (SET_OUT o IF…GOTO)."""
    inner: "SetOutStmt | GotoIfStmt"


@dataclass
class CommentStmt:
    text: str


@dataclass
class CmdStopStmt:
    pass


@dataclass
class ProcDecl:
    name: str
    params: list[str]
    body: list["Stmt"]
    pad_name: str | None = None     # `AS "..."`: el nombre que muestra el pad


Stmt = (
    PointDecl
    | VarDecl
    | TimerDecl
    | Assignment
    | MoveStmt
    | WaitInStmt
    | WaitTimeStmt
    | SetOutStmt
    | ToolStmt
    | CoordStmt
    | IfStmt
    | CallStmt
    | ProcDecl
    | LabelStmt
    | GotoIfStmt
    | DisabledStmt
    | CommentStmt
    | CmdStopStmt
)


@dataclass
class SourceProgram:
    """Raíz del AST: todas las declaraciones y sentencias de nivel superior."""
    statements: list[Stmt] = field(default_factory=list)
