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
class InputRef:
    """Estado de una entrada FISICA del robot: INPUT(X012).

    Distinto de VarRef, que es una variable interna. El backend .act solo sabe
    condicionar sobre esto (accion nativa 10001); no hay opcode confirmado para
    comparar variables internas.
    """
    name: str


@dataclass
class StateConst:
    """ON / OFF como valor de expresion, para IF INPUT(X) == ON."""
    on: bool


@dataclass
class BinOp:
    op: str  # "==", "!=", "+", "-"
    left: "Expr"
    right: "Expr"


Expr = Const | VarRef | InputRef | StateConst | BinOp

# --- Puntos ------------------------------------------------------------------


@dataclass
class PointLiteral:
    pose: Pose


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
class MoveStmt:
    kind: str  # "MOVEJ" | "MOVEL"
    point: PointExpr
    speed: float


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
class IfStmt:
    cond: Expr
    then_body: list["Stmt"]
    else_body: list["Stmt"] = field(default_factory=list)


@dataclass
class BaseStmt:
    """Selecciona la coordenada de base. Accion nativa 800."""
    coord_id: int


@dataclass
class ToolStmt:
    """Selecciona la herramienta. Accion nativa 801."""
    tool_id: int


@dataclass
class CallStmt:
    name: str
    args: list[Expr] = field(default_factory=list)


@dataclass
class ProcDecl:
    name: str
    params: list[str]
    body: list["Stmt"]
    #: Id explicito de la entrada en la biblioteca del robot: PROC HOME(id=1).
    #: Lo necesita el backend .act; el backend de bytecode del PLC lo ignora.
    #: Son mutuamente excluyentes: un PROC tiene params o tiene id, no los dos.
    proc_id: int | None = None


Stmt = (
    PointDecl
    | VarDecl
    | TimerDecl
    | Assignment
    | MoveStmt
    | WaitInStmt
    | WaitTimeStmt
    | SetOutStmt
    | IfStmt
    | CallStmt
    | ProcDecl
    | BaseStmt
    | ToolStmt
)


@dataclass
class SourceProgram:
    """Raíz del AST: todas las declaraciones y sentencias de nivel superior."""
    statements: list[Stmt] = field(default_factory=list)
