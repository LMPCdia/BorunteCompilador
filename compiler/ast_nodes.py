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
class CallStmt:
    name: str
    args: list[Expr] = field(default_factory=list)


@dataclass
class ProcDecl:
    name: str
    params: list[str]
    body: list["Stmt"]


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
)


@dataclass
class SourceProgram:
    """Raíz del AST: todas las declaraciones y sentencias de nivel superior."""
    statements: list[Stmt] = field(default_factory=list)
