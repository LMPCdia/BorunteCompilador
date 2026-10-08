"""
Primera pasada del compilador: árbol de Lark -> AST propio (compiler/ast_nodes.py).

Sin efectos de lado, sin resolución de direcciones — solo arma datos. Esto
es seguro de hacer bottom-up porque no dependemos de saber PCs todavía.
"""

from __future__ import annotations

from pathlib import Path

from lark import Lark, Token, Transformer
from lark.exceptions import VisitError

from comms.robot_client import Pose
from compiler.ast_nodes import (
    Assignment,
    BinOp,
    CallStmt,
    Const,
    CoordStmt,
    IfStmt,
    MoveStmt,
    PointDecl,
    PointLiteral,
    PointName,
    PointOffset,
    ProcDecl,
    SetOutStmt,
    SourceProgram,
    TimerDecl,
    ToolStmt,
    VarDecl,
    VarRef,
    WaitInStmt,
    WaitTimeStmt,
)

GRAMMAR_PATH = Path(__file__).parent / "grammar.lark"


def _strip_newline(args: list) -> list:
    return [a for a in args if not (isinstance(a, Token) and a.type == "NEWLINE")]


def _whole_number(value, keyword: str) -> int:
    number = float(value)
    if number != int(number) or not 0 <= number <= 65535:
        from compiler.codegen import CompileError

        raise CompileError(f"{keyword} {value}: tiene que ser un número entero entre 0 y 65535")
    return int(number)


class _AstBuilder(Transformer):
    # -- expresiones ----------------------------------------------------------

    def number(self, args):
        return float(args[0])

    def signed_number(self, args):
        return float(args[0])

    @staticmethod
    def _to_expr(value):
        if isinstance(value, (Const, VarRef, BinOp)):
            return value
        if isinstance(value, bool):
            return Const(1.0 if value else 0.0)
        if isinstance(value, (int, float)):
            return Const(float(value))
        if isinstance(value, str):
            return VarRef(value)
        raise TypeError(f"No se pudo interpretar como expresión: {value!r}")

    def eq(self, args):
        left, right = args
        return BinOp("==", self._to_expr(left), self._to_expr(right))

    def neq(self, args):
        left, right = args
        return BinOp("!=", self._to_expr(left), self._to_expr(right))

    def add(self, args):
        left, right = args
        return BinOp("+", self._to_expr(left), self._to_expr(right))

    def sub(self, args):
        left, right = args
        return BinOp("-", self._to_expr(left), self._to_expr(right))

    def NAME(self, tok):  # noqa: N802
        return str(tok)

    # Nota: NAME() de arriba se usa en varios contextos (nombre de punto,
    # parámetro de PROC, nombre de variable en una expresión, etc.). La
    # interpretación "esto es una referencia a variable" se resuelve
    # explícitamente en cada regla que la necesita (ver assignment, call_stmt),
    # no acá — porque en otros contextos ese mismo token NO es una variable.

    # -- puntos -----------------------------------------------------------------

    def world_literal(self, args):
        vals = [float(a) for a in args]
        return PointLiteral(Pose(*vals))

    def joint_literal(self, args):
        vals = [float(a) for a in args]
        return PointLiteral(Pose(*vals), frame="JOINT")

    def offset_expr(self, args):
        vals = [float(a) for a in args]
        return Pose(*vals)

    def point_ref(self, args):
        (name,) = args
        return PointName(str(name))

    def point_expr(self, args):
        (value,) = args
        return value

    def point_offset(self, args):
        base, offset = args
        return PointOffset(base, offset)

    # -- declaraciones ------------------------------------------------------------

    def point_decl(self, args):
        args = _strip_newline(args)
        name, expr = args
        return PointDecl(str(name), expr)

    def var_decl(self, args):
        args = _strip_newline(args)
        name = str(args[0])
        type_ = str(args[1])
        init = self._to_expr(args[2]) if len(args) > 2 else None
        return VarDecl(name, type_, init)

    def timer_decl(self, args):
        args = _strip_newline(args)
        name, seconds = args
        return TimerDecl(str(name), float(seconds))

    def assignment(self, args):
        args = _strip_newline(args)
        name, expr = args
        return Assignment(str(name), self._to_expr(expr))

    def move_stmt(self, args):
        args = _strip_newline(args)
        kind, point, speed = args
        return MoveStmt(str(kind), point, float(speed))

    def wait_in_stmt(self, args):
        args = _strip_newline(args)
        name = str(args[0])
        timeout = float(args[1]) if len(args) > 1 else None
        return WaitInStmt(name, timeout)

    def wait_time_stmt(self, args):
        args = _strip_newline(args)
        if args and args[0] is not None and not isinstance(args[0], Token):
            return WaitTimeStmt(seconds=float(args[0]))
        return WaitTimeStmt(until_move_done=True)

    def tool_stmt(self, args):
        args = _strip_newline(args)
        return ToolStmt(_whole_number(args[0], "TOOL"))

    def coord_stmt(self, args):
        args = _strip_newline(args)
        return CoordStmt(_whole_number(args[0], "COORD"))

    def set_out_stmt(self, args):
        args = _strip_newline(args)
        name, state = args
        return SetOutStmt(str(name), str(state) == "ON")

    def blank_line(self, args):
        # Línea vacía o solo-comentario: no produce nodo. Las reglas que
        # acumulan sentencias filtran estos None (ver _stmt_list).
        return None

    @staticmethod
    def _stmt_list(args) -> list:
        return [a for a in args if a is not None]

    def then_block(self, args):
        return self._stmt_list(args)

    def else_block(self, args):
        return self._stmt_list(args)

    def if_stmt(self, args):
        args = _strip_newline(args)
        cond = args[0]
        then_body = args[1]
        else_body = args[2] if len(args) > 2 else []
        return IfStmt(cond, then_body, else_body)

    def call_stmt(self, args):
        args = _strip_newline(args)
        args = [a for a in args if a is not None]
        name = str(args[0])
        call_args = [self._to_expr(a) for a in args[1:]]
        return CallStmt(name, call_args)

    def proc_body(self, args):
        return self._stmt_list(args)

    def proc_decl(self, args):
        args = _strip_newline(args)
        args = [a for a in args if a is not None]
        name = str(args[0])
        # args puede incluir params (NAME sueltos) antes del proc_body (una lista)
        body = args[-1]
        params = [str(p) for p in args[1:-1]]
        return ProcDecl(name, params, body)

    def start(self, args):
        return SourceProgram(self._stmt_list(args))

    def statement(self, args):
        (stmt,) = args
        return stmt


def build_ast(source: str) -> SourceProgram:
    grammar_text = GRAMMAR_PATH.read_text(encoding="utf-8")
    parser = Lark(grammar_text, parser="lalr")
    # Toda sentencia de la gramática termina en NEWLINE, así que un archivo sin
    # salto de línea final moría con "Unexpected token $END". Es exactamente lo
    # que pasa escribiendo en el editor de la GUI sin apretar Enter al final,
    # así que se tolera acá en vez de hacérselo notar al usuario.
    if source and not source.endswith("\n"):
        source += "\n"
    tree = parser.parse(source)
    try:
        return _AstBuilder().transform(tree)
    except VisitError as e:
        # Lark envuelve lo que tira el Transformer: un CompileError tiene que
        # llegar como tal, con su mensaje, no como "Error trying to process...".
        from compiler.codegen import CompileError

        if isinstance(e.orig_exc, CompileError):
            raise e.orig_exc from None
        raise
