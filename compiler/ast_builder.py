"""
Primera pasada del compilador: árbol de Lark -> AST propio (compiler/ast_nodes.py).

Sin efectos de lado, sin resolución de direcciones — solo arma datos. Esto
es seguro de hacer bottom-up porque no dependemos de saber PCs todavía.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from lark import Lark, Token, Transformer, v_args
from lark.exceptions import (
    UnexpectedCharacters,
    UnexpectedEOF,
    UnexpectedInput,
    UnexpectedToken,
    VisitError,
)

from comms.robot_client import Pose
from compiler.ast_nodes import (
    Assignment,
    BinOp,
    BindOpt,
    CallStmt,
    CmdStopStmt,
    CommentStmt,
    Const,
    CoordStmt,
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
    SourceProgram,
    TimerDecl,
    ToolStmt,
    VarDecl,
    VarRef,
    WaitInStmt,
    WaitTimeStmt,
)

GRAMMAR_PATH = Path(__file__).parent / "grammar.lark"


class _PadName(str):
    """El texto de un `AS "..."` (para no confundirlo con un NAME)."""


class _GotoTarget(str):
    """La etiqueta de un `THEN GOTO etiqueta`."""


def _unquote(token) -> str:
    # ESCAPED_STRING es una cadena de JSON (comillas dobles y \\ escapes).
    return json.loads(str(token))


def _compile_error(message: str):
    from compiler.codegen import CompileError

    return CompileError(message)


def _strip_newline(args: list) -> list:
    return [a for a in args if not (isinstance(a, Token) and a.type == "NEWLINE")]


def _whole(value, keyword: str, low: int, high: int) -> int:
    number = float(value)
    if not math.isfinite(number) or number != int(number) or not low <= number <= high:
        raise _compile_error(f"{keyword} {number:g}: tiene que ser un número entero entre "
                             f"{low} y {high}")
    return int(number)


def _whole_number(value, keyword: str) -> int:
    number = float(value)
    if not math.isfinite(number) or number != int(number) or not 0 <= number <= 15:
        from compiler.codegen import CompileError

        raise CompileError(f"{keyword} {number:g}: tiene que ser un número entero entre 0 y 15 "
                           f"(rango conservador: en el pad se vieron hasta el 2)")
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

    def as_clause(self, args):
        return _PadName(_unquote(args[0]))

    def point_decl(self, args):
        args = _strip_newline(args)
        name, expr, pad_name = args
        return PointDecl(str(name), expr, str(pad_name) if pad_name is not None else None)

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

    def move_kind(self, args):
        return str(args[0])

    def var_type(self, args):
        return str(args[0])

    def state(self, args):
        return str(args[0])

    # -- opciones de un movimiento: cada una devuelve (clave, valor) -------------------

    def smooth_opt(self, args):
        return ("smooth", _whole(args[0], "SMOOTH", 0, 9))

    def delay_opt(self, args):
        seconds = float(args[0])
        if not math.isfinite(seconds) or not 0 <= seconds <= 3600:
            raise _compile_error(f"DELAY {seconds:g}: tiene que estar entre 0 y 3600 segundos")
        return ("delay_s", seconds)

    def tool_opt(self, args):
        return ("tool", _whole_number(args[0], "TOOL"))

    def coord_opt(self, args):
        return ("coord", _whole_number(args[0], "COORD"))

    @staticmethod
    def _bind(end: bool, args) -> tuple:
        name, state, mm, disabled = args
        distance = float(mm)
        if not math.isfinite(distance) or not 0 <= distance <= 10000:
            raise _compile_error(f"distancia {distance:g} mm: tiene que estar entre 0 y 10000")
        return ("bind", BindOpt(end, str(name), str(state) == "ON", distance, disabled is None))

    def near_end_opt(self, args):
        return self._bind(True, args)

    def away_start_opt(self, args):
        return self._bind(False, args)

    def bind_disabled(self, args):
        return str(args[0])

    def move_stmt(self, args):
        args = _strip_newline(args)
        kind, point, speed, *options = args
        fields: dict = {}
        for key, value in options:
            if key in fields:
                names = {"smooth": "SMOOTH", "delay_s": "DELAY", "tool": "TOOL", "coord": "COORD",
                         "bind": "NEAR_END / AWAY_START"}
                raise _compile_error(f"{names[key]} repetido en el mismo movimiento")
            fields[key] = value
        return MoveStmt(str(kind), point, float(speed), **fields)

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

    def label_stmt(self, args):
        args = _strip_newline(args)
        name, number, pad_name = args
        if number is not None:
            number = _whole(number, "LABEL", 0, 9999)
        return LabelStmt(str(name), number, str(pad_name) if pad_name is not None else None)

    def goto_if_stmt(self, args):
        args = _strip_newline(args)
        cond, label = args
        if isinstance(cond, BinOp) and cond.op == "==" and isinstance(cond.right, VarRef):
            cond = BinOp("==", cond.right, cond.left)
        if not (isinstance(cond, BinOp) and cond.op == "==" and isinstance(cond.left, VarRef)
                and isinstance(cond.right, Const) and cond.right.value in (0, 1)):
            raise _compile_error("IF … GOTO solo pregunta por una entrada: "
                                 "IF X030 == 1 GOTO etiqueta")
        return GotoIfStmt(cond.left.name, int(cond.right.value), str(label))

    def disabled_stmt(self, args):
        args = _strip_newline(args)
        (inner,) = args
        return DisabledStmt(inner)

    def comment_stmt(self, args):
        args = _strip_newline(args)
        return CommentStmt(_unquote(args[0]))

    def cmd_stop_stmt(self, args):
        return CmdStopStmt()

    def goto_clause(self, args):
        return _GotoTarget(str(args[0]))

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
        goto = next((str(a) for a in args if isinstance(a, _GotoTarget)), None)
        call_args = [self._to_expr(a) for a in args[1:] if not isinstance(a, _GotoTarget)]
        return CallStmt(name, call_args, goto)

    def proc_body(self, args):
        return self._stmt_list(args)

    def proc_decl(self, args):
        args = _strip_newline(args)
        args = [a for a in args if a is not None]
        name = str(args[0])
        # args puede incluir params (NAME sueltos) y un `AS "..."` antes del
        # proc_body (una lista)
        body = args[-1]
        middle = args[1:-1]
        pad_name = next((str(a) for a in middle if isinstance(a, _PadName)), None)
        params = [str(p) for p in middle if not isinstance(p, _PadName)]
        return ProcDecl(name, params, body, pad_name)

    def start(self, args):
        return SourceProgram(self._stmt_list(args))

    @v_args(meta=True)
    def statement(self, meta, children):
        (stmt,) = children
        # Línea de la sentencia, para los mensajes de error (no es parte del
        # AST "de verdad": los nodos no la declaran como campo).
        stmt.line = getattr(meta, "line", None)
        return stmt


# Cómo se llaman, para el usuario, los terminales que Lark pone en "se esperaba".
_TOKEN_NAMES = {
    "NEWLINE": "fin de línea", "LPAR": "«(»", "RPAR": "«)»", "COMMA": "«,»",
    "EQUAL": "«=»", "COLON": "«:»", "PLUS": "«+»", "MINUS": "«-»",
    "NAME": "un nombre", "SIGNED_NUMBER": "un número", "$END": "fin del archivo",
    "STRING": "un texto entre comillas",
    "S": "«s»",
}

# Palabras clave del lenguaje (siempre en mayúscula).
_KEYWORDS = {
    "POINT", "VAR", "TIMER", "PROC", "ENDPROC", "IF", "THEN", "ELSE", "ENDIF", "WAIT", "WAIT_IN",
    "UNTIL", "MOVE_DONE", "SET_OUT", "SPEED", "MOVEJ", "MOVEL", "WORLD", "JOINT", "OFFSET",
    "TOOL", "COORD", "ON", "OFF", "INT", "REAL", "BOOL",
    "SMOOTH", "DELAY", "NEAR_END", "AWAY_START", "DISABLED", "LABEL", "GOTO", "COMMENT", "CMD",
    "STOP", "AS",
}


def _syntax_message(e: UnexpectedInput, source: str) -> str:
    line = getattr(e, "line", None)
    text = source.splitlines()[line - 1].strip() if line and line <= len(source.splitlines()) else ""
    where = f"Línea {line}" if line else "Error de sintaxis"
    if isinstance(e, UnexpectedEOF) or (isinstance(e, UnexpectedToken) and e.token.type == "$END"):
        return "El programa termina antes de tiempo: ¿falta un ENDIF o un ENDPROC?"
    if isinstance(e, UnexpectedCharacters):
        return f"{where}: carácter inesperado «{e.char}» en «{text}»"
    if isinstance(e, UnexpectedToken):
        token = str(e.token)
        found = "fin de línea" if e.token.type == "NEWLINE" else f"«{token}»"
        history = getattr(e, "token_history", None) or []
        previous = str(history[-1]) if history else ""
        if previous and previous != previous.upper() and previous.upper() in _KEYWORDS:
            return (f"{where}: «{previous}»: las palabras clave van en mayúscula "
                    f"(«{previous.upper()}»).")
        if previous in _KEYWORDS and e.token.type == "NEWLINE" and history[-1].type == "NAME":
            return f"{where}: «{previous}» no va acá. ¿Sobra, o falta abrir el bloque (IF/PROC)?"
        raw_line = source.splitlines()[line - 1] if line and line <= len(source.splitlines()) else ""
        col = (getattr(e, "column", 0) or 0) - 1
        after_digit = 0 < col <= len(raw_line) and raw_line[col - 1].isdigit()
        if token == "," and "COMMA" not in e.expected and after_digit:
            return f"{where}: los decimales van con punto, no con coma («{text}»)."
        if e.expected == {"COMMA"} or (e.expected == {"COMMA", "RPAR"} and token in _KEYWORDS):
            return f"{where}: falta una coma «,» en «{text}»."
        if "RPAR" in e.expected and e.token.type != "COMMA":
            return f"{where}: falta cerrar un paréntesis «)» en «{text}»."
        if token in _KEYWORDS:
            # Una palabra clave donde no va: casi siempre falta cerrar un bloque.
            extra = ""
            if token in ("ENDPROC", "ELSE") or token == "ENDIF":
                extra = " ¿Falta un ENDIF antes, o sobra este?"
            return f"{where}: «{token}» no va acá («{text}»).{extra}"
        if token.upper() in _KEYWORDS and token != token.upper():
            return (f"{where}: «{token}»: las palabras clave van en mayúscula "
                    f"(«{token.upper()}»).")
        expected = {_TOKEN_NAMES.get(t, f"«{t}»") for t in e.expected if not t.startswith("__")}
        if "«)»" in expected and e.token.type != "NEWLINE":
            expected = {"«)»", "«,»"} & expected  # dentro de un paréntesis, lo que importa
        hint = f" Se esperaba: {', '.join(sorted(expected)[:6])}." if expected else ""
        return f"{where}: no se esperaba {found} en «{text}».{hint}"
    return f"{where}: error de sintaxis en «{text}»"


def build_ast(source: str) -> SourceProgram:
    grammar_text = GRAMMAR_PATH.read_text(encoding="utf-8")
    parser = Lark(grammar_text, parser="lalr", propagate_positions=True)
    # Toda sentencia de la gramática termina en NEWLINE, así que un archivo sin
    # salto de línea final moría con "Unexpected token $END". Es exactamente lo
    # que pasa escribiendo en el editor de la GUI sin apretar Enter al final,
    # así que se tolera acá en vez de hacérselo notar al usuario.
    if source and not source.endswith("\n"):
        source += "\n"
    try:
        tree = parser.parse(source)
    except UnexpectedInput as e:
        from compiler.codegen import CompileError

        raise CompileError(_syntax_message(e, source), line=getattr(e, "line", None)) from None
    try:
        return _AstBuilder().transform(tree)
    except VisitError as e:
        # Lark envuelve lo que tira el Transformer: un CompileError tiene que
        # llegar como tal, con su mensaje, no como "Error trying to process...".
        from compiler.codegen import CompileError

        if isinstance(e.orig_exc, CompileError):
            raise e.orig_exc from None
        raise
