"""
Compilador: texto DSL -> Program (runtime/bytecode.py).

Implementación de una sola pasada usando un Transformer de Lark, con
backpatching para saltos hacia adelante (IF/ELSE, llamadas a PROC definidos
más abajo en el archivo).

Alcance v0.1 (a propósito limitado — ver docs/INSTRUCTION_SET.md):
  - Los offsets de puntos se resuelven en tiempo de COMPILACIÓN (no en
    runtime). Offsets calculados dinámicamente quedan para una v0.2.
  - Nombres de E/S tipo "X010" / "Y10" se interpretan tomando la parte
    numérica; no hay validación de que ese número exista en el PLC real.
"""

from __future__ import annotations

from pathlib import Path

from lark import Lark, Token, Transformer, v_args

from comms.robot_client import Pose
from runtime.bytecode import (
    ADD_VAR,
    CALL,
    END,
    JUMP,
    JUMP_IF_ZERO,
    MOVEJ,
    MOVEL,
    RET,
    SET_OUT,
    SET_VAR,
    WAIT_IN,
    WAIT_TIME,
    Instruction,
    Program,
)

GRAMMAR_PATH = Path(__file__).parent / "grammar.lark"


def _strip_newline(args: list) -> list:
    """Lark no filtra automáticamente terminales nombrados como NEWLINE (solo
    filtra literales anónimos como "="). Todas las reglas de sentencia
    terminan en NEWLINE por gramática, así que lo sacamos acá."""
    return [a for a in args if not (isinstance(a, Token) and a.type == "NEWLINE")]


class CompileError(Exception):
    pass


def _io_number(name: str) -> int:
    """'X010' -> 10, 'Y10' -> 10. Simplificación v0.1 (ver docstring)."""
    digits = "".join(c for c in name if c.isdigit())
    if not digits:
        raise CompileError(f"No se pudo interpretar el número de E/S en {name!r}")
    return int(digits)


class _Compiler(Transformer):
    """
    Recorre el árbol y va emitiendo Instruction directamente en self.program.
    No es un Transformer "puro" (tiene efectos de lado) a propósito — para
    esta v0.1 es más simple que separar en AST + codegen como dos pasadas.
    """

    def __init__(self) -> None:
        super().__init__()
        self.program = Program()
        self._next_var_index = 0
        self._pending_proc_patches: list[tuple[int, str]] = []  # (índice de instrucción CALL, nombre)

    # -- resolución final, llamar después de transform() ------------------

    def finalize(self) -> Program:
        self.program.instructions.append(Instruction(END))
        for instr_index, name in self._pending_proc_patches:
            if name not in self.program.proc_addresses:
                raise CompileError(f"Llamada a procedimiento no definido: {name!r}")
            self.program.instructions[instr_index].b = self.program.proc_addresses[name]
        return self.program

    def _var_index(self, name: str) -> int:
        if name not in self.program.var_names:
            self.program.var_names[name] = self._next_var_index
            self._next_var_index += 1
        return self.program.var_names[name]

    def _emit(self, instr: Instruction) -> int:
        self.program.instructions.append(instr)
        return len(self.program.instructions) - 1

    def _register_point(self, pose: Pose, name: str | None = None) -> int:
        index = len(self.program.points)
        self.program.points.append(pose)
        if name:
            self.program.point_names[name] = index
        return index

    # -- reglas de la gramática --------------------------------------------

    def number(self, args):
        return float(args[0])

    def signed_number(self, args):
        return float(args[0])

    def world_literal(self, args):
        vals = [float(a) for a in args]
        return Pose(*vals)

    def point_expr(self, args):
        # Caso "point_expr: world_literal" sin alias -> Lark envuelve el
        # resultado en un nodo point_expr con un solo hijo; lo desenvolvemos.
        (pose,) = args
        return pose

    def offset_expr(self, args):
        vals = [float(a) for a in args]
        return Pose(*vals)

    def point_ref(self, args):
        (name,) = args
        name = str(name)
        if name not in self.program.point_names:
            raise CompileError(f"Punto no definido: {name!r}")
        index = self.program.point_names[name]
        return self.program.points[index]

    def point_offset(self, args):
        base, offset = args
        return Pose(*(b + o for b, o in zip(
            (base.a, base.b, base.c, base.d, base.e, base.f),
            (offset.a, offset.b, offset.c, offset.d, offset.e, offset.f),
        )))

    def point_decl(self, args):
        args = _strip_newline(args)
        name, pose = args
        self._register_point(pose, name=str(name))

    def var_decl(self, args):
        args = _strip_newline(args)
        name = str(args[0])
        self._var_index(name)
        # inicialización opcional: v0.1 solo soporta constantes numéricas
        if len(args) > 2:
            value = args[2]
            self._emit(Instruction(SET_VAR, a=self._var_index(name), b=int(value)))

    def timer_decl(self, args):
        args = _strip_newline(args)
        pass  # v0.1: los TIMER se resuelven inline en WAIT <n>s, esto es solo metadata

    def move_stmt(self, args):
        args = _strip_newline(args)
        kind, pose, speed = args
        index = self._register_point(pose)
        opcode = MOVEJ if str(kind) == "MOVEJ" else MOVEL
        self._emit(Instruction(opcode, b=index, d=int(speed)))

    def wait_in_stmt(self, args):
        args = _strip_newline(args)
        name = str(args[0])
        timeout_s = float(args[1]) if len(args) > 1 else 0
        self._emit(Instruction(WAIT_IN, a=_io_number(name), b=int(timeout_s * 1000)))

    def wait_time_stmt(self, args):
        args = _strip_newline(args)
        if args and args[0] is not None:
            ms = int(float(args[0]) * 1000)
            self._emit(Instruction(WAIT_TIME, b=ms))
        # "WAIT UNTIL MOVE_DONE" no emite nada: MOVEJ/MOVEL ya bloquean (ver runtime/vm.py)

    def set_out_stmt(self, args):
        args = _strip_newline(args)
        name, state = args
        state_val = 1 if str(state) == "ON" else 0
        self._emit(Instruction(SET_OUT, a=_io_number(name), b=state_val))

    def if_stmt(self, args):
        # Nota: por cómo Lark arma el árbol acá (statement* variádico dentro
        # de la regla), el manejo de bloques then/else se resuelve mejor con
        # una gramática que use bloques explícitos. v0.1: soportado a nivel
        # de diseño en INSTRUCTION_SET.md (JUMP_IF_ZERO/JUMP); la resolución
        # completa de bloques queda para cuando se separe en AST + codegen
        # de dos pasadas (ver TODO en docs/ARCHITECTURE.md).
        raise CompileError(
            "IF/ELSE todavía no tiene codegen completo en v0.1 — "
            "ver TODO en compiler/codegen.py::if_stmt"
        )

    def call_stmt(self, args):
        args = _strip_newline(args)
        name = str(args[0])
        idx = self._emit(Instruction(CALL, b=0, comment=f"call {name}"))
        self._pending_proc_patches.append((idx, name))

    def proc_decl(self, args):
        raise CompileError(
            "PROC todavía no tiene codegen completo en v0.1 (requiere resolver "
            "la dirección de inicio antes de que el cuerpo se emita, lo cual no "
            "es directo con un Transformer de una sola pasada bottom-up) — "
            "ver TODO en compiler/codegen.py::proc_decl"
        )


def compile_source(source: str) -> Program:
    grammar_text = GRAMMAR_PATH.read_text(encoding="utf-8")
    parser = Lark(grammar_text, parser="lalr")
    tree = parser.parse(source)
    compiler = _Compiler()
    compiler.transform(tree)
    return compiler.finalize()
