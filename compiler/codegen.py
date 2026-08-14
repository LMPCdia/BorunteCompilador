"""
Segunda pasada del compilador: AST (compiler/ast_nodes.py) -> Program
(runtime/bytecode.py).

A diferencia de la v0.1 (que usaba un Transformer de Lark bottom-up y no
podía resolver IF/PROC), esto es un recorrido recursivo normal en Python:
como controlamos el orden de visita, podemos emitir un salto ANTES de
conocer su destino final y "parchearlo" (backpatch) una vez que terminamos
de emitir el bloque que salta.

Alcance v0.2 (limitaciones conocidas, a propósito):
  - IF solo soporta condiciones de la forma `VAR == CONST` (no `VAR == VAR`
    ni expresiones compuestas). Ver JUMP_IF_VAR_NEQ_CONST en
    docs/INSTRUCTION_SET.md.
  - PROC no tiene calling convention real: los parámetros se parsean pero
    no se bindean a nada dentro del cuerpo. Union de esto: por ahora tratá
    los PROC como subrutinas sin parámetros (usá VAR globales si necesitás
    pasar datos).
  - Los offsets de puntos se resuelven en tiempo de compilación, no en
    runtime.
"""

from __future__ import annotations

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
    VarDecl,
    VarRef,
    WaitInStmt,
    WaitTimeStmt,
)
from runtime.bytecode import (
    ADD_VAR,
    CALL,
    END,
    JUMP,
    JUMP_IF_VAR_NEQ_CONST,
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


class CompileError(Exception):
    pass


def _io_number(name: str) -> int:
    """'X010' -> 10, 'Y10' -> 10. Simplificación v0.1/v0.2 — ver docstring de arriba."""
    digits = "".join(c for c in name if c.isdigit())
    if not digits:
        raise CompileError(f"No se pudo interpretar el número de E/S en {name!r}")
    return int(digits)


class _Emitter:
    def __init__(self) -> None:
        self.program = Program()
        self._next_var_index = 0
        self._pending_calls: list[tuple[int, str]] = []  # (índice de instrucción CALL, nombre de proc)

    # -- utilidades ---------------------------------------------------------

    def var_idx(self, name: str) -> int:
        if name not in self.program.var_names:
            self.program.var_names[name] = self._next_var_index
            self._next_var_index += 1
        return self.program.var_names[name]

    def emit(self, instr: Instruction) -> int:
        self.program.instructions.append(instr)
        return len(self.program.instructions) - 1

    def here(self) -> int:
        return len(self.program.instructions)

    def register_point(self, pose: Pose, name: str | None = None) -> int:
        index = len(self.program.points)
        self.program.points.append(pose)
        if name:
            self.program.point_names[name] = index
        return index

    def resolve_point_expr(self, node) -> Pose:
        if isinstance(node, PointLiteral):
            return node.pose
        if isinstance(node, PointName):
            if node.name not in self.program.point_names:
                raise CompileError(f"Punto no definido: {node.name!r}")
            return self.program.points[self.program.point_names[node.name]]
        if isinstance(node, PointOffset):
            base = self.resolve_point_expr(node.base)
            off = node.offset
            return Pose(*(b + o for b, o in zip(
                (base.a, base.b, base.c, base.d, base.e, base.f),
                (off.a, off.b, off.c, off.d, off.e, off.f),
            )))
        raise CompileError(f"Nodo de punto desconocido: {node!r}")

    def eval_const_expr(self, node) -> float | None:
        """Evalúa la expresión si es 100% resoluble en tiempo de compilación
        (sin variables). Devuelve None si depende de algo que solo se conoce
        en runtime."""
        if isinstance(node, Const):
            return node.value
        if isinstance(node, BinOp):
            left = self.eval_const_expr(node.left)
            right = self.eval_const_expr(node.right)
            if left is None or right is None:
                return None
            if node.op == "+":
                return left + right
            if node.op == "-":
                return left - right
            if node.op == "==":
                return 1.0 if left == right else 0.0
            if node.op == "!=":
                return 1.0 if left != right else 0.0
        return None  # VarRef u operación no soportada en tiempo de compilación

    # -- emisión de sentencias ------------------------------------------------

    def emit_stmt(self, stmt) -> None:
        if isinstance(stmt, PointDecl):
            self.register_point(self.resolve_point_expr(stmt.expr), name=stmt.name)
        elif isinstance(stmt, VarDecl):
            idx = self.var_idx(stmt.name)
            if stmt.init is not None:
                value = self.eval_const_expr(stmt.init)
                if value is None:
                    raise CompileError(
                        f"VAR {stmt.name}: solo se soportan inicializadores constantes en v0.2"
                    )
                self.emit(Instruction(SET_VAR, a=idx, b=int(value)))
        elif isinstance(stmt, TimerDecl):
            pass  # v0.2: TIMER es metadata; se usa inline en WAIT <n>s
        elif isinstance(stmt, Assignment):
            self._emit_assignment(stmt)
        elif isinstance(stmt, MoveStmt):
            index = self.register_point(self.resolve_point_expr(stmt.point))
            opcode = MOVEJ if stmt.kind == "MOVEJ" else MOVEL
            self.emit(Instruction(opcode, b=index, d=int(stmt.speed)))
        elif isinstance(stmt, WaitInStmt):
            timeout_ms = int((stmt.timeout_s or 0) * 1000)
            self.emit(Instruction(WAIT_IN, a=_io_number(stmt.io_name), b=timeout_ms))
        elif isinstance(stmt, WaitTimeStmt):
            if not stmt.until_move_done and stmt.seconds is not None:
                self.emit(Instruction(WAIT_TIME, b=int(stmt.seconds * 1000)))
            # until_move_done: no-op — MOVEJ/MOVEL ya bloquean (ver runtime/vm.py)
        elif isinstance(stmt, SetOutStmt):
            self.emit(Instruction(SET_OUT, a=_io_number(stmt.io_name), b=1 if stmt.state else 0))
        elif isinstance(stmt, IfStmt):
            self._emit_if(stmt)
        elif isinstance(stmt, CallStmt):
            self._emit_call(stmt)
        elif isinstance(stmt, ProcDecl):
            self._emit_proc_decl(stmt)
        else:
            raise CompileError(f"Sentencia no soportada: {stmt!r}")

    def _emit_assignment(self, stmt: Assignment) -> None:
        idx = self.var_idx(stmt.name)
        const = self.eval_const_expr(stmt.expr)
        if const is not None:
            self.emit(Instruction(SET_VAR, a=idx, b=int(const)))
            return
        # patrón "x = x + k" -> ADD_VAR
        if (
            isinstance(stmt.expr, BinOp)
            and stmt.expr.op == "+"
            and isinstance(stmt.expr.left, VarRef)
            and stmt.expr.left.name == stmt.name
            and isinstance(stmt.expr.right, Const)
        ):
            self.emit(Instruction(ADD_VAR, a=idx, b=int(stmt.expr.right.value)))
            return
        raise CompileError(
            f"Asignación no soportada en v0.2: {stmt.name} = <expresión no constante "
            f"ni patrón 'x = x + k'>. Ver limitaciones en el docstring de este archivo."
        )

    def _emit_if(self, stmt: IfStmt) -> None:
        cond = stmt.cond
        if not isinstance(cond, BinOp) or cond.op != "==":
            raise CompileError(
                "v0.2 solo soporta condiciones IF de la forma VAR == CONST "
                "(ver JUMP_IF_VAR_NEQ_CONST en docs/INSTRUCTION_SET.md)"
            )
        if isinstance(cond.left, VarRef) and isinstance(cond.right, Const):
            var_name, const_val = cond.left.name, cond.right.value
        elif isinstance(cond.right, VarRef) and isinstance(cond.left, Const):
            var_name, const_val = cond.right.name, cond.left.value
        else:
            raise CompileError("v0.2 solo soporta condiciones IF de la forma VAR == CONST")

        var_index = self.var_idx(var_name)
        skip_idx = self.emit(
            Instruction(JUMP_IF_VAR_NEQ_CONST, a=var_index, b=int(const_val), c=0,
                        comment=f"si {var_name} != {int(const_val)} -> else/fin")
        )
        for s in stmt.then_body:
            self.emit_stmt(s)

        if stmt.else_body:
            end_jump_idx = self.emit(Instruction(JUMP, b=0, comment="saltar bloque else"))
            self.program.instructions[skip_idx].c = self.here()
            for s in stmt.else_body:
                self.emit_stmt(s)
            self.program.instructions[end_jump_idx].b = self.here()
        else:
            self.program.instructions[skip_idx].c = self.here()

    def _emit_call(self, stmt: CallStmt) -> None:
        idx = self.emit(Instruction(CALL, b=0, comment=f"call {stmt.name}"))
        self._pending_calls.append((idx, stmt.name))

    def _emit_proc_decl(self, stmt: ProcDecl) -> None:
        skip_idx = self.emit(Instruction(JUMP, b=0, comment=f"saltar cuerpo de {stmt.name}"))
        start_pc = self.here()
        self.program.proc_addresses[stmt.name] = start_pc
        for s in stmt.body:
            self.emit_stmt(s)
        self.emit(Instruction(RET))
        self.program.instructions[skip_idx].b = self.here()

    def finalize(self) -> Program:
        self.emit(Instruction(END))
        for instr_index, name in self._pending_calls:
            if name not in self.program.proc_addresses:
                raise CompileError(f"Llamada a procedimiento no definido: {name!r}")
            self.program.instructions[instr_index].b = self.program.proc_addresses[name]
        return self.program


def compile_source(source: str) -> Program:
    ast = build_ast(source)
    emitter = _Emitter()
    for stmt in ast.statements:
        emitter.emit_stmt(stmt)
    return emitter.finalize()
