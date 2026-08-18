"""
Segunda pasada del compilador: AST (compiler/ast_nodes.py) -> Program
(runtime/bytecode.py).

A diferencia de la v0.1 (que usaba un Transformer de Lark bottom-up y no
podía resolver IF/PROC), esto es un recorrido recursivo normal en Python:
como controlamos el orden de visita, podemos emitir un salto ANTES de
conocer su destino final y "parchearlo" (backpatch) una vez que terminamos
de emitir el bloque que salta.

Pasadas (en este orden):
  0. `_collect_proc_signatures`: junta las firmas de TODOS los PROC antes de
     emitir nada. Hace falta porque una llamada hacia adelante también tiene
     que poder cargar los slots de los parámetros del PROC que llama, y para
     eso necesita conocer su aridad antes de haber visto su declaración.
  1. `build_ast` (compiler/ast_builder.py): texto -> AST.
  2. `_Emitter`: AST -> Program, con backpatching de saltos.

Alcance v0.3 (limitaciones conocidas, a propósito):
  - IF soporta `VAR == CONST` (JUMP_IF_VAR_NEQ_CONST) y `VAR == VAR`
    (JUMP_IF_VAR_NEQ_VAR), pero no expresiones compuestas ni otros
    operadores que `==`.
  - PROC tiene calling convention por slots fijos, sin pila de frames: no es
    recursiva ni reentrante, y los parámetros son por valor. Ver la sección
    "Calling convention de PROC" en docs/INSTRUCTION_SET.md — esas mismas
    limitaciones las tiene que respetar plc_vm/.
  - Los offsets de puntos se resuelven en tiempo de compilación, no en
    runtime.
  - La tabla de puntos no se deduplica: cada MOVEJ/MOVEL registra una entrada
    nueva aunque mueva a un POINT ya declarado.
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
    COPY_VAR,
    END,
    JUMP,
    JUMP_IF_VAR_NEQ_CONST,
    JUMP_IF_VAR_NEQ_VAR,
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


def param_slot_name(proc: str, param: str) -> str:
    """Nombre reservado del slot de un parámetro: `<proc>.<param>`.

    El punto no es un carácter válido en un identificador del DSL (NAME es
    CNAME), así que un slot nunca puede colisionar con una VAR del usuario.
    """
    return f"{proc}.{param}"


def _collect_proc_signatures(statements: list) -> dict[str, list[str]]:
    """Pasada 0: nombre de PROC -> lista de nombres de sus parámetros.

    Recorre también los cuerpos de IF y de otros PROC, así que una declaración
    anidada se ve igual. Detectar acá el PROC duplicado es importante: si se
    deja pasar, la segunda declaración le pisa la dirección a la primera en
    `proc_addresses` y todas las llamadas terminan yendo a la segunda, en
    silencio.
    """
    signatures: dict[str, list[str]] = {}

    def walk(stmts: list) -> None:
        for stmt in stmts:
            if isinstance(stmt, ProcDecl):
                if stmt.name in signatures:
                    raise CompileError(
                        f"PROC {stmt.name!r} declarado más de una vez"
                    )
                signatures[stmt.name] = list(stmt.params)
                walk(stmt.body)
            elif isinstance(stmt, IfStmt):
                walk(stmt.then_body)
                walk(stmt.else_body)

    walk(statements)
    return signatures


class _Emitter:
    def __init__(self, proc_signatures: dict[str, list[str]] | None = None) -> None:
        self.program = Program()
        self._next_var_index = 0
        self._pending_calls: list[tuple[int, str]] = []  # (índice de instrucción CALL, nombre de proc)
        self._proc_signatures = dict(proc_signatures or {})
        # PROC cuyo cuerpo estamos emitiendo, para resolver sus parámetros al
        # slot correcto. None = nivel superior.
        self._current_proc: str | None = None

    # -- utilidades ---------------------------------------------------------

    def var_idx(self, name: str) -> int:
        if name not in self.program.var_names:
            self.program.var_names[name] = self._next_var_index
            self._next_var_index += 1
        return self.program.var_names[name]

    def resolve_var(self, name: str) -> int:
        """Índice del banco de variables para un nombre USADO en una expresión.

        Dentro del cuerpo de un PROC, un nombre que coincide con uno de sus
        parámetros resuelve al slot del parámetro; cualquier otro nombre es
        global. Esto es lo que hace que el cuerpo lea el valor que le cargó el
        llamador.
        """
        if self._current_proc is not None:
            slot = param_slot_name(self._current_proc, name)
            if slot in self.program.var_names:
                return self.program.var_names[slot]
        return self.var_idx(name)

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
        idx = self.resolve_var(stmt.name)
        const = self.eval_const_expr(stmt.expr)
        if const is not None:
            self.emit(Instruction(SET_VAR, a=idx, b=int(const)))
            return
        # "a = b" entre variables -> COPY_VAR
        if isinstance(stmt.expr, VarRef):
            self.emit(
                Instruction(
                    COPY_VAR,
                    a=idx,
                    b=self.resolve_var(stmt.expr.name),
                    comment=f"{stmt.name} = {stmt.expr.name}",
                )
            )
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
            f"Asignación no soportada en v0.3: {stmt.name} = <expresión que no es "
            f"constante, ni otra variable, ni el patrón 'x = x + k'>. "
            f"Ver limitaciones en el docstring de este archivo."
        )

    def _emit_condition_jump(self, cond) -> int:
        """Emite el salto que SALTEA el bloque `then` si la condición es falsa.

        Devuelve el índice de la instrucción emitida, para que el llamador le
        parchee el destino (`c`) cuando lo conozca. Los dos opcodes de salto
        condicional usan `c` como destino, así que el backpatch es el mismo
        para ambos.
        """
        if not isinstance(cond, BinOp) or cond.op != "==":
            raise CompileError(
                "El IF solo soporta condiciones con '==' (VAR == CONST o VAR == VAR) "
                "— ver JUMP_IF_VAR_NEQ_CONST y JUMP_IF_VAR_NEQ_VAR en "
                "docs/INSTRUCTION_SET.md"
            )
        # VAR == VAR
        if isinstance(cond.left, VarRef) and isinstance(cond.right, VarRef):
            left, right = cond.left.name, cond.right.name
            return self.emit(
                Instruction(
                    JUMP_IF_VAR_NEQ_VAR,
                    a=self.resolve_var(left),
                    b=self.resolve_var(right),
                    c=0,
                    comment=f"si {left} != {right} -> else/fin",
                )
            )
        # VAR == CONST (en cualquiera de los dos órdenes)
        if isinstance(cond.left, VarRef) and isinstance(cond.right, Const):
            var_name, const_val = cond.left.name, cond.right.value
        elif isinstance(cond.right, VarRef) and isinstance(cond.left, Const):
            var_name, const_val = cond.right.name, cond.left.value
        else:
            raise CompileError(
                "El IF solo soporta condiciones de la forma VAR == CONST o VAR == VAR "
                "(no expresiones compuestas)"
            )
        return self.emit(
            Instruction(JUMP_IF_VAR_NEQ_CONST, a=self.resolve_var(var_name),
                        b=int(const_val), c=0,
                        comment=f"si {var_name} != {int(const_val)} -> else/fin")
        )

    def _emit_if(self, stmt: IfStmt) -> None:
        skip_idx = self._emit_condition_jump(stmt.cond)
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
        if stmt.name not in self._proc_signatures:
            raise CompileError(f"Llamada a procedimiento no definido: {stmt.name!r}")

        params = self._proc_signatures[stmt.name]
        if len(stmt.args) != len(params):
            raise CompileError(
                f"{stmt.name}() espera {len(params)} argumento(s) "
                f"({', '.join(params) or 'ninguno'}) pero se le pasaron {len(stmt.args)}"
            )

        # Carga de los slots ANTES del CALL. Funciona igual para una llamada
        # hacia adelante porque la aridad viene de la pasada 0, no de haber
        # visto ya la declaración.
        for param, arg in zip(params, stmt.args):
            slot = self.var_idx(param_slot_name(stmt.name, param))
            const = self.eval_const_expr(arg)
            if const is not None:
                self.emit(
                    Instruction(SET_VAR, a=slot, b=int(const),
                                comment=f"{stmt.name}.{param} = {int(const)}")
                )
            elif isinstance(arg, VarRef):
                self.emit(
                    Instruction(COPY_VAR, a=slot, b=self.resolve_var(arg.name),
                                comment=f"{stmt.name}.{param} = {arg.name}")
                )
            else:
                raise CompileError(
                    f"Argumento no soportado en la llamada a {stmt.name}(): el "
                    f"parámetro {param!r} solo acepta una constante o una variable, "
                    f"no una expresión compuesta"
                )

        idx = self.emit(Instruction(CALL, b=0, comment=f"call {stmt.name}"))
        self._pending_calls.append((idx, stmt.name))

    def _emit_proc_decl(self, stmt: ProcDecl) -> None:
        if stmt.name in self.program.proc_addresses:
            raise CompileError(f"PROC {stmt.name!r} declarado más de una vez")

        # Los slots se reservan aunque al PROC nunca se lo llame, para que el
        # cuerpo pueda resolver sus parámetros y para que aparezcan en la
        # tabla de variables (la GUI los muestra).
        for param in stmt.params:
            self.var_idx(param_slot_name(stmt.name, param))
        self.program.proc_params[stmt.name] = list(stmt.params)

        skip_idx = self.emit(Instruction(JUMP, b=0, comment=f"saltar cuerpo de {stmt.name}"))
        start_pc = self.here()
        self.program.proc_addresses[stmt.name] = start_pc

        outer_proc = self._current_proc
        self._current_proc = stmt.name
        try:
            for s in stmt.body:
                self.emit_stmt(s)
        finally:
            self._current_proc = outer_proc

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
    signatures = _collect_proc_signatures(ast.statements)  # pasada 0
    emitter = _Emitter(signatures)
    for stmt in ast.statements:
        emitter.emit_stmt(stmt)
    return emitter.finalize()
