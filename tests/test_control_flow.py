"""
Tests de control de flujo del compiler.

Cubre lo que estaba roto en el codegen v0.1 (IF/ELSE y PROC/CALL, resueltos
en v0.2 con dos pasadas) y lo que se agregó en v0.3: IF con VAR == VAR,
calling convention de PROC por slots fijos, asignacion entre variables, y el
manejo de comentarios y lineas en blanco.

Ver compiler/codegen.py y docs/INSTRUCTION_SET.md.
"""

import pytest

from comms.robot_client import BorunteRobotClient
from comms.robot_simulator import SimulatedBorunteRobot
from compiler.codegen import CompileError, compile_source, param_slot_name
from runtime.bytecode import (
    CALL,
    COPY_VAR,
    END,
    JUMP_IF_VAR_NEQ_CONST,
    JUMP_IF_VAR_NEQ_VAR,
    SET_VAR,
)
from runtime.plc_io_simulator import PlcIoSimulator
from runtime.vm import ReferenceVM


def _make_vm(program):
    sim = SimulatedBorunteRobot(move_duration_s=0.02)
    robot = BorunteRobotClient(host="fake", client=sim)
    robot.connect()
    plc_io = PlcIoSimulator()
    return ReferenceVM(program, robot, plc_io), plc_io


def test_if_then_branch_taken():
    src = """VAR pieza : INT = 1
IF pieza == 1 THEN
SET_OUT(Y10, ON)
ELSE
SET_OUT(Y11, ON)
ENDIF
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(10) is True
    assert plc_io.read_output(11) is False


def test_if_else_branch_taken():
    src = """VAR pieza : INT = 2
IF pieza == 1 THEN
SET_OUT(Y10, ON)
ELSE
SET_OUT(Y11, ON)
ENDIF
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(10) is False
    assert plc_io.read_output(11) is True


def test_if_without_else():
    src = """VAR pieza : INT = 5
IF pieza == 1 THEN
SET_OUT(Y10, ON)
ENDIF
SET_OUT(Y12, ON)
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(10) is False
    assert plc_io.read_output(12) is True  # el código después del IF se ejecuta igual


# --- IF con VAR == VAR (opcode 0x0E, agregado en el contrato v0.2) ----------


def test_if_var_eq_var_true_branch():
    src = """VAR a : INT = 3
VAR b : INT = 3
IF a == b THEN
SET_OUT(Y10, ON)
ELSE
SET_OUT(Y11, ON)
ENDIF
"""
    program = compile_source(src)
    assert any(i.opcode == JUMP_IF_VAR_NEQ_VAR for i in program.instructions)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(10) is True
    assert plc_io.read_output(11) is False


def test_if_var_eq_var_false_branch():
    src = """VAR a : INT = 3
VAR b : INT = 4
IF a == b THEN
SET_OUT(Y10, ON)
ELSE
SET_OUT(Y11, ON)
ENDIF
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(10) is False
    assert plc_io.read_output(11) is True


def test_if_var_eq_var_uses_the_two_var_opcode_not_the_const_one():
    """Regresión: con dos variables NO se puede usar JUMP_IF_VAR_NEQ_CONST,
    porque interpretaría el índice de la segunda variable como un valor."""
    src = """VAR a : INT = 3
VAR b : INT = 3
IF a == b THEN
SET_OUT(Y10, ON)
ENDIF
"""
    program = compile_source(src)
    jumps = [i for i in program.instructions if i.opcode == JUMP_IF_VAR_NEQ_VAR]
    assert len(jumps) == 1
    assert jumps[0].a == program.var_names["a"]
    assert jumps[0].b == program.var_names["b"]
    assert not any(i.opcode == JUMP_IF_VAR_NEQ_CONST for i in program.instructions)


def test_if_still_rejects_operator_other_than_eq():
    src = """VAR a : INT = 1
VAR b : INT = 2
IF a != b THEN
SET_OUT(Y10, ON)
ENDIF
"""
    with pytest.raises(CompileError):
        compile_source(src)


def test_if_rejects_compound_expression():
    src = """VAR a : INT = 1
IF a + 1 == 2 THEN
SET_OUT(Y10, ON)
ENDIF
"""
    with pytest.raises(CompileError):
        compile_source(src)


def test_proc_call_executes_body():
    src = """POINT p_home = WORLD(0.0, 500.0, 300.0, 0.0, 0.0, 0.0)
PROC saludar()
SET_OUT(Y20, ON)
ENDPROC
SET_OUT(Y21, ON)
saludar()
SET_OUT(Y22, ON)
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    # las 3 salidas se activaron: la de antes del CALL, la de dentro del
    # PROC (llamado), y la de después del CALL
    assert plc_io.read_output(20) is True
    assert plc_io.read_output(21) is True
    assert plc_io.read_output(22) is True
    # y el orden real de ejecución fue 21 -> 20 -> 22 (el cuerpo del PROC
    # se saltea en la pasada lineal y solo corre cuando se lo llama)
    order = [num for (_, num, state) in plc_io.output_log]
    assert order == [21, 20, 22]


def test_call_to_undefined_proc_fails_at_compile_time():
    src = "no_existe()\n"
    with pytest.raises(CompileError):
        compile_source(src)


def test_proc_defined_after_call_still_resolves():
    """El CALL puede aparecer ANTES del PROC en el texto — el backpatching
    tiene que resolverlo igual."""
    src = """SET_OUT(Y30, ON)
hacer_algo()
PROC hacer_algo()
SET_OUT(Y31, ON)
ENDPROC
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(30) is True
    assert plc_io.read_output(31) is True


# --- Calling convention de PROC (slots fijos, contrato v0.2) -----------------
#
# Los parámetros NO van por pila: cada uno tiene un slot fijo llamado
# "<proc>.<param>" en el banco de variables, que el llamador carga con SET_VAR
# (constante) o COPY_VAR (variable) antes del CALL. Ver la sección "Calling
# convention de PROC" en docs/INSTRUCTION_SET.md.


def _var(program, name):
    """Índice de una variable en el banco, por nombre."""
    return program.var_names[name]


def test_proc_param_slot_is_registered():
    src = """PROC apilar(altura)
SET_OUT(Y20, ON)
ENDPROC
apilar(5)
"""
    program = compile_source(src)
    assert program.proc_params == {"apilar": ["altura"]}
    assert param_slot_name("apilar", "altura") in program.var_names


def test_proc_param_slot_never_collides_with_user_var():
    """Un punto no es válido en un identificador del DSL, así que el slot
    "apilar.altura" no puede chocar con una VAR del usuario llamada altura."""
    src = """VAR altura : INT = 1
PROC apilar(altura)
SET_OUT(Y20, ON)
ENDPROC
apilar(9)
"""
    program = compile_source(src)
    assert program.var_names["altura"] != program.var_names["apilar.altura"]


def test_proc_const_arg_loads_slot_with_set_var():
    src = """PROC apilar(altura)
SET_OUT(Y20, ON)
ENDPROC
apilar(5)
"""
    program = compile_source(src)
    slot = program.var_names["apilar.altura"]
    loads = [
        i for i in program.instructions
        if i.opcode == SET_VAR and i.a == slot and i.b == 5
    ]
    assert len(loads) == 1


def test_proc_var_arg_loads_slot_with_copy_var():
    src = """VAR nivel : INT = 7
PROC apilar(altura)
SET_OUT(Y20, ON)
ENDPROC
apilar(nivel)
"""
    program = compile_source(src)
    slot = program.var_names["apilar.altura"]
    origen = program.var_names["nivel"]
    loads = [
        i for i in program.instructions
        if i.opcode == COPY_VAR and i.a == slot and i.b == origen
    ]
    assert len(loads) == 1


def test_proc_body_reads_the_param_value():
    """Lo que cierra el circuito: el cuerpo tiene que VER el valor que le
    cargó el llamador, no un cero."""
    src = """PROC decidir(modo)
IF modo == 2 THEN
SET_OUT(Y21, ON)
ELSE
SET_OUT(Y22, ON)
ENDIF
ENDPROC
decidir(2)
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(21) is True
    assert plc_io.read_output(22) is False


def test_proc_two_calls_with_different_args_take_different_branches():
    src = """PROC decidir(modo)
IF modo == 1 THEN
SET_OUT(Y31, ON)
ENDIF
IF modo == 2 THEN
SET_OUT(Y32, ON)
ENDIF
ENDPROC
decidir(1)
decidir(2)
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    # las dos llamadas corrieron el mismo cuerpo con valores distintos
    assert plc_io.read_output(31) is True
    assert plc_io.read_output(32) is True


def test_proc_params_are_by_value():
    """Escribir el parámetro adentro del PROC modifica el slot, NO la variable
    del llamador. Es consecuencia de no tener frames — está documentado como
    limitación en el contrato."""
    src = """VAR nivel : INT = 7
PROC pisar(x)
x = 99
ENDPROC
pisar(nivel)
"""
    program = compile_source(src)
    vm, _ = _make_vm(program)
    vm.run_from(0)
    assert vm.variables[_var(program, "nivel")] == 7
    assert vm.variables[_var(program, "pisar.x")] == 99


def test_proc_call_before_declaration_still_loads_params():
    """La llamada hacia adelante también tiene que cargar los slots — para eso
    existe la pasada 0 (_collect_proc_signatures)."""
    src = """decidir(2)
PROC decidir(modo)
IF modo == 2 THEN
SET_OUT(Y41, ON)
ENDIF
ENDPROC
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(41) is True


def test_proc_arity_mismatch_too_few_args():
    src = """PROC apilar(altura, columna)
SET_OUT(Y20, ON)
ENDPROC
apilar(5)
"""
    with pytest.raises(CompileError, match="espera 2 argumento"):
        compile_source(src)


def test_proc_arity_mismatch_too_many_args():
    src = """PROC apilar(altura)
SET_OUT(Y20, ON)
ENDPROC
apilar(5, 6)
"""
    with pytest.raises(CompileError, match="espera 1 argumento"):
        compile_source(src)


def test_proc_rejects_compound_expression_as_arg():
    src = """VAR n : INT = 1
PROC apilar(altura)
SET_OUT(Y20, ON)
ENDPROC
apilar(n + 1)
"""
    with pytest.raises(CompileError):
        compile_source(src)


def test_duplicate_proc_is_a_compile_error():
    """Antes la segunda declaración le pisaba la dirección a la primera en
    silencio, y todas las llamadas terminaban yendo a la segunda."""
    src = """PROC hacer()
SET_OUT(Y20, ON)
ENDPROC
PROC hacer()
SET_OUT(Y21, ON)
ENDPROC
hacer()
"""
    with pytest.raises(CompileError, match="más de una vez"):
        compile_source(src)


def test_proc_with_two_params_loads_both_in_order():
    src = """VAR n : INT = 4
PROC mover(a, b)
SET_OUT(Y20, ON)
ENDPROC
mover(3, n)
"""
    program = compile_source(src)
    slot_a = program.var_names["mover.a"]
    slot_b = program.var_names["mover.b"]
    # los dos loads salen antes del CALL, y en el orden de los parámetros
    seq = [
        i for i in program.instructions
        if (i.opcode in (SET_VAR, COPY_VAR) and i.a in (slot_a, slot_b))
        or i.opcode == CALL
    ]
    assert [i.a for i in seq[:2]] == [slot_a, slot_b]
    assert seq[2].opcode == CALL


# --- Asignación entre variables (bug: daba CompileError) ---------------------


def test_assignment_between_variables_emits_copy_var():
    src = """VAR a : INT = 1
VAR b : INT = 5
a = b
"""
    program = compile_source(src)
    copies = [i for i in program.instructions if i.opcode == COPY_VAR]
    assert len(copies) == 1
    assert copies[0].a == program.var_names["a"]
    assert copies[0].b == program.var_names["b"]
    vm, _ = _make_vm(program)
    vm.run_from(0)
    assert vm.variables[program.var_names["a"]] == 5


def test_assignment_between_variables_runs_in_vm():
    src = """VAR origen : INT = 42
VAR destino : INT = 0
destino = origen
IF destino == 42 THEN
SET_OUT(Y50, ON)
ENDIF
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(50) is True


# --- Comentarios y líneas en blanco (bug: no compilaban) --------------------


def test_comment_on_its_own_line_compiles():
    src = """; programa de prueba
SET_OUT(Y60, ON)
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(60) is True


def test_blank_line_at_start_of_file_compiles():
    src = "\nSET_OUT(Y61, ON)\n"
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(61) is True


def test_blank_lines_and_comments_inside_proc_and_if():
    src = """; encabezado

VAR a : INT = 1

PROC hacer()

; adentro del PROC
SET_OUT(Y62, ON)

ENDPROC

IF a == 1 THEN

; adentro del THEN
SET_OUT(Y63, ON)

ELSE

; adentro del ELSE
SET_OUT(Y64, ON)

ENDIF

hacer()
"""
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(62) is True
    assert plc_io.read_output(63) is True
    assert plc_io.read_output(64) is False


def test_trailing_comment_on_a_statement_line_still_works():
    src = "SET_OUT(Y65, ON)   ; enciende la pinza\n"
    program = compile_source(src)
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(65) is True


def test_file_with_only_comments_compiles_to_just_end():
    program = compile_source("; nada que hacer\n")
    assert [i.opcode for i in program.instructions] == [END]


def test_file_without_trailing_newline_compiles():
    """Es exactamente lo que pasa escribiendo en el editor de la GUI sin
    apretar Enter al final."""
    program = compile_source("SET_OUT(Y66, ON)")
    vm, plc_io = _make_vm(program)
    vm.run_from(0)
    assert plc_io.read_output(66) is True


def test_empty_source_compiles_to_just_end():
    program = compile_source("")
    assert [i.opcode for i in program.instructions] == [END]
