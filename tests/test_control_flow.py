"""
Tests de IF/ELSE y PROC/CALL — lo que estaba roto en el codegen v0.1 y ahora
debería andar con el codegen v0.2 (dos pasadas, ver compiler/codegen.py).
"""

import pytest

from comms.robot_client import BorunteRobotClient
from comms.robot_simulator import SimulatedBorunteRobot
from compiler.codegen import CompileError, compile_source
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


def test_if_rejects_non_const_condition():
    src = """VAR a : INT = 1
VAR b : INT = 2
IF a == b THEN
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
