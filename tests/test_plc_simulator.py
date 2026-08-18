"""
Tests del PLC simulado (comms/plc_simulator.py).

Lo importante de este archivo no son los tests de opcodes uno por uno, sino la
sección final: **el mismo programa ejecutado por la VM de referencia en Python y
por la VM del PLC tiene que dar el mismo resultado**. Son dos implementaciones
independientes del mismo contrato, por caminos completamente distintos (una
recorre objetos `Instruction` en memoria; la otra decodifica words de registros
Modbus). Si divergen, una de las dos está mal leyendo
`docs/INSTRUCTION_SET.md` — y eso es exactamente lo que hay que descubrir ANTES
de escribir el ladder, no después.

Lo que estos tests NO prueban: que el mapa de registros sea el correcto contra un
CX3G real. Eso sigue siendo hipótesis (ver el docstring de
`comms/plc_simulator.py`).
"""

import time

import pytest

from comms.plc_client import (
    CoolmayModbusError,
    CoolmayPlcClient,
    PlcCommand,
    PlcVmError,
    PlcVmState,
)
from comms.plc_simulator import CALL_STACK_DEPTH, SimulatedCoolmayPlc
from comms.robot_client import BorunteRobotClient, Pose
from comms.robot_simulator import SimulatedBorunteRobot
from compiler.codegen import compile_source
from runtime.bytecode import Instruction, Program
from runtime.plc_io_simulator import PlcIoSimulator
from runtime.vm import ReferenceVM

TIMEOUT = 10.0


def _plc(**kw):
    """Cliente + simulador, ya conectados. tick_s=0 para que los tests vuelen."""
    kw.setdefault("tick_s", 0.0)
    sim = SimulatedCoolmayPlc(**kw)
    client = CoolmayPlcClient(host="sim", client=sim)
    client.connect()
    return client, sim


def _run(source: str, **kw):
    """Compila, carga, arranca y espera. Devuelve (cliente, simulador, programa)."""
    client, sim = _plc(**kw)
    program = compile_source(source)
    client.upload_program(program)
    client.start()
    assert sim.wait_until_done(TIMEOUT), "el programa no terminó a tiempo"
    return client, sim, program


# --- ciclo de vida y handshake ------------------------------------------------


def test_starts_stopped():
    client, sim = _plc()
    status = client.read_status()
    assert status.state == PlcVmState.STOPPED
    assert status.pc == 0
    assert status.error == 0


def test_reports_the_contract_version_it_implements():
    client, _ = _plc()
    assert client.read_status().contract_version == 2


def test_acknowledges_known_commands():
    client, sim = _plc()
    client.start(timeout_s=1.0)  # no levanta = hubo acuse
    assert sim._get(3) == int(PlcCommand.NONE)


def test_does_not_acknowledge_an_unknown_command():
    """Un ladder que no sabe qué hacer con un valor no lo acusa, y el cliente lo
    reporta como timeout en vez de dar por hecho que se ejecutó."""
    client, sim = _plc()
    sim.write_register(3, 99)
    assert sim._get(3) == 99  # quedó sin acusar


def test_finishes_at_end_with_state_finished():
    _, sim, program = _run("SET_OUT(Y10, ON)\n")
    assert sim.state == PlcVmState.FINISHED
    assert sim.read_output(10) is True


def test_reset_clears_error_and_pc():
    client, sim = _plc()
    sim._set(2, int(PlcVmError.WAIT_IN_TIMEOUT))
    sim._set(1, int(PlcVmState.ERROR))
    sim._set(0, 17)
    client.reset()
    status = client.read_status()
    assert status.error == 0
    assert status.pc == 0
    assert status.state == PlcVmState.STOPPED


# --- opcodes -------------------------------------------------------------------


def test_set_var_and_add_var():
    _, sim, program = _run("VAR n : INT = 5\nn = n + 3\n")
    assert sim.get_var(program.var_names["n"]) == 8


def test_copy_var():
    _, sim, program = _run("VAR a : INT = 0\nVAR b : INT = 42\na = b\n")
    assert sim.get_var(program.var_names["a"]) == 42


def test_negative_variable_values():
    _, sim, program = _run("VAR n : INT = 5\nn = n + -8\n")
    assert sim.get_var(program.var_names["n"]) == -3


def test_if_then_branch():
    _, sim, _ = _run(
        "VAR n : INT = 1\nIF n == 1 THEN\nSET_OUT(Y10, ON)\nELSE\nSET_OUT(Y11, ON)\nENDIF\n"
    )
    assert sim.read_output(10) is True
    assert sim.read_output(11) is False


def test_if_else_branch():
    _, sim, _ = _run(
        "VAR n : INT = 2\nIF n == 1 THEN\nSET_OUT(Y10, ON)\nELSE\nSET_OUT(Y11, ON)\nENDIF\n"
    )
    assert sim.read_output(10) is False
    assert sim.read_output(11) is True


def test_if_var_eq_var():
    _, sim, _ = _run(
        "VAR a : INT = 7\nVAR b : INT = 7\nIF a == b THEN\nSET_OUT(Y12, ON)\nENDIF\n"
    )
    assert sim.read_output(12) is True


def test_call_and_ret():
    _, sim, _ = _run(
        "PROC hacer()\nSET_OUT(Y20, ON)\nENDPROC\nSET_OUT(Y21, ON)\nhacer()\n"
    )
    assert sim.read_output(20) is True
    assert sim.read_output(21) is True


def test_proc_with_parameters_runs_on_the_plc():
    """La calling convention por slots tiene que funcionar también del lado del
    PLC: es lo que el ladder va a tener que respetar."""
    _, sim, program = _run(
        "VAR total : INT = 0\n"
        "PROC sumar(cuanto)\n"
        "total = total + 1\n"
        "ENDPROC\n"
        "sumar(1)\n"
        "sumar(1)\n"
        "sumar(1)\n"
    )
    assert sim.get_var(program.var_names["total"]) == 3


def test_wait_time_advances():
    _, sim, _ = _run("SET_OUT(Y10, ON)\nWAIT 0.02s\nSET_OUT(Y10, OFF)\n")
    assert sim.read_output(10) is False


def test_wait_in_succeeds_when_the_input_is_already_on():
    client, sim = _plc()
    sim.set_input(5, True)
    program = compile_source("WAIT_IN(X5, 1)\nSET_OUT(Y10, ON)\n")
    client.upload_program(program)
    client.start()
    assert sim.wait_until_done(TIMEOUT)
    assert sim.read_output(10) is True
    assert sim.state == PlcVmState.FINISHED


def test_wait_in_times_out_into_an_error_state():
    client, sim = _plc()
    program = compile_source("WAIT_IN(X6, 0.05)\nSET_OUT(Y10, ON)\n")
    client.upload_program(program)
    client.start()
    assert sim.wait_until_done(TIMEOUT)
    status = client.read_status()
    assert status.state == PlcVmState.ERROR
    assert status.error == int(PlcVmError.WAIT_IN_TIMEOUT)
    assert sim.read_output(10) is False


def test_move_without_a_robot_is_simulated_as_a_delay():
    _, sim, program = _run(
        "POINT p = WORLD(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)\nMOVEJ p SPEED 50\n",
        move_duration_s=0.01,
    )
    assert sim.state == PlcVmState.FINISHED


def test_move_drives_the_robot_when_one_is_given():
    """Los dos niveles juntos: el PLC simulado le manda los movimientos al robot
    simulado, que es como va a funcionar el sistema de verdad."""
    robot_sim = SimulatedBorunteRobot(move_duration_s=0.01)
    robot = BorunteRobotClient(host="sim", client=robot_sim)
    robot.connect()

    client, sim = _plc(robot=robot)
    program = compile_source(
        "POINT p = WORLD(100.0, 200.0, 300.0, 0.0, 0.0, 0.0)\nMOVEJ p SPEED 50\n"
    )
    client.upload_program(program)
    client.start()
    assert sim.wait_until_done(TIMEOUT)
    assert sim.state == PlcVmState.FINISHED

    final = robot.read_world_position()
    assert final.a == pytest.approx(100.0, abs=0.01)
    assert final.b == pytest.approx(200.0, abs=0.01)
    assert final.c == pytest.approx(300.0, abs=0.01)


# --- errores --------------------------------------------------------------------


def test_unknown_opcode_sets_the_error_state():
    client, sim = _plc()
    # Se escribe un opcode que no existe en la tabla, directo en el registro.
    sim.write_registers(1000, [0x77, 0, 0, 0, 0, 0, 0, 0])
    sim._set(4, 1)  # D4 = 1 instrucción cargada
    client.start()
    assert sim.wait_until_done(TIMEOUT)
    status = client.read_status()
    assert status.state == PlcVmState.ERROR
    assert status.error == int(PlcVmError.UNKNOWN_OPCODE)


def test_point_index_out_of_range_sets_the_error_state():
    client, sim = _plc()
    program = Program(instructions=[Instruction("MOVEJ", b=9), Instruction("END")])
    client.upload_program(program)  # sin puntos cargados
    client.start()
    assert sim.wait_until_done(TIMEOUT)
    status = client.read_status()
    assert status.state == PlcVmState.ERROR
    assert status.error == int(PlcVmError.POINT_INDEX_OUT_OF_RANGE)


def test_ret_without_call_sets_the_error_state():
    client, sim = _plc()
    client.upload_program(Program(instructions=[Instruction("RET"), Instruction("END")]))
    client.start()
    assert sim.wait_until_done(TIMEOUT)
    status = client.read_status()
    assert status.state == PlcVmState.ERROR
    assert status.error == int(PlcVmError.RET_WITHOUT_CALL)


def test_call_stack_overflow_sets_the_error_state():
    """Un CALL a sí mismo desborda la pila. El límite del simulador es a
    propósito más chico que el que probablemente tenga el CX3G, para que un
    programa demasiado anidado falle acá antes que en el PLC."""
    client, sim = _plc()
    client.upload_program(
        Program(instructions=[Instruction("CALL", b=0), Instruction("END")])
    )
    client.start()
    assert sim.wait_until_done(TIMEOUT)
    status = client.read_status()
    assert status.state == PlcVmState.ERROR
    assert status.error == int(PlcVmError.CALL_STACK_OVERFLOW)
    assert sim.executed_instructions <= CALL_STACK_DEPTH + 1


def test_var_index_out_of_range_sets_the_error_state():
    client, sim = _plc()
    client.upload_program(
        Program(instructions=[Instruction("SET_VAR", a=500, b=1), Instruction("END")])
    )
    client.start()
    assert sim.wait_until_done(TIMEOUT)
    status = client.read_status()
    assert status.state == PlcVmState.ERROR
    assert status.error == int(PlcVmError.VAR_INDEX_OUT_OF_RANGE)


# --- parada y pausa ----------------------------------------------------------------


def test_stop_interrupts_a_long_program():
    client, sim = _plc(tick_s=0.005)
    # Un bucle infinito: JUMP a sí mismo. Sin STOP no termina nunca.
    client.upload_program(
        Program(instructions=[Instruction("JUMP", b=0), Instruction("END")])
    )
    client.start()
    client.stop()
    assert sim.wait_until_done(2.0), "el STOP no cortó la ejecución"
    assert sim.state == PlcVmState.STOPPED


def test_stop_interrupts_a_long_wait():
    """Un time.sleep() pelado dejaría a la VM sorda al comando durante todo el
    WAIT."""
    client, sim = _plc(tick_s=0.0)
    client.upload_program(
        Program(instructions=[Instruction("WAIT_TIME", b=30_000), Instruction("END")])
    )
    client.start()
    client.stop()
    assert sim.wait_until_done(3.0), "el STOP no interrumpió el WAIT"


def test_pause_and_resume():
    client, sim = _plc(tick_s=0.01)
    client.upload_program(
        Program(instructions=[Instruction("JUMP", b=0), Instruction("END")])
    )
    client.start()
    client.pause()
    assert client.read_status().state == PlcVmState.PAUSED

    # El contador se lee DESPUÉS de darle tiempo a que termine la instrucción que
    # ya tenía en vuelo: la pausa corta en el próximo límite de instrucción, no
    # en el medio de una. Leerlo antes hacía fallar el test una de cada tantas
    # corridas, por una carrera del test, no del simulador.
    time.sleep(0.1)
    ejecutadas = sim.executed_instructions

    # Estando pausada, de acá no se mueve.
    time.sleep(0.2)
    assert sim.executed_instructions == ejecutadas

    client.resume()
    assert client.read_status().state == PlcVmState.RUNNING
    time.sleep(0.2)
    assert sim.executed_instructions > ejecutadas
    client.stop()


def test_a_second_start_does_not_restart_a_running_program():
    client, sim = _plc(tick_s=0.01)
    client.upload_program(
        Program(instructions=[Instruction("JUMP", b=0), Instruction("END")])
    )
    client.start()
    client.start()
    assert sim.state == PlcVmState.RUNNING
    client.stop()


# --- equivalencia con la VM de referencia ---------------------------------------------
#
# Dos implementaciones independientes del mismo contrato. Si divergen, una de las
# dos está leyendo mal docs/INSTRUCTION_SET.md.

PROGRAMAS_EQUIVALENTES = [
    pytest.param("SET_OUT(Y10, ON)\nSET_OUT(Y11, OFF)\n", id="salidas"),
    pytest.param("VAR n : INT = 5\nn = n + 3\n", id="add_var"),
    pytest.param("VAR a : INT = 1\nVAR b : INT = 9\na = b\n", id="copy_var"),
    pytest.param(
        "VAR n : INT = 1\nIF n == 1 THEN\nSET_OUT(Y10, ON)\nELSE\nSET_OUT(Y11, ON)\nENDIF\n",
        id="if_const_verdadero",
    ),
    pytest.param(
        "VAR n : INT = 4\nIF n == 1 THEN\nSET_OUT(Y10, ON)\nELSE\nSET_OUT(Y11, ON)\nENDIF\n",
        id="if_const_falso",
    ),
    pytest.param(
        "VAR a : INT = 3\nVAR b : INT = 3\nIF a == b THEN\nSET_OUT(Y12, ON)\nENDIF\n",
        id="if_var_var",
    ),
    pytest.param(
        "PROC hacer()\nSET_OUT(Y20, ON)\nENDPROC\nSET_OUT(Y21, ON)\nhacer()\n",
        id="proc_sin_parametros",
    ),
    pytest.param(
        "VAR total : INT = 0\nPROC sumar(x)\ntotal = total + 1\nENDPROC\n"
        "sumar(1)\nsumar(1)\n",
        id="proc_con_parametros",
    ),
    pytest.param(
        "VAR n : INT = 0\nPROC p(x)\nn = x\nENDPROC\np(7)\n",
        id="parametro_leido_en_el_cuerpo",
    ),
]


def _run_on_reference_vm(program):
    robot = BorunteRobotClient(host="sim", client=SimulatedBorunteRobot(0.01))
    robot.connect()
    plc_io = PlcIoSimulator()
    vm = ReferenceVM(program, robot, plc_io)
    vm.run_from(0)
    return vm, plc_io


@pytest.mark.parametrize("source", PROGRAMAS_EQUIVALENTES)
def test_reference_vm_and_plc_agree_on_outputs(source):
    program = compile_source(source)

    _, plc_io = _run_on_reference_vm(program)
    _, sim, _ = _run(source)

    numeros = {num for (_, num, _) in plc_io.output_log} | set(sim.outputs)
    referencia = {n: plc_io.read_output(n) for n in numeros}
    del_plc = {n: sim.read_output(n) for n in numeros}
    assert referencia == del_plc, (
        f"la VM de referencia y la del PLC discrepan en las salidas:\n"
        f"  referencia: {referencia}\n"
        f"  PLC:        {del_plc}"
    )


@pytest.mark.parametrize("source", PROGRAMAS_EQUIVALENTES)
def test_reference_vm_and_plc_agree_on_variables(source):
    program = compile_source(source)

    vm, _ = _run_on_reference_vm(program)
    _, sim, _ = _run(source)

    for name, index in program.var_names.items():
        de_referencia = vm.variables.get(index, 0)
        del_plc = sim.get_var(index)
        assert de_referencia == del_plc, (
            f"la variable {name!r} (índice {index}) quedó en {de_referencia} en la "
            f"VM de referencia y en {del_plc} en la del PLC"
        )
