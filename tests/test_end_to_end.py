"""
Test de punta a punta: compila un programa DSL real, lo corre en la VM de
referencia contra un robot simulado (sin hardware), y verifica que terminó
en la posición esperada y que las salidas se activaron en el orden correcto.

Este es el test que demuestra que el pipeline completo (lenguaje -> bytecode
-> ejecución) funciona en software, independientemente de si el hardware
real se comporta igual (eso se valida aparte, ver docs/ARCHITECTURE.md).
"""

from comms.robot_client import BorunteRobotClient, Pose
from comms.robot_simulator import SimulatedBorunteRobot
from compiler.codegen import compile_source
from runtime.plc_io_simulator import PlcIoSimulator
from runtime.vm import ReferenceVM

PROGRAM = """POINT p_home = WORLD(0.0, 500.0, 300.0, 0.0, 0.0, 0.0)
POINT p_pieza = WORLD(100.0, 600.0, 200.0, 0.0, 0.0, 0.0)
WAIT_IN(X010, 2)
MOVEJ p_home SPEED 80
MOVEL p_pieza SPEED 50
SET_OUT(Y10, ON)
WAIT 0.05s
SET_OUT(Y10, OFF)
MOVEJ p_home SPEED 80
"""


def test_end_to_end_against_simulator():
    program = compile_source(PROGRAM)

    sim = SimulatedBorunteRobot(move_duration_s=0.05)
    robot = BorunteRobotClient(host="fake", client=sim)
    robot.connect()

    plc_io = PlcIoSimulator()
    plc_io.set_input(10, True)  # X010 ya está activa antes de correr el programa

    vm = ReferenceVM(program, robot, plc_io)
    vm.run_from(0)

    # El programa termina en p_home
    final_pose = robot.read_world_position()
    assert abs(final_pose.a - 0.0) < 0.5
    assert abs(final_pose.b - 500.0) < 0.5
    assert abs(final_pose.c - 300.0) < 0.5

    # Y10 se prendió y se apagó, en ese orden
    states = [state for (_, num, state) in plc_io.output_log if num == 10]
    assert states == [True, False], f"Secuencia de Y10 inesperada: {states}"

    print("\n--- bytecode generado ---")
    print(program.dump())
    print("\n--- trace de ejecución ---")
    print("\n".join(vm.trace.events))


def test_wait_in_times_out_if_input_never_arrives():
    program = compile_source("WAIT_IN(X099, 0.05)\n")
    sim = SimulatedBorunteRobot()
    robot = BorunteRobotClient(host="fake", client=sim)
    robot.connect()
    plc_io = PlcIoSimulator()  # X099 nunca se activa

    vm = ReferenceVM(program, robot, plc_io)
    import pytest
    from runtime.vm import VmError
    with pytest.raises(VmError):
        vm.run_from(0)
