"""
VM de referencia: ejecuta un Program (runtime/bytecode.py) contra un
BorunteRobotClient (real o simulado) y un PlcIoSimulator.

Sirve para probar un programa en la PC (contra el simulador) antes de
exportarlo al pad con compiler/pad_codegen.py. En producción el programa lo
ejecuta el controlador del Borunte, no esta VM.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from comms.robot_client import BorunteRobotClient
from runtime.bytecode import (
    ADD_VAR,
    ALARM_CLEAR_CONTINUE,
    CALL,
    COPY_VAR,
    END,
    JUMP,
    JUMP_IF_VAR_NEQ_CONST,
    JUMP_IF_VAR_NEQ_VAR,
    JUMP_IF_ZERO,
    MOVEJ,
    MOVEL,
    NOP,
    RET,
    SET_OUT,
    SET_VAR,
    WAIT_IN,
    WAIT_TIME,
    Program,
)
from runtime.plc_io_simulator import PlcIoSimulator


class VmError(Exception):
    pass


@dataclass
class VmTrace:
    """Log de ejecución, útil para debug y para los tests."""
    events: list[str] = field(default_factory=list)

    def log(self, msg: str) -> None:
        self.events.append(msg)


class ReferenceVM:
    def __init__(
        self,
        program: Program,
        robot: BorunteRobotClient,
        plc_io: PlcIoSimulator,
        max_steps: int = 100_000,
    ) -> None:
        self.program = program
        self.robot = robot
        self.plc_io = plc_io
        self.max_steps = max_steps
        self.variables: dict[int, float] = {}
        self.call_stack: list[int] = []
        self.trace = VmTrace()
        self._stop_requested = False
        self.stopped_by_request = False

    def request_stop(self) -> None:
        """Pide que la ejecución corte en el próximo límite de instrucción.

        Pensado para llamarse desde OTRO hilo que el que corre `run_from()`
        (en la GUI: el hilo de la UI pide, el worker ejecuta). Es un flag
        booleano, así que no hace falta lock.

        NO interrumpe el movimiento en curso: si el robot está moviéndose,
        `_exec_move()` está bloqueado en `wait_until_stopped()` y la VM recién
        corta cuando ese movimiento termina. Para frenar el robot en el
        momento hay que mandarle la orden a él (ver `VmWorker.stop()`), y un
        paro de emergencia de verdad va por una línea física, no por Modbus.
        """
        self._stop_requested = True

    def run_from(self, pc: int = 0) -> None:
        steps = 0
        while 0 <= pc < len(self.program.instructions):
            if self._stop_requested:
                self.stopped_by_request = True
                self.trace.log(f"PC={pc} detenido por pedido del usuario")
                return
            if steps >= self.max_steps:
                raise VmError(f"max_steps excedido (posible loop infinito) en PC={pc}")
            steps += 1

            instr = self.program.instructions[pc]
            self.trace.log(f"PC={pc} {instr!r}")

            if instr.opcode == END:
                return
            elif instr.opcode == NOP:
                pc += 1
            elif instr.opcode == MOVEJ:
                self._exec_move(instr, linear=False)
                pc += 1
            elif instr.opcode == MOVEL:
                self._exec_move(instr, linear=True)
                pc += 1
            elif instr.opcode == WAIT_IN:
                self._exec_wait_in(instr)
                pc += 1
            elif instr.opcode == SET_OUT:
                self.plc_io.set_output(instr.a, bool(instr.b))
                pc += 1
            elif instr.opcode == WAIT_TIME:
                import time
                time.sleep(instr.b / 1000.0)
                pc += 1
            elif instr.opcode == JUMP:
                pc = instr.b
            elif instr.opcode == JUMP_IF_ZERO:
                value = self.variables.get(instr.a, 0)
                pc = instr.b if value == 0 else pc + 1
            elif instr.opcode == JUMP_IF_VAR_NEQ_CONST:
                value = self.variables.get(instr.a, 0)
                pc = instr.c if value != instr.b else pc + 1
            elif instr.opcode == JUMP_IF_VAR_NEQ_VAR:
                left = self.variables.get(instr.a, 0)
                right = self.variables.get(instr.b, 0)
                pc = instr.c if left != right else pc + 1
            elif instr.opcode == COPY_VAR:
                self.variables[instr.a] = self.variables.get(instr.b, 0)
                pc += 1
            elif instr.opcode == CALL:
                self.call_stack.append(pc + 1)
                pc = instr.b
            elif instr.opcode == RET:
                if not self.call_stack:
                    raise VmError(f"RET sin CALL correspondiente en PC={pc}")
                pc = self.call_stack.pop()
            elif instr.opcode == SET_VAR:
                self.variables[instr.a] = instr.b
                pc += 1
            elif instr.opcode == ADD_VAR:
                self.variables[instr.a] = self.variables.get(instr.a, 0) + instr.b
                pc += 1
            elif instr.opcode == ALARM_CLEAR_CONTINUE:
                self.robot.cmd_clear_alarm_and_continue()
                pc += 1
            else:
                raise VmError(f"Opcode desconocido: {instr.opcode!r} en PC={pc}")

    def call_proc(self, name: str) -> None:
        if name not in self.program.proc_addresses:
            raise VmError(f"Procedimiento no encontrado: {name!r}")
        self.run_from(self.program.proc_addresses[name])

    # -- ejecución de instrucciones que hablan con el robot -----------------

    def _exec_move(self, instr, linear: bool) -> None:
        pose = self.program.points[instr.b]
        speed = instr.d
        self.trace.log(f"  -> {'MOVEL' if linear else 'MOVEJ'} a {pose} vel={speed}")
        self.robot.send_target_pose(pose)
        self.robot.cmd_start_button()
        self.robot.wait_until_stopped()

    def _exec_wait_in(self, instr) -> None:
        number = instr.a
        timeout_ms = instr.b
        ok = self.plc_io.wait_input(number, timeout_s=timeout_ms / 1000.0 if timeout_ms else 3600)
        if not ok:
            raise VmError(f"Timeout esperando entrada {number}")
