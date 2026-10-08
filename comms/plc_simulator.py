"""
Simulador del PLC Coolmay CX3G corriendo la VM del contrato.

IMPORTANTE — dos advertencias distintas, no confundirlas:

1. **Esto implementa una HIPÓTESIS del mapa de registros**, igual que
   `comms/robot_simulator.py` implementa una hipótesis del comportamiento del
   robot. El mapa de `docs/INSTRUCTION_SET.md` lo definimos nosotros y nadie lo
   validó contra un CX3G. Si el PLC real expone los registros `D` distinto, hay
   que corregir el contrato y este archivo.

2. **Esto NO es la VM en ladder.** `plc_vm/` sigue vacío de código a propósito
   (regla 2 del proyecto): la VM real se escribe a mano en GX Developer/Works2.
   Lo que aporta este módulo es la **especificación ejecutable** de lo que ese
   ladder tiene que hacer — cada rama del `_step()` de acá debería tener un
   equivalente directo en una red de ladder. Si algo acá resulta difícil de
   expresar, es señal de que el opcode todavía no está listo para pasar a
   ladder.

Con esto el sistema de 2 niveles se puede ejercitar completo sin hardware: la
GUI habla Modbus con este PLC simulado, que ejecuta el bytecode y (si se le
pasa un robot) le manda los movimientos al simulador del robot.
"""

from __future__ import annotations

import threading
import time

from comms.fake_modbus import FakeModbusClient
from comms.plc_client import (
    CONTRACT_VERSION,
    D_BYTECODE_BASE,
    D_COMMAND,
    D_CONTRACT_VERSION,
    D_ERROR,
    D_INSTRUCTION_COUNT,
    D_PC,
    D_POINT_COUNT,
    D_POINTS_BASE,
    D_STATE,
    D_VAR_BASE,
    VAR_BANK_SIZE,
    WORDS_PER_INSTRUCTION,
    WORDS_PER_POINT,
    CoolmayPlcClient,
    PlcCommand,
    PlcVmError,
    PlcVmState,
)
from comms.robot_client import Pose
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
)

# Profundidad de la pila de retorno. El CX3G real tiene un límite propio de
# anidamiento de subrutinas que todavía no conocemos (está anotado como duda
# abierta en el contrato); acá se elige un número chico a propósito, para que
# un programa demasiado anidado falle en el simulador antes que en el PLC.
CALL_STACK_DEPTH = 8


class SimulatedCoolmayPlc(FakeModbusClient):
    """PLC simulado que ejecuta el bytecode que se le carga.

    Se le pasa a `CoolmayPlcClient` como `client`, igual que
    `SimulatedBorunteRobot` se le pasa a `BorunteRobotClient`.
    """

    def __init__(
        self,
        tick_s: float = 0.005,
        move_duration_s: float = 0.05,
        robot: object | None = None,
    ) -> None:
        """
        `tick_s`: pausa entre instrucciones. Existe para que el PC se pueda ver
        avanzar desde la GUI — un PLC real tiene su propio tiempo de ciclo.

        `robot`: opcional. Si se pasa un `BorunteRobotClient`, los MOVEJ/MOVEL
        se le mandan de verdad; si no, se simulan como una demora. Pasarlo es lo
        que permite ejercitar los dos niveles juntos.
        """
        super().__init__()
        # RLock y no Lock: write_register() del padre llama a _on_write(), que
        # vuelve a tomar el lock.
        self._lock = threading.RLock()
        self._tick_s = tick_s
        self._move_duration_s = move_duration_s
        self._robot = robot

        self._thread: threading.Thread | None = None
        self._stop_flag = threading.Event()
        self._pause_flag = threading.Event()
        self._call_stack: list[int] = []
        self.outputs: dict[int, bool] = {}
        self.inputs: dict[int, bool] = {}
        self.executed_instructions = 0

        self.registers[D_STATE] = int(PlcVmState.STOPPED)
        self.registers[D_CONTRACT_VERSION] = CONTRACT_VERSION

    # -- acceso a registros con lock -------------------------------------------

    def read_holding_registers(self, address: int, count: int = 1, slave: int = 1):
        with self._lock:
            return super().read_holding_registers(address, count=count, slave=slave)

    def write_register(self, address: int, value: int, slave: int = 1):
        with self._lock:
            return super().write_register(address, value, slave=slave)

    def write_registers(self, address: int, values: list[int], slave: int = 1):
        with self._lock:
            return super().write_registers(address, values, slave=slave)

    def _get(self, address: int) -> int:
        with self._lock:
            return self.registers.get(address, 0)

    def _set(self, address: int, value: int) -> None:
        with self._lock:
            self.registers[address] = int(value) & 0xFFFF

    # -- handshake de comandos ---------------------------------------------------

    def _on_write(self, address: int, values: list[int]) -> None:
        if address != D_COMMAND:
            return
        try:
            command = PlcCommand(values[0])
        except ValueError:
            # Comando desconocido: NO se acusa. Es lo que haría un ladder que no
            # sabe qué hacer con ese valor, y el cliente lo reporta como timeout.
            return
        self._handle_command(command)

    def _handle_command(self, command: PlcCommand) -> None:
        if command == PlcCommand.NONE:
            return
        if command == PlcCommand.START:
            self._start()
        elif command == PlcCommand.STOP:
            self._stop()
        elif command == PlcCommand.PAUSE:
            if self.state == PlcVmState.RUNNING:
                self._pause_flag.set()
                self._set(D_STATE, int(PlcVmState.PAUSED))
        elif command == PlcCommand.RESUME:
            if self.state == PlcVmState.PAUSED:
                self._pause_flag.clear()
                self._set(D_STATE, int(PlcVmState.RUNNING))
        elif command == PlcCommand.RESET:
            self._stop()
            self._set(D_ERROR, int(PlcVmError.NONE))
            self._set(D_PC, 0)
            self._set(D_STATE, int(PlcVmState.STOPPED))

        # Acuse: la VM devuelve D3 a 0. El cliente espera esto con timeout.
        self._set(D_COMMAND, int(PlcCommand.NONE))

    # -- estado ------------------------------------------------------------------

    @property
    def state(self) -> PlcVmState:
        try:
            return PlcVmState(self._get(D_STATE))
        except ValueError:
            return PlcVmState.ERROR

    @property
    def pc(self) -> int:
        return self._get(D_PC)

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def shutdown(self) -> None:
        """Corta la ejecución y espera al hilo.

        Un PLC real sigue corriendo cuando la GUI se cierra — eso es lo que se
        quiere. Pero este PLC vive DENTRO del proceso de la GUI, así que dejarlo
        girando después de que el panel se fue es un hilo huérfano tocando
        objetos que se están destruyendo, y eso crashea al salir.
        """
        self._stop()

    def wait_until_done(self, timeout_s: float = 10.0) -> bool:
        """Espera a que la ejecución termine. Devuelve False si se pasó el tiempo."""
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout_s)
        return not thread.is_alive()

    # -- arranque / parada --------------------------------------------------------

    def _start(self) -> None:
        if self.is_running():
            return  # ya corriendo; un segundo START no reinicia
        self._stop_flag.clear()
        self._pause_flag.clear()
        self._call_stack = []
        self.executed_instructions = 0
        self._set(D_PC, 0)
        self._set(D_ERROR, int(PlcVmError.NONE))
        self._set(D_STATE, int(PlcVmState.RUNNING))
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _stop(self) -> None:
        self._stop_flag.set()
        self._pause_flag.clear()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(2.0)
        if self.state == PlcVmState.RUNNING:
            self._set(D_STATE, int(PlcVmState.STOPPED))

    # -- ejecución -----------------------------------------------------------------

    def _fail(self, error: PlcVmError) -> None:
        self._set(D_ERROR, int(error))
        self._set(D_STATE, int(PlcVmState.ERROR))

    def _run(self) -> None:
        instruction_count = self._get(D_INSTRUCTION_COUNT)
        while not self._stop_flag.is_set():
            while self._pause_flag.is_set() and not self._stop_flag.is_set():
                time.sleep(self._tick_s)
            if self._stop_flag.is_set():
                break

            pc = self._get(D_PC)
            if not (0 <= pc < instruction_count):
                self._fail(PlcVmError.UNKNOWN_OPCODE)
                return

            try:
                instr = self._read_instruction(pc)
            except Exception:  # noqa: BLE001 — opcode que no está en la tabla
                self._fail(PlcVmError.UNKNOWN_OPCODE)
                return

            siguiente = self._step(instr, pc)
            self.executed_instructions += 1

            if siguiente is None:  # END o error
                return
            self._set(D_PC, siguiente)
            if self._tick_s:
                time.sleep(self._tick_s)

        if self.state == PlcVmState.RUNNING:
            self._set(D_STATE, int(PlcVmState.STOPPED))

    def _read_instruction(self, pc: int):
        base = D_BYTECODE_BASE + pc * WORDS_PER_INSTRUCTION
        with self._lock:
            words = [self.registers.get(base + i, 0) for i in range(WORDS_PER_INSTRUCTION)]
        return CoolmayPlcClient.decode_instruction(words)

    def _read_point(self, index: int) -> Pose:
        base = D_POINTS_BASE + index * WORDS_PER_POINT
        with self._lock:
            words = [self.registers.get(base + i, 0) for i in range(WORDS_PER_POINT)]
        return CoolmayPlcClient.decode_point(words)

    # -- variables (banco D10-D109) --------------------------------------------------

    def get_var(self, index: int) -> int:
        raw = self._get(D_VAR_BASE + index)
        return raw - 0x10000 if raw & 0x8000 else raw

    def set_var(self, index: int, value: int) -> bool:
        if not (0 <= index < VAR_BANK_SIZE):
            return False
        self._set(D_VAR_BASE + index, value)
        return True

    # -- una instrucción -------------------------------------------------------------

    def _step(self, instr, pc: int) -> int | None:
        """Ejecuta una instrucción. Devuelve el próximo PC, o None para terminar.

        Cada rama de acá es una red de ladder en el PLC real.
        """
        op = instr.opcode

        if op == END:
            self._set(D_STATE, int(PlcVmState.FINISHED))
            return None

        if op == NOP:
            return pc + 1

        if op == SET_VAR:
            if not self.set_var(instr.a, instr.b):
                self._fail(PlcVmError.VAR_INDEX_OUT_OF_RANGE)
                return None
            return pc + 1

        if op == ADD_VAR:
            if not (0 <= instr.a < VAR_BANK_SIZE):
                self._fail(PlcVmError.VAR_INDEX_OUT_OF_RANGE)
                return None
            self.set_var(instr.a, self.get_var(instr.a) + instr.b)
            return pc + 1

        if op == COPY_VAR:
            if not (0 <= instr.a < VAR_BANK_SIZE and 0 <= instr.b < VAR_BANK_SIZE):
                self._fail(PlcVmError.VAR_INDEX_OUT_OF_RANGE)
                return None
            self.set_var(instr.a, self.get_var(instr.b))
            return pc + 1

        if op == JUMP:
            return instr.b

        if op == JUMP_IF_ZERO:
            return instr.b if self.get_var(instr.a) == 0 else pc + 1

        if op == JUMP_IF_VAR_NEQ_CONST:
            return instr.c if self.get_var(instr.a) != instr.b else pc + 1

        if op == JUMP_IF_VAR_NEQ_VAR:
            return instr.c if self.get_var(instr.a) != self.get_var(instr.b) else pc + 1

        if op == CALL:
            if len(self._call_stack) >= CALL_STACK_DEPTH:
                self._fail(PlcVmError.CALL_STACK_OVERFLOW)
                return None
            self._call_stack.append(pc + 1)
            return instr.b

        if op == RET:
            if not self._call_stack:
                self._fail(PlcVmError.RET_WITHOUT_CALL)
                return None
            return self._call_stack.pop()

        if op == SET_OUT:
            self.outputs[instr.a] = bool(instr.b)
            return pc + 1

        if op == WAIT_TIME:
            self._sleep_interruptible(instr.b / 1000.0)
            return pc + 1

        if op == WAIT_IN:
            timeout_s = (instr.b / 1000.0) if instr.b else 3600.0
            if not self._wait_input(instr.a, timeout_s):
                self._fail(PlcVmError.WAIT_IN_TIMEOUT)
                return None
            return pc + 1

        if op in (MOVEJ, MOVEL):
            if not self._do_move(instr):
                return None
            return pc + 1

        if op == ALARM_CLEAR_CONTINUE:
            return pc + 1  # sin modelo de alarmas todavía

        self._fail(PlcVmError.UNKNOWN_OPCODE)
        return None

    def _do_move(self, instr) -> bool:
        if not (0 <= instr.b < self._get(D_POINT_COUNT)):
            self._fail(PlcVmError.POINT_INDEX_OUT_OF_RANGE)
            return False
        pose = self._read_point(instr.b)
        if self._robot is not None:
            try:
                self._robot.send_target_pose(pose)
                self._robot.cmd_start_button()
                self._robot.wait_until_stopped()
            except Exception:  # noqa: BLE001
                self._fail(PlcVmError.MOVE_TIMEOUT)
                return False
        else:
            self._sleep_interruptible(self._move_duration_s)
        return True

    def _sleep_interruptible(self, seconds: float) -> None:
        """Duerme, pero atendiendo un STOP. Un `time.sleep()` pelado dejaría a la
        VM sorda al comando durante todo un WAIT."""
        self._stop_flag.wait(seconds)

    def _wait_input(self, number: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._stop_flag.is_set():
                return False
            with self._lock:
                if self.inputs.get(number, False):
                    return True
            time.sleep(min(0.01, self._tick_s or 0.01))
        return False

    # -- para los tests y para "apretar botones" desde la GUI ---------------------

    def set_input(self, number: int, state: bool) -> None:
        with self._lock:
            self.inputs[number] = state

    def read_output(self, number: int) -> bool:
        with self._lock:
            return self.outputs.get(number, False)
