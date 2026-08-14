"""
Ejecuta la VM de referencia en un QThread aparte — la ejecución bloquea
(espera al robot, timers, WAIT_IN), así que no puede correr en el hilo de
la UI o la ventana se congela.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, Signal

from comms.robot_client import BorunteRobotClient
from runtime.bytecode import Program
from runtime.plc_io_simulator import PlcIoSimulator
from runtime.vm import ReferenceVM, VmError


class VmWorker(QObject):
    log_line = Signal(str)
    finished = Signal(bool, str)  # (éxito, mensaje)

    def __init__(self, program: Program, robot: BorunteRobotClient, plc_io: PlcIoSimulator) -> None:
        super().__init__()
        self.program = program
        self.robot = robot
        self.plc_io = plc_io
        self._vm: ReferenceVM | None = None

    def run(self) -> None:
        vm = ReferenceVM(self.program, self.robot, self.plc_io)
        self._vm = vm

        original_log = vm.trace.log

        def log_and_emit(msg: str) -> None:
            original_log(msg)
            self.log_line.emit(msg)

        vm.trace.log = log_and_emit  # type: ignore[method-assign]

        try:
            vm.run_from(0)
            self.finished.emit(True, "Ejecución completa.")
        except VmError as e:
            self.finished.emit(False, f"Error de ejecución: {e}")
        except Exception as e:  # noqa: BLE001 — queremos capturar cualquier cosa y mostrarla en la UI
            self.finished.emit(False, f"Error inesperado: {e!r}")
