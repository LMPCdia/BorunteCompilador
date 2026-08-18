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
            if vm.stopped_by_request:
                self.finished.emit(False, "Ejecución detenida por el usuario.")
            else:
                self.finished.emit(True, "Ejecución completa.")
        except VmError as e:
            self.finished.emit(False, f"Error de ejecución: {e}")
        except Exception as e:  # noqa: BLE001 — queremos capturar cualquier cosa y mostrarla en la UI
            self.finished.emit(False, f"Error inesperado: {e!r}")

    def stop(self) -> None:
        """Pide la parada. Se llama desde el hilo de la UI, no desde el worker.

        Dos cosas separadas, porque son dos problemas distintos:

        1. `vm.request_stop()` corta el bucle de instrucciones en el próximo
           límite. Es solo un flag booleano, así que es seguro desde otro hilo.
        2. `robot.cmd_stop()` frena el robot. Sin esto, un movimiento en curso
           sigue hasta el final antes de que la VM llegue a evaluar el flag.

        ⚠️ Con hardware real, el punto 2 NO puede salir por el mismo socket
        Modbus que está usando el worker: dos hilos escribiendo en la misma
        conexión intercalan transacciones. Habría que abrir una segunda
        conexión para los comandos de parada. Contra el simulador no se nota
        porque no hay socket de verdad.

        Y lo que corresponde decir igual: un paro de emergencia de verdad va
        por una línea física al robot, no por Modbus ni por este botón.
        """
        if self._vm is not None:
            self._vm.request_stop()
        try:
            self.robot.cmd_stop()
        except Exception:  # noqa: BLE001 — si el robot no responde, el flag ya está puesto
            pass
