"""
Panel de la VM del PLC: conectar al CX3G, cargar el programa compilado,
arrancar/parar/pausar/resetear, y ver PC + estado + variables en vivo.

Ojo con la distinción, porque es la que más confunde en esta aplicación:

- El botón **Ejecutar** de la barra de herramientas corre `runtime/vm.py`, la VM
  de referencia, **en la PC**, hablándole al robot por Modbus.
- Este panel carga el bytecode **en el PLC** y le pide que lo ejecute **él**,
  que es el modo en que el sistema va a funcionar de verdad.

Los dos hacen lo mismo desde afuera y por caminos completamente distintos. Que
den el mismo resultado es justamente lo que valida el contrato.

⚠️ Del otro lado hay una VM que **todavía no existe en ladder**. Contra el
simulador (`comms/plc_simulator.py`) esto anda; contra un CX3G real no se probó
nunca, y hasta que alguien escriba el ladder no hay nada que responda el
handshake.
"""

from __future__ import annotations

import contextlib
import time

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from comms.plc_client import (
    D_VAR_BASE,
    MAX_INSTRUCTIONS,
    MAX_POINTS,
    CoolmayModbusError,
    CoolmayPlcClient,
    PlcVmState,
    VmStatus,
)
from comms.plc_simulator import SimulatedCoolmayPlc
from gui.plc_status_worker import PlcStatusWorker
from runtime.bytecode import Program

VARIABLES_MOSTRADAS = 16

ESTADO_TEXTO = {
    PlcVmState.STOPPED: ("Parada", "#7f8c8d"),
    PlcVmState.RUNNING: ("Corriendo", "#1f8b4c"),
    PlcVmState.ERROR: ("Error", "#c0392b"),
    PlcVmState.PAUSED: ("Pausada", "#b8860b"),
    PlcVmState.FINISHED: ("Terminada", "#2c7bb6"),
}


class PlcPanel(QWidget):
    """Panel completo del nivel PLC. Emite `message` para la ventana de mensajes."""

    message = Signal(str, str, str)  # (severidad: info|warning|error, texto, origen)

    def __init__(self) -> None:
        super().__init__()
        self._plc: CoolmayPlcClient | None = None
        self._simulator: SimulatedCoolmayPlc | None = None
        self._thread: QThread | None = None
        self._worker: PlcStatusWorker | None = None
        self._uploaded_instructions = 0
        self._last_status: VmStatus | None = None
        # Hilos que no terminaron a tiempo. Se retienen a proposito: ver
        # _stop_polling().
        self._leaked_threads: list[tuple[object, object]] = []

        self._build_ui()
        self._refresh_enabled()

    # -- construcción -----------------------------------------------------------

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.addWidget(self._build_connection_box())
        layout.addWidget(self._build_program_box())
        layout.addWidget(self._build_control_box())
        layout.addWidget(self._build_status_box())
        layout.addWidget(self._build_variables_box(), stretch=1)

    def _build_connection_box(self) -> QGroupBox:
        box = QGroupBox("PLC")
        self.mode_sim = QRadioButton("Simulado (sin hardware)")
        self.mode_real = QRadioButton("CX3G real")
        self.mode_sim.setChecked(True)

        self.ip_edit = QLineEdit("192.168.1.20")
        self.ip_edit.setEnabled(False)
        self.mode_real.toggled.connect(self.ip_edit.setEnabled)

        self.connect_btn = QPushButton("Conectar")
        self.connect_btn.clicked.connect(self._on_connect)
        self.disconnect_btn = QPushButton("Desconectar")
        self.disconnect_btn.clicked.connect(self._on_disconnect)

        self.connection_label = QLabel("Sin conectar")

        row = QHBoxLayout()
        row.addWidget(self.mode_sim)
        row.addWidget(self.mode_real)
        row.addWidget(QLabel("IP:"))
        row.addWidget(self.ip_edit)
        row.addWidget(self.connect_btn)
        row.addWidget(self.disconnect_btn)
        row.addStretch()
        row.addWidget(self.connection_label)

        outer = QVBoxLayout(box)
        outer.addLayout(row)
        return box

    def _build_program_box(self) -> QGroupBox:
        box = QGroupBox("Programa en el PLC")
        self.upload_btn = QPushButton("Cargar programa compilado")
        self.upload_btn.clicked.connect(self._on_upload)
        self.verify_btn = QPushButton("Verificar carga")
        self.verify_btn.clicked.connect(self._on_verify)

        self.capacity_instr = QProgressBar()
        self.capacity_instr.setMaximum(MAX_INSTRUCTIONS)
        self.capacity_instr.setFormat(f"%v / {MAX_INSTRUCTIONS} instrucciones")
        self.capacity_points = QProgressBar()
        self.capacity_points.setMaximum(MAX_POINTS)
        self.capacity_points.setFormat(f"%v / {MAX_POINTS} puntos")

        row = QHBoxLayout()
        row.addWidget(self.upload_btn)
        row.addWidget(self.verify_btn)
        row.addStretch()

        outer = QVBoxLayout(box)
        outer.addLayout(row)
        outer.addWidget(self.capacity_instr)
        outer.addWidget(self.capacity_points)
        return box

    def _build_control_box(self) -> QGroupBox:
        box = QGroupBox("Control de la VM")
        self.start_btn = QPushButton("Arrancar")
        self.stop_btn = QPushButton("Parar")
        self.pause_btn = QPushButton("Pausar")
        self.resume_btn = QPushButton("Continuar")
        self.reset_btn = QPushButton("Reset")

        self.start_btn.clicked.connect(lambda: self._send("start", "Arrancada"))
        self.stop_btn.clicked.connect(lambda: self._send("stop", "Parada"))
        self.pause_btn.clicked.connect(lambda: self._send("pause", "Pausada"))
        self.resume_btn.clicked.connect(lambda: self._send("resume", "Continuada"))
        self.reset_btn.clicked.connect(lambda: self._send("reset", "Reseteada"))

        row = QHBoxLayout(box)
        for btn in (
            self.start_btn, self.stop_btn, self.pause_btn,
            self.resume_btn, self.reset_btn,
        ):
            row.addWidget(btn)
        row.addStretch()
        return box

    def _build_status_box(self) -> QGroupBox:
        box = QGroupBox("Estado en vivo")
        self.state_label = QLabel("—")
        self.pc_label = QLabel("—")
        self.error_label = QLabel("—")
        self.contract_label = QLabel("—")
        self.counts_label = QLabel("—")

        grid = QGridLayout(box)
        for col, (titulo, widget) in enumerate([
            ("Estado", self.state_label),
            ("Program Counter", self.pc_label),
            ("Error", self.error_label),
            ("Contrato", self.contract_label),
            ("Cargado", self.counts_label),
        ]):
            grid.addWidget(QLabel(f"<b>{titulo}</b>"), 0, col)
            grid.addWidget(widget, 1, col)
        return box

    def _build_variables_box(self) -> QGroupBox:
        box = QGroupBox(f"Banco de variables (primeras {VARIABLES_MOSTRADAS})")
        self.variables_table = QTableWidget(0, 3)
        self.variables_table.setHorizontalHeaderLabels(["Registro", "Nombre", "Valor"])
        self.variables_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.variables_table.verticalHeader().setVisible(False)
        self.variables_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        outer = QVBoxLayout(box)
        outer.addWidget(self.variables_table)
        return box

    # -- programa compilado (lo inyecta la ventana) --------------------------------

    def set_program(self, program: Program | None) -> None:
        self._program = program
        if program is not None:
            self.capacity_instr.setValue(min(len(program.instructions), MAX_INSTRUCTIONS))
            self.capacity_points.setValue(min(len(program.points), MAX_POINTS))
        else:
            self.capacity_instr.setValue(0)
            self.capacity_points.setValue(0)
        self._refresh_enabled()

    @property
    def program(self) -> Program | None:
        return getattr(self, "_program", None)

    # -- conexión -------------------------------------------------------------------

    def is_connected(self) -> bool:
        return self._plc is not None

    def _on_connect(self) -> None:
        try:
            if self.mode_sim.isChecked():
                self._simulator = SimulatedCoolmayPlc()
                plc = CoolmayPlcClient(host="simulador", client=self._simulator)
            else:
                self._simulator = None
                plc = CoolmayPlcClient(host=self.ip_edit.text())
            if not plc.connect():
                raise CoolmayModbusError("No se pudo abrir la conexión")
            self._plc = plc
        except Exception as e:  # noqa: BLE001
            self._plc = None
            self._simulator = None
            self.connection_label.setText("Error")
            self.message.emit("error", f"No se pudo conectar al PLC: {e}", "plc")
            self._refresh_enabled()
            return

        destino = "simulador" if self.mode_sim.isChecked() else self.ip_edit.text()
        self.connection_label.setText(f"Conectado ({destino})")
        self.message.emit("info", f"Conectado al PLC ({destino}).", "plc")
        if self.mode_real.isChecked():
            self.message.emit(
                "warning",
                "El nivel PLC no se probó nunca contra un CX3G real: el mapa de "
                "registros es propuesta y la VM en ladder todavía no existe.",
                "plc",
            )
        self._start_polling()
        self._refresh_enabled()

    def _on_disconnect(self) -> None:
        self._stop_polling()
        if self._plc is not None:
            try:
                self._plc.close()
            except Exception:  # noqa: BLE001
                pass
        # El PLC simulado vive dentro de este proceso: si se lo deja
        # ejecutando, queda un hilo huérfano tocando objetos que ya se están
        # destruyendo. Un CX3G real, en cambio, sigue corriendo solo — y eso es
        # lo correcto, así que esto aplica únicamente al simulador.
        if self._simulator is not None:
            self._simulator.shutdown()
        self._plc = None
        self._simulator = None
        self._last_status = None
        self.connection_label.setText("Sin conectar")
        self._clear_status()
        self.message.emit("info", "Desconectado del PLC.", "plc")
        self._refresh_enabled()

    # -- poll del estado ---------------------------------------------------------------

    def _start_polling(self) -> None:
        if self._plc is None or self._thread is not None:
            return
        self._thread = QThread()
        self._worker = PlcStatusWorker(self._plc, variable_count=VARIABLES_MOSTRADAS)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.status_read.connect(self._on_status)
        self._worker.failed.connect(self._on_poll_failed)
        self._thread.start()

    def _stop_polling(self) -> None:
        """Baja el hilo de poll del todo. Solo al desconectar o al cerrar.

        Las señales se desconectan ANTES de soltar las referencias: si queda una
        emisión encolada hacia el hilo de la UI y el worker ya fue recolectado,
        Qt toca un objeto C++ destruido y el proceso crashea. Y el `wait()`
        tampoco es opcional, por lo mismo.

        Para las operaciones normales (comandos, carga) NO se usa esto: se pausa
        con `_pause_polling()`, que es más barato y no tiene ese riesgo.
        """
        # Los atributos se limpian PRIMERO y una sola vez. Si se limpiaran al
        # final, el camino de timeout salía sin limpiarlos y una segunda llamada
        # volvía a desconectar señales que ya no estaban (Qt avisa con un
        # RuntimeWarning por cada una).
        worker, thread = self._worker, self._thread
        self._worker = None
        self._thread = None
        if worker is None and thread is None:
            return

        if worker is not None:
            worker.request_stop()
            with contextlib.suppress(RuntimeError, TypeError):
                worker.status_read.disconnect(self._on_status)
            with contextlib.suppress(RuntimeError, TypeError):
                worker.failed.disconnect(self._on_poll_failed)

        if thread is not None:
            thread.quit()
            if not thread.wait(3000):
                # Un hilo que no terminó no se suelta nunca: si Python recolecta
                # el QThread con el hilo vivo, Qt destruye el objeto C++ por
                # debajo y el proceso crashea. Preferimos la fuga.
                self._leaked_threads.append((worker, thread))
                self.message.emit(
                    "warning",
                    "El hilo de poll del PLC no terminó a tiempo; queda retenido "
                    "para no arriesgar un cierre sucio.",
                    "plc",
                )

    def _pause_polling(self) -> None:
        """Pausa el poll y espera a que el worker suelte el socket.

        Sin esta espera, el comando del panel y la lectura del worker se
        intercalarían sobre la misma conexión Modbus y las respuestas se
        cruzarían: el worker leería la respuesta del comando del panel como si
        fuera su estado.
        """
        if self._worker is None:
            return
        self._worker.set_paused(True)
        deadline = time.monotonic() + 2.0
        while not self._worker.is_idle() and time.monotonic() < deadline:
            QThread.msleep(5)

    def _resume_polling(self) -> None:
        if self._worker is not None:
            self._worker.set_paused(False)

    def _on_poll_failed(self, mensaje: str) -> None:
        self.message.emit("error", mensaje, "plc")
        self._on_disconnect()

    def _on_status(self, status: VmStatus, variables: list) -> None:
        self._last_status = status
        texto, color = ESTADO_TEXTO.get(status.state, (str(status.state), "#7f8c8d"))
        self.state_label.setText(f"<b style='color:{color}'>{texto}</b>")
        self.pc_label.setText(str(status.pc))
        if status.error:
            self.error_label.setText(
                f"<b style='color:#c0392b'>{status.error} — {status.error_name}</b>"
            )
        else:
            self.error_label.setText("sin error")
        self.contract_label.setText(f"v{status.contract_version}")
        self.counts_label.setText(
            f"{status.instruction_count} instr · {status.point_count} puntos"
        )
        self._refresh_variables(variables)
        self._refresh_enabled()

    def _refresh_variables(self, variables: list) -> None:
        nombres = self._variable_names()
        self.variables_table.setRowCount(len(variables))
        for i, value in enumerate(variables):
            self.variables_table.setItem(i, 0, QTableWidgetItem(f"D{D_VAR_BASE + i}"))
            self.variables_table.setItem(i, 1, QTableWidgetItem(nombres.get(i, "")))
            item = QTableWidgetItem(str(value))
            item.setTextAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
            )
            self.variables_table.setItem(i, 2, item)

    def _variable_names(self) -> dict[int, str]:
        """Índice de variable -> nombre, tomado del programa compilado.

        El PLC solo tiene números; los nombres los sabe el compilador. Sin esto
        el panel muestra 16 filas de enteros sin decir qué es cada uno.
        """
        program = self.program
        if program is None:
            return {}
        return {index: name for name, index in program.var_names.items()}

    def _clear_status(self) -> None:
        for label in (
            self.state_label, self.pc_label, self.error_label,
            self.contract_label, self.counts_label,
        ):
            label.setText("—")
        self.variables_table.setRowCount(0)

    # -- carga y comandos -----------------------------------------------------------------

    def _on_upload(self) -> None:
        if self._plc is None or self.program is None:
            return
        # El poll se pausa primero: cargar son cientos de transacciones y el
        # worker está leyendo por el mismo socket.
        self._pause_polling()
        try:
            self._plc.upload_program(self.program)
            self._plc.verify_program(self.program)
        except CoolmayModbusError as e:
            self.message.emit("error", f"Falló la carga en el PLC: {e}", "plc")
            self._resume_polling()
            self._refresh_enabled()
            return
        except Exception as e:  # noqa: BLE001
            self.message.emit("error", f"Error inesperado cargando el PLC: {e!r}", "plc")
            self._resume_polling()
            self._refresh_enabled()
            return

        self._uploaded_instructions = len(self.program.instructions)
        self.message.emit(
            "info",
            f"Cargadas y verificadas {self._uploaded_instructions} instrucciones y "
            f"{len(self.program.points)} puntos en el PLC.",
            "plc",
        )
        self._resume_polling()
        self._refresh_enabled()

    def _on_verify(self) -> None:
        if self._plc is None or self.program is None:
            return
        self._pause_polling()
        try:
            self._plc.verify_program(self.program)
            self.message.emit("info", "La carga del PLC coincide con el programa.", "plc")
        except CoolmayModbusError as e:
            self.message.emit("error", f"La verificación falló: {e}", "plc")
        finally:
            self._resume_polling()

    def _send(self, comando: str, descripcion: str) -> None:
        if self._plc is None:
            return
        self._pause_polling()
        try:
            getattr(self._plc, comando)()
            self.message.emit("info", f"VM del PLC: {descripcion}.", "plc")
        except CoolmayModbusError as e:
            # El caso típico: no hay ladder del otro lado que acuse el comando.
            self.message.emit("error", f"El PLC no aceptó el comando: {e}", "plc")
        except Exception as e:  # noqa: BLE001
            self.message.emit("error", f"Error mandando el comando: {e!r}", "plc")
        finally:
            self._resume_polling()
            self._refresh_enabled()

    # -- habilitación de controles ----------------------------------------------------------

    def _refresh_enabled(self) -> None:
        conectado = self.is_connected()
        estado = self._last_status.state if self._last_status else None
        corriendo = estado == PlcVmState.RUNNING
        pausada = estado == PlcVmState.PAUSED

        self.connect_btn.setEnabled(not conectado)
        self.disconnect_btn.setEnabled(conectado)
        self.mode_sim.setEnabled(not conectado)
        self.mode_real.setEnabled(not conectado)

        hay_programa = self.program is not None
        # No se carga con la VM corriendo: reescribiría el bytecode bajo los pies
        # del PC que lo está ejecutando.
        self.upload_btn.setEnabled(conectado and hay_programa and not corriendo)
        self.verify_btn.setEnabled(conectado and hay_programa and not corriendo)

        cargado = self._uploaded_instructions > 0
        self.start_btn.setEnabled(conectado and cargado and not corriendo and not pausada)
        self.stop_btn.setEnabled(conectado and (corriendo or pausada))
        self.pause_btn.setEnabled(conectado and corriendo)
        self.resume_btn.setEnabled(conectado and pausada)
        self.reset_btn.setEnabled(conectado and not corriendo)

    # -- cierre -------------------------------------------------------------------------------

    def shutdown(self) -> None:
        """Para el hilo de poll y el PLC simulado. La ventana lo llama al cerrarse."""
        self._stop_polling()
        if self._simulator is not None:
            self._simulator.shutdown()
