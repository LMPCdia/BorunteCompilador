"""
Ventana principal, con la disposición de paneles acoplables al estilo
WorkVisual: estructura del proyecto y campos de trabajo a la izquierda,
propiedades a la derecha, mensajes y log abajo, y el editor / bytecode /
puntos en el centro.

    ┌───────────────── menú + barra de herramientas ─────────────────┐
    │ Estructura   │ Programa │ Bytecode │ Puntos │ Pad │ Sim 3D │ Prop. │
    │ del proyecto │                                    │             │
    ├──────────────┤                                    │             │
    │ Campos de    │                                    │             │
    │ trabajo      ├────────────────────────────────────┴─────────────┤
    │              │ Ventana de mensajes │ Log de ejecución            │
    ├──────────────┴──────────────────────────────────────────────────┤
    │ barra de estado: conexión · estado de la VM · puntos · cursor   │
    └─────────────────────────────────────────────────────────────────┘
"""

from __future__ import annotations

import re
from pathlib import Path

from PySide6.QtCore import Qt, QThread
from PySide6.QtGui import QAction, QFont, QKeySequence, QTextCursor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QDockWidget,
    QFrame,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QStyle,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from comms.robot_client import Pose
from compiler.codegen import CompileError, compile_source
from compiler.pad_codegen import PadOptions, compile_to_pad_report
from gui.connection_panel import ConnectionPanel
from gui.message_window import MessageWindow
from gui.project_tree import POSE_AXIS_NAMES, ProjectTree
from gui.properties_panel import PropertiesPanel
from gui.sim_view import SimView
from gui.syntax_highlighter import DslSyntaxHighlighter
from gui.vm_worker import VmWorker
from gui.work_fields import WorkFieldsPanel, field_by_key
from pad.listing import list_backup
from runtime.bytecode import Program
from runtime.plc_io_simulator import PlcIoSimulator

EXAMPLE_PROGRAM = """; Programa de ejemplo: un cordón de soldadura de 300 mm
; MOVEJ va a puntos JOINT (ángulos de eje) y MOVEL a puntos WORLD (X,Y,Z,U,V,W).
; Las E/S se escriben como en el pad: X010, Y034, ...
; TOOL n / COORD n eligen la herramienta y el sistema de coordenadas del pad.
POINT p_home = JOINT(0.347, 45.894, -44.865, -0.792, -75.952, -0.859)
POINT p_pieza = WORLD(1097.1, -150.0, 721.9, 180.0, -10.0, 180.0)

PROC soldar_pieza()
MOVEJ p_home SPEED 80
MOVEL p_pieza + OFFSET(0, 0, 100, 0, 0, 0) SPEED 50
MOVEL p_pieza SPEED 10
SET_OUT(Y010, ON)
MOVEL p_pieza + OFFSET(0, 300, 0, 0, 0, 0) SPEED 5
SET_OUT(Y010, OFF)
MOVEL p_pieza + OFFSET(0, 300, 100, 0, 0, 0) SPEED 50
MOVEJ p_home SPEED 80
ENDPROC

soldar_pieza()
"""

MONOSPACE = "Consolas, Menlo, monospace"


def _scrollable(widget: QWidget) -> QScrollArea:
    """Envuelve un panel para que, si la ventana es más angosta que lo que el
    panel necesita, aparezca una barra de desplazamiento en vez de agrandar el
    ancho mínimo de TODA la ventana. Sin esto, en Windows (fuentes y estilos
    más anchos que en Linux) la ventana pedía 2300 px y no entraba en pantalla."""
    area = QScrollArea()
    area.setWidget(widget)
    area.setWidgetResizable(True)
    area.setFrameShape(QFrame.Shape.NoFrame)
    return area


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Borunte DSL")
        self.resize(1400, 860)

        self._program: Program | None = None
        # Archivo abierto/guardado: de ahí sale el nombre del programa en el pad.
        self._current_path: Path | None = None
        self._plc_io = PlcIoSimulator()
        self._thread: QThread | None = None
        self._worker: VmWorker | None = None
        # Los puntos digitalizados viven APARTE de program.points: recompilar
        # reconstruye esa lista y los borraría.
        self._digitized: list[tuple[str, Pose]] = []
        self._docks: dict[str, QDockWidget] = {}

        self._build_central()
        self._build_docks()
        self._build_actions()
        self._build_menus()
        self._build_toolbar()
        self._build_status_bar()
        self._connect_signals()

        self.messages.info("Listo. F7 para compilar.", "gui")
        for severity, message in self._pending_reports:
            self._sim_report(severity, message)
        self._pending_reports = []
        self._apply_work_field("programacion")
        self._refresh_action_states()

    # -- construcción: centro --------------------------------------------------

    def _build_central(self) -> None:
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.editor = QPlainTextEdit()
        self.editor.setFont(QFont(MONOSPACE, 11))
        self.editor.setPlainText(EXAMPLE_PROGRAM)
        self.highlighter = DslSyntaxHighlighter(
            self.editor.document(), qt_palette=self.editor.palette()
        )
        self.tabs.addTab(self.editor, "Programa")

        self.bytecode_view = QPlainTextEdit()
        self.bytecode_view.setReadOnly(True)
        self.bytecode_view.setFont(QFont(MONOSPACE, 10))
        self.tabs.addTab(self.bytecode_view, "Bytecode")

        self.tabs.addTab(self._build_points_tab(), "Puntos")

        # Listado de lo que se exportó al pad, para revisarlo antes de llevarlo
        # al robot.
        self.pad_view = QPlainTextEdit()
        self.pad_view.setReadOnly(True)
        self.pad_view.setFont(QFont(MONOSPACE, 10))
        self.tabs.addTab(self.pad_view, "Pad")

        # Simulación 3D del respaldo del pad (sim/). La vista 3D solo se crea
        # si hay OpenGL: ver gui/viewport3d.py.
        self._pending_reports: list[tuple[str, str]] = []
        self.sim_view = SimView(self.editor.toPlainText, report=self._sim_report,
                                clear_reports=lambda: self.messages.clear_source("simulador"))
        self.sim_tab = _scrollable(self.sim_view)
        self.tabs.addTab(self.sim_tab, "Simulación 3D")
        self.editor.textChanged.connect(self.sim_view.source_changed)

    def _build_points_tab(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        splitter = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(splitter)

        # -- tabla del programa compilado (solo lectura) --
        program_side = QWidget()
        program_layout = QVBoxLayout(program_side)
        program_layout.setContentsMargins(0, 0, 0, 0)
        program_layout.addWidget(QLabel("Tabla de puntos del programa compilado (solo lectura)"))
        self.points_table = QTableWidget(0, 1 + len(POSE_AXIS_NAMES))
        self.points_table.setHorizontalHeaderLabels(["#", *POSE_AXIS_NAMES])
        self.points_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.points_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        program_layout.addWidget(self.points_table)
        splitter.addWidget(program_side)

        # -- tabla de digitalizados (nombre editable) --
        digit_side = QWidget()
        digit_layout = QVBoxLayout(digit_side)
        digit_layout.setContentsMargins(0, 0, 0, 0)
        digit_layout.addWidget(
            QLabel("Puntos digitalizados desde el robot (nombre editable)")
        )
        self.digitized_table = QTableWidget(0, 1 + len(POSE_AXIS_NAMES))
        self.digitized_table.setHorizontalHeaderLabels(["Nombre", *POSE_AXIS_NAMES])
        self.digitized_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        digit_layout.addWidget(self.digitized_table)

        row = QHBoxLayout()
        self.digitize_btn = QPushButton("Digitalizar punto (F8)")
        self.delete_digitized_btn = QPushButton("Borrar digitalizado")
        self.insert_points_btn = QPushButton("Insertar como POINT en el editor")
        row.addWidget(self.digitize_btn)
        row.addWidget(self.delete_digitized_btn)
        row.addStretch()
        row.addWidget(self.insert_points_btn)
        digit_layout.addLayout(row)
        splitter.addWidget(digit_side)

        return container

    # -- construcción: paneles acoplables --------------------------------------

    def _add_dock(
        self,
        key: str,
        title: str,
        widget: QWidget,
        area: Qt.DockWidgetArea,
    ) -> QDockWidget:
        dock = QDockWidget(title, self)
        dock.setObjectName(f"dock_{key}")
        dock.setWidget(_scrollable(widget))
        self.addDockWidget(area, dock)
        self._docks[key] = dock
        return dock

    def _build_docks(self) -> None:
        self.project_tree = ProjectTree()
        self.project_tree.rebuild(None)
        self._add_dock(
            "estructura", "Estructura del proyecto",
            self.project_tree, Qt.DockWidgetArea.LeftDockWidgetArea,
        )

        self.work_fields = WorkFieldsPanel()
        self._add_dock(
            "campos", "Campos de trabajo",
            self.work_fields, Qt.DockWidgetArea.LeftDockWidgetArea,
        )

        self.properties_panel = PropertiesPanel()
        self._add_dock(
            "propiedades", "Propiedades",
            self.properties_panel, Qt.DockWidgetArea.RightDockWidgetArea,
        )

        self.connection_panel = ConnectionPanel()
        # El QGroupBox ya traía su propio título "Conexión" y el dock agrega
        # otro igual arriba: se veía dos veces.
        self.connection_panel.setTitle("")
        self._add_dock(
            "conexion", "Conexión",
            self.connection_panel, Qt.DockWidgetArea.RightDockWidgetArea,
        )

        self.messages = MessageWindow()
        self._add_dock(
            "mensajes", "Ventana de mensajes",
            self.messages, Qt.DockWidgetArea.BottomDockWidgetArea,
        )

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont(MONOSPACE, 10))
        self._add_dock(
            "log", "Log de ejecución",
            self.log_view, Qt.DockWidgetArea.BottomDockWidgetArea,
        )

    # -- construcción: acciones, menú, barra -----------------------------------

    def _icon(self, pixmap: QStyle.StandardPixmap):
        return self.style().standardIcon(pixmap)

    def _build_actions(self) -> None:
        sp = QStyle.StandardPixmap

        self.act_open = QAction(self._icon(sp.SP_DialogOpenButton), "&Abrir…", self)
        self.act_open.setShortcut(QKeySequence.StandardKey.Open)

        self.act_save = QAction(self._icon(sp.SP_DialogSaveButton), "&Guardar…", self)
        self.act_save.setShortcut(QKeySequence.StandardKey.Save)

        self.act_quit = QAction("&Salir", self)
        self.act_quit.setShortcut(QKeySequence.StandardKey.Quit)

        self.act_compile = QAction(self._icon(sp.SP_BrowserReload), "&Compilar", self)
        self.act_compile.setShortcut(QKeySequence("F7"))

        self.act_run = QAction(self._icon(sp.SP_MediaPlay), "&Ejecutar", self)
        self.act_run.setShortcut(QKeySequence("F5"))
        self.act_run.setEnabled(False)

        self.act_stop = QAction(self._icon(sp.SP_MediaStop), "&Parar", self)
        self.act_stop.setShortcut(QKeySequence("Shift+F5"))
        self.act_stop.setEnabled(False)

        self.act_export_pad = QAction(self._icon(sp.SP_DriveFDIcon), "E&xportar para el pad…", self)
        self.act_export_pad.setShortcut(QKeySequence("Ctrl+E"))

        # Apagado por defecto: lo que el respaldo real nunca mostró no va al
        # robot salvo que se pida a propósito (ver compiler/pad_codegen.py).
        self.act_allow_unverified = QAction("Permitir instrucciones sin confirmar en el pad", self)
        self.act_allow_unverified.setCheckable(True)
        self.act_allow_unverified.setChecked(False)

        self.act_digitize = QAction(self._icon(sp.SP_ArrowDown), "&Digitalizar punto", self)
        self.act_digitize.setShortcut(QKeySequence("F8"))
        self.act_digitize.setEnabled(False)

        self.act_about = QAction("&Acerca de", self)

    def _build_menus(self) -> None:
        bar = self.menuBar()

        m_file = bar.addMenu("&Archivo")
        m_file.addAction(self.act_open)
        m_file.addAction(self.act_save)
        m_file.addSeparator()
        m_file.addAction(self.act_quit)

        m_program = bar.addMenu("&Programa")
        m_program.addAction(self.act_compile)
        m_program.addAction(self.act_run)
        m_program.addAction(self.act_stop)
        m_program.addSeparator()
        m_program.addAction(self.act_export_pad)
        m_program.addAction(self.act_allow_unverified)

        m_robot = bar.addMenu("&Robot")
        m_robot.addAction(self.act_digitize)

        m_sim = bar.addMenu("&Simulación")
        act = m_sim.addAction("Simular")
        act.setShortcut("F9")  # F6 es "encuadrar" en la vista 3D, como en Inventor
        act.triggered.connect(lambda: (self.tabs.setCurrentWidget(self.sim_tab), self.sim_view.simulate()))
        act = m_sim.addAction("Gráficas de movimiento…")
        act.setShortcut("Ctrl+G")
        act.triggered.connect(self.sim_view.show_charts)

        # El menú Ventana se arma con los toggles que ya trae cada dock.
        self.menu_window = bar.addMenu("&Ventana")
        for key in ("estructura", "campos", "propiedades", "conexion", "mensajes", "log"):
            self.menu_window.addAction(self._docks[key].toggleViewAction())

        m_help = bar.addMenu("A&yuda")
        m_help.addAction(self.act_about)

    def _build_toolbar(self) -> None:
        self.toolbar = self.addToolBar("Principal")
        self.toolbar.setObjectName("toolbar_principal")
        self.toolbar.addAction(self.act_open)
        self.toolbar.addAction(self.act_save)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.act_compile)
        self.toolbar.addAction(self.act_run)
        self.toolbar.addAction(self.act_stop)
        self.toolbar.addAction(self.act_export_pad)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.act_digitize)

    def _build_status_bar(self) -> None:
        self.status_connection = QLabel("Sin conectar")
        self.status_vm = QLabel("VM: detenida")
        self.status_points = QLabel("Puntos: 0")
        self.status_cursor = QLabel("Línea 1, Col 1")
        for label in (
            self.status_connection, self.status_vm,
            self.status_points, self.status_cursor,
        ):
            self.statusBar().addPermanentWidget(label)

    def _connect_signals(self) -> None:
        self.act_open.triggered.connect(self._on_open)
        self.act_save.triggered.connect(self._on_save)
        self.act_quit.triggered.connect(self.close)
        self.act_compile.triggered.connect(self._on_compile)
        self.act_run.triggered.connect(self._on_run)
        self.act_stop.triggered.connect(self._on_stop)
        self.act_export_pad.triggered.connect(self._on_export_pad)
        self.act_digitize.triggered.connect(self._on_digitize)
        self.act_about.triggered.connect(self._on_about)

        self.digitize_btn.clicked.connect(self._on_digitize)
        self.delete_digitized_btn.clicked.connect(self._on_delete_digitized)
        self.insert_points_btn.clicked.connect(self._on_insert_points)

        self.project_tree.line_requested.connect(self._goto_line)
        self.project_tree.currentItemChanged.connect(
            lambda current, _previous: self.properties_panel.show_item(current)
        )

        self.work_fields.field_changed.connect(self._apply_work_field)
        self.editor.cursorPositionChanged.connect(self._refresh_cursor_label)
        self.connection_panel.connect_btn.clicked.connect(self._refresh_connection_label)

    # -- campos de trabajo ------------------------------------------------------

    def _apply_work_field(self, key: str) -> None:
        field = field_by_key(key)
        for dock_key, dock in self._docks.items():
            dock.setVisible(dock_key in field.visible_docks)
        self.statusBar().showMessage(f"Campo de trabajo: {field.label}", 3000)

    # -- acciones de archivo ----------------------------------------------------

    def _on_open(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Abrir programa", filter="Borunte DSL (*.krlb *.txt);;Todos (*)"
        )
        if path:
            with open(path, encoding="utf-8") as f:
                self.editor.setPlainText(f.read())
            self._current_path = Path(path)
            self.messages.info(f"Abierto {path}", "archivo")

    def _on_save(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Guardar programa", filter="Borunte DSL (*.krlb);;Todos (*)"
        )
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.editor.toPlainText())
            self._current_path = Path(path)
            self.messages.info(f"Guardado {path}", "archivo")

    def _on_about(self) -> None:
        QMessageBox.about(
            self,
            "Acerca de Borunte DSL",
            "Compilador y entorno para programar un robot Borunte. El programa se "
            "exporta como respaldo del pad (HCBackupRobot_*.zip) y se importa por "
            "pendrive.\n\n"
            "El formato del respaldo está deducido, no confirmado por Borunte: ver "
            "docs/PAD_FORMAT.md. Probá cada programa nuevo a velocidad baja.",
        )

    # -- compilar ---------------------------------------------------------------

    def _on_compile(self) -> None:
        source = self.editor.toPlainText()
        try:
            program = compile_source(source)
        except CompileError as e:
            self._program = None
            self.messages.error(str(e), "compilador")
            self._show_messages_dock()
            self._refresh_action_states()
            return
        except Exception as e:  # noqa: BLE001 — errores de parseo de Lark, etc.
            self._program = None
            self.messages.error(f"Error de sintaxis: {e}", "compilador")
            self._show_messages_dock()
            self._refresh_action_states()
            return

        self._program = program
        self.bytecode_view.setPlainText(program.dump())
        self._populate_points_table()
        self.project_tree.rebuild(program, source)
        self.messages.info(
            f"Compilado: {len(program.instructions)} instrucciones, "
            f"{len(program.points)} puntos, {len(program.proc_addresses)} procedimientos.",
            "compilador",
        )
        self._warn_about_duplicate_points(program)
        self._refresh_action_states()
        self.tabs.setCurrentWidget(self.bytecode_view)

    def _warn_about_duplicate_points(self, program: Program) -> None:
        """La tabla de puntos no se deduplica: cada MOVEJ/MOVEL agrega una
        entrada aunque mueva a un POINT ya declarado. No cambia lo que se
        exporta al pad (ahí cada movimiento lleva su punto adentro), pero
        conviene saberlo al mirar la pestaña Puntos."""
        total = len(program.points)
        distintos = len({tuple(p.to_scaled_ints()) for p in program.points})
        if total > distintos:
            self.messages.warning(
                f"La tabla de puntos tiene {total} entradas para {distintos} "
                f"posiciones distintas (no se deduplica).",
                "compilador",
            )

    def _sim_report(self, severity: str, message: str) -> None:
        if not hasattr(self, "messages"):  # todavía construyendo la ventana
            self._pending_reports.append((severity, message))
            return
        # Los avisos de modelos rotos no son de una simulación: no se borran
        # cuando arranca la siguiente.
        source = "modelos" if message.startswith("Modelo de robot") else "simulador"
        {"info": self.messages.info, "warning": self.messages.warning,
         "error": self.messages.error}[severity](message, source)
        if severity == "error":
            self._show_messages_dock()

    # -- exportar al pad ----------------------------------------------------------

    def pad_program_name(self) -> str:
        """Nombre del programa en el pad: el del archivo, sin lo que el pad no
        acepte. Sin archivo, uno fijo."""
        if self._current_path is None:
            return "BorunteDSL"
        name = re.sub(r"[^A-Za-z0-9_.\-]", "_", self._current_path.stem)
        return name or "BorunteDSL"

    def export_to_pad(self, directory: str | Path) -> Path | None:
        """Compila para el pad y escribe el respaldo en `directory`. Devuelve la
        ruta, o None si no compiló (el error queda en la ventana de mensajes)."""
        source = self.editor.toPlainText()
        options = PadOptions(program_name=self.pad_program_name(),
                             allow_unverified=self.act_allow_unverified.isChecked())
        try:
            backup, warnings = compile_to_pad_report(source, options)
        except CompileError as e:
            self.messages.error(f"No se puede exportar al pad: {e}", "pad")
            self._show_messages_dock()
            return None
        except Exception as e:  # noqa: BLE001 — errores de parseo de Lark, etc.
            self.messages.error(f"No se puede exportar al pad: {e}", "pad")
            self._show_messages_dock()
            return None

        try:
            path = backup.write(directory)
        except OSError as e:
            self.messages.error(f"No se pudo escribir el respaldo en {directory}: {e}", "pad")
            self._show_messages_dock()
            return None
        for warning in warnings:
            self.messages.warning(warning, "pad")
        if options.allow_unverified:
            self.messages.warning("Exportado CON instrucciones sin confirmar habilitadas: "
                                  "probalo primero a velocidad baja.", "pad")
        self.pad_view.setPlainText("\n".join(list_backup(backup)))
        self.tabs.setCurrentWidget(self.pad_view)
        self.messages.info(
            f"Exportado {path}. Copialo a la raíz de un pendrive e importalo desde el "
            f"pad. El formato no está confirmado: probalo a velocidad baja.",
            "pad",
        )
        return path

    def _on_export_pad(self) -> None:
        directory = QFileDialog.getExistingDirectory(self, "Carpeta donde guardar el respaldo")
        if directory:
            self.export_to_pad(directory)

    def _populate_points_table(self) -> None:
        assert self._program is not None
        self.points_table.setRowCount(len(self._program.points))
        for i, pose in enumerate(self._program.points):
            values = [str(i)] + [f"{v:g}" for v in (pose.a, pose.b, pose.c, pose.d, pose.e, pose.f)]
            for col, value in enumerate(values):
                self.points_table.setItem(i, col, QTableWidgetItem(value))
        self.status_points.setText(f"Puntos: {len(self._program.points)}")

    # -- ejecutar / parar --------------------------------------------------------

    def _on_run(self) -> None:
        if self._program is None:
            return
        if not self.connection_panel.is_connected():
            self.messages.warning(
                "Conectate al simulador o al robot real antes de ejecutar.", "vm"
            )
            self._show_messages_dock()
            return

        self.log_view.clear()
        self._docks["log"].setVisible(True)

        robot = self.connection_panel.robot
        assert robot is not None

        self._thread = QThread()
        self._worker = VmWorker(self._program, robot, self._plc_io)
        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.run)
        self._worker.log_line.connect(self._append_log)
        self._worker.finished.connect(self._on_run_finished)
        self._worker.finished.connect(self._thread.quit)

        self.status_vm.setText("VM: corriendo")
        self.messages.info("Ejecución iniciada.", "vm")
        self._thread.start()
        self._refresh_action_states()

    def _on_stop(self) -> None:
        if self._worker is None:
            return
        self.messages.warning(
            "Parada pedida. No interrumpe el movimiento en curso: la VM corta "
            "cuando termina. Un paro de emergencia va por línea física.",
            "vm",
        )
        self._worker.stop()

    def _append_log(self, msg: str) -> None:
        self.log_view.appendPlainText(msg)

    def _on_run_finished(self, success: bool, message: str) -> None:
        self._append_log(f"\n{'✔' if success else '✘'} {message}")
        if success:
            self.messages.info(message, "vm")
        else:
            self.messages.error(message, "vm")

        # Esperar a que el hilo termine ANTES de soltar la referencia. Si se
        # sueltan acá nomás, Python puede recolectar el QThread mientras el
        # hilo todavía está saliendo y Qt destruye el objeto C++ por debajo:
        # eso no es una excepción, es un crash del proceso.
        if self._thread is not None:
            self._thread.quit()
            self._thread.wait(5000)
        self._worker = None
        self._thread = None

        self.status_vm.setText("VM: detenida")
        self._refresh_action_states()

    def closeEvent(self, event) -> None:  # noqa: N802 — nombre de Qt
        # Una simulación en curso procesa eventos: sin esto seguía corriendo
        # después de cerrar la ventana.
        self.sim_view.cancel()
        super().closeEvent(event)

    def is_running(self) -> bool:
        return self._worker is not None

    # -- digitalización de puntos -------------------------------------------------

    def _on_digitize(self) -> None:
        robot = self.connection_panel.robot
        if robot is None:
            self.messages.warning("Conectate al robot antes de digitalizar.", "digitalizar")
            self._show_messages_dock()
            return
        if self.is_running():
            # No debería llegar acá (la acción está deshabilitada), pero si
            # llegara: el worker está usando el mismo cliente Modbus y leer la
            # posición desde el hilo de la UI intercalaría transacciones sobre
            # el mismo socket.
            self.messages.warning(
                "No se puede digitalizar mientras la VM está corriendo.", "digitalizar"
            )
            return

        try:
            pose = robot.read_world_position()
        except Exception as e:  # noqa: BLE001
            self.messages.error(f"No se pudo leer la posición: {e}", "digitalizar")
            self._show_messages_dock()
            return

        name = f"p_digit_{len(self._digitized) + 1}"
        self._digitized.append((name, pose))
        self._refresh_digitized_table()
        self.messages.info(
            f"Digitalizado {name} en ({pose.a:g}, {pose.b:g}, {pose.c:g}).", "digitalizar"
        )
        self.tabs.setCurrentIndex(2)  # tab Puntos

    def _refresh_digitized_table(self) -> None:
        table = self.digitized_table
        table.blockSignals(True)
        table.setRowCount(len(self._digitized))
        for row, (name, pose) in enumerate(self._digitized):
            table.setItem(row, 0, QTableWidgetItem(name))
            for col, value in enumerate(
                (pose.a, pose.b, pose.c, pose.d, pose.e, pose.f), start=1
            ):
                item = QTableWidgetItem(f"{value:g}")
                # Las coordenadas no se editan a mano: vienen del robot. Solo
                # el nombre es del usuario.
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                table.setItem(row, col, item)
        table.blockSignals(False)

    def _sync_digitized_names(self) -> None:
        """Lee de vuelta los nombres que el usuario editó en la tabla."""
        for row in range(min(self.digitized_table.rowCount(), len(self._digitized))):
            item = self.digitized_table.item(row, 0)
            if item and item.text().strip():
                _, pose = self._digitized[row]
                self._digitized[row] = (item.text().strip(), pose)

    def _on_delete_digitized(self) -> None:
        row = self.digitized_table.currentRow()
        if not (0 <= row < len(self._digitized)):
            self.messages.warning("Elegí una fila para borrar.", "digitalizar")
            return
        name, _ = self._digitized.pop(row)
        self._refresh_digitized_table()
        self.messages.info(f"Borrado el punto digitalizado {name}.", "digitalizar")

    def _on_insert_points(self) -> None:
        """Genera las declaraciones POINT y las inserta en el editor.

        Sin esto la tabla no tenía salida hacia el programa: se podían capturar
        puntos y quedaban ahí, mirándote.
        """
        self._sync_digitized_names()
        if not self._digitized:
            self.messages.warning("No hay puntos digitalizados para insertar.", "digitalizar")
            return

        lines = [self.point_declaration(name, pose) for name, pose in self._digitized]
        block = "\n".join(lines) + "\n"
        cursor = self.editor.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.StartOfLine)
        cursor.insertText(block)
        self.messages.info(
            f"Insertadas {len(lines)} declaraciones POINT en el editor.", "digitalizar"
        )
        self.tabs.setCurrentWidget(self.editor)

    @staticmethod
    def point_declaration(name: str, pose: Pose) -> str:
        coords = ", ".join(
            f"{v:.3f}" for v in (pose.a, pose.b, pose.c, pose.d, pose.e, pose.f)
        )
        return f"POINT {name} = WORLD({coords})"

    # -- navegación y estado ------------------------------------------------------

    def _goto_line(self, line: int) -> None:
        block = self.editor.document().findBlockByLineNumber(line - 1)
        if not block.isValid():
            return
        cursor = QTextCursor(block)
        self.editor.setTextCursor(cursor)
        self.tabs.setCurrentWidget(self.editor)
        self.editor.setFocus()

    def _refresh_cursor_label(self) -> None:
        cursor = self.editor.textCursor()
        self.status_cursor.setText(
            f"Línea {cursor.blockNumber() + 1}, Col {cursor.positionInBlock() + 1}"
        )

    def _refresh_connection_label(self) -> None:
        if self.connection_panel.is_connected():
            self.status_connection.setText("Conectado")
            self.messages.info("Conectado.", "conexión")
        else:
            self.status_connection.setText("Sin conectar")
        self._refresh_action_states()

    def _refresh_action_states(self) -> None:
        running = self.is_running()
        self.act_run.setEnabled(self._program is not None and not running)
        self.act_stop.setEnabled(running)
        # Digitalizar necesita robot conectado Y la VM parada: el worker está
        # usando el mismo cliente Modbus.
        can_digitize = self.connection_panel.is_connected() and not running
        self.act_digitize.setEnabled(can_digitize)
        self.digitize_btn.setEnabled(can_digitize)

    def _show_messages_dock(self) -> None:
        self._docks["mensajes"].setVisible(True)
        self._docks["mensajes"].raise_()
