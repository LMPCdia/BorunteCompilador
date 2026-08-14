"""Ventana principal: editor del DSL, compilar, ejecutar, ver puntos y log."""

from __future__ import annotations

from PySide6.QtCore import Qt, QThread
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from compiler.codegen import CompileError, compile_source
from gui.connection_panel import ConnectionPanel
from gui.vm_worker import VmWorker
from runtime.bytecode import Program
from runtime.plc_io_simulator import PlcIoSimulator

EXAMPLE_PROGRAM = """POINT p_home = WORLD(0.0, 500.0, 300.0, 0.0, 0.0, 0.0)
POINT p_pieza = WORLD(100.0, 600.0, 200.0, 0.0, 0.0, 0.0)

VAR pieza : INT = 1

PROC soldar_pieza()
MOVEJ p_home SPEED 80
MOVEL p_pieza SPEED 50
IF pieza == 1 THEN
SET_OUT(Y10, ON)
ELSE
SET_OUT(Y11, ON)
ENDIF
WAIT 0.2s
SET_OUT(Y10, OFF)
MOVEJ p_home SPEED 80
ENDPROC

soldar_pieza()
"""


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Borunte DSL — Editor / Compilador / Ejecución")
        self.resize(1100, 700)

        self._program: Program | None = None
        self._plc_io = PlcIoSimulator()
        self._thread: QThread | None = None
        self._worker: VmWorker | None = None

        self._build_ui()

    # -- construcción de la UI -----------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)

        self.connection_panel = ConnectionPanel()
        root_layout.addWidget(self.connection_panel)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        root_layout.addWidget(splitter, stretch=1)

        # -- editor (izquierda) --
        editor_container = QWidget()
        editor_layout = QVBoxLayout(editor_container)
        editor_layout.setContentsMargins(0, 0, 0, 0)

        self.editor = QPlainTextEdit()
        self.editor.setFont(QFont("Consolas, Menlo, monospace", 11))
        self.editor.setPlainText(EXAMPLE_PROGRAM)
        editor_layout.addWidget(self.editor)

        buttons_row = QHBoxLayout()
        self.open_btn = QPushButton("Abrir…")
        self.save_btn = QPushButton("Guardar…")
        self.compile_btn = QPushButton("Compilar")
        self.run_btn = QPushButton("Ejecutar")
        self.run_btn.setEnabled(False)
        buttons_row.addWidget(self.open_btn)
        buttons_row.addWidget(self.save_btn)
        buttons_row.addStretch()
        buttons_row.addWidget(self.compile_btn)
        buttons_row.addWidget(self.run_btn)
        editor_layout.addLayout(buttons_row)

        splitter.addWidget(editor_container)

        # -- tabs (derecha): bytecode / puntos / log --
        self.tabs = QTabWidget()
        splitter.addWidget(self.tabs)
        splitter.setSizes([550, 550])

        self.bytecode_view = QPlainTextEdit()
        self.bytecode_view.setReadOnly(True)
        self.bytecode_view.setFont(QFont("Consolas, Menlo, monospace", 10))
        self.tabs.addTab(self.bytecode_view, "Bytecode")

        self.points_table = QTableWidget(0, 7)
        self.points_table.setHorizontalHeaderLabels(["#", "A", "B", "C", "D", "E", "F"])
        self.tabs.addTab(self.points_table, "Puntos")

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Consolas, Menlo, monospace", 10))
        self.tabs.addTab(self.log_view, "Log de ejecución")

        # -- conexiones de señales --
        self.open_btn.clicked.connect(self._on_open)
        self.save_btn.clicked.connect(self._on_save)
        self.compile_btn.clicked.connect(self._on_compile)
        self.run_btn.clicked.connect(self._on_run)

    # -- acciones -----------------------------------------------------------

    def _on_open(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Abrir programa", filter="Borunte DSL (*.krlb *.txt);;Todos (*)")
        if path:
            with open(path, encoding="utf-8") as f:
                self.editor.setPlainText(f.read())

    def _on_save(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Guardar programa", filter="Borunte DSL (*.krlb);;Todos (*)")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.editor.toPlainText())

    def _on_compile(self) -> None:
        source = self.editor.toPlainText()
        try:
            self._program = compile_source(source)
        except CompileError as e:
            self._program = None
            self.run_btn.setEnabled(False)
            self.bytecode_view.setPlainText(f"Error de compilación:\n\n{e}")
            self.tabs.setCurrentWidget(self.bytecode_view)
            return
        except Exception as e:  # noqa: BLE001 — errores de parseo de Lark, etc.
            self._program = None
            self.run_btn.setEnabled(False)
            self.bytecode_view.setPlainText(f"Error de sintaxis:\n\n{e}")
            self.tabs.setCurrentWidget(self.bytecode_view)
            return

        self.bytecode_view.setPlainText(self._program.dump())
        self._populate_points_table()
        self.run_btn.setEnabled(True)
        self.tabs.setCurrentWidget(self.bytecode_view)

    def _populate_points_table(self) -> None:
        assert self._program is not None
        self.points_table.setRowCount(len(self._program.points))
        for i, pose in enumerate(self._program.points):
            values = [i, pose.a, pose.b, pose.c, pose.d, pose.e, pose.f]
            for col, v in enumerate(values):
                self.points_table.setItem(i, col, QTableWidgetItem(str(v)))

    def _on_run(self) -> None:
        if self._program is None:
            return
        if not self.connection_panel.is_connected():
            QMessageBox.warning(self, "Sin conexión", "Conectate al simulador o al robot real antes de ejecutar.")
            return

        self.log_view.clear()
        self.tabs.setCurrentWidget(self.log_view)
        self.run_btn.setEnabled(False)

        robot = self.connection_panel.robot
        assert robot is not None

        self._thread = QThread()
        self._worker = VmWorker(self._program, robot, self._plc_io)
        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.run)
        self._worker.log_line.connect(self._append_log)
        self._worker.finished.connect(self._on_run_finished)
        self._worker.finished.connect(self._thread.quit)

        self._thread.start()

    def _append_log(self, msg: str) -> None:
        self.log_view.appendPlainText(msg)

    def _on_run_finished(self, success: bool, message: str) -> None:
        self._append_log(f"\n{'✔' if success else '✘'} {message}")
        self.run_btn.setEnabled(True)
