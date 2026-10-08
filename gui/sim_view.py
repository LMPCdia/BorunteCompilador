"""
Pestaña "Simulación 3D": simula sobre el modelo de robot elegido (`sim/`)
el programa del editor o un respaldo del pad (`HCBackupRobot_*.zip`), lo
anima, y arma la celda: piezas STEP/STL/OBJ, herramientas y sistemas de
coordenadas del pad. Todo eso se guarda junto en un `.layout.json`.

Todo menos la vista 3D funciona sin placa de video (y así se prueba). La vista
se crea solo si hay OpenGL: ver el aviso en gui/viewport3d.py.

Reglas que salieron de probarla como usuario:
- Ninguna acción del usuario puede tirar una excepción sin mostrarla: en el
  .exe sin consola eso es "no hace nada".
- La simulación nunca cuelga la interfaz: tiene presupuesto, detecta bucles y
  se puede cancelar.
- Si cambia algo que la simulación usó (modelo, herramientas, coordenadas,
  programa), el resultado se marca desactualizado.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Callable

from PySide6.QtCore import QSettings, Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QGuiApplication, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from compiler.codegen import CompileError
from compiler.pad_codegen import MAX_TOOL_OR_COORD, PadOptions, compile_to_pad_report, io_point
from pad.backup import PadBackup
from pad.listing import io_name
from sim.kinematics import RobotModel, identity, pose_matrix, rot_axis
from sim.meshes import Mesh, MeshError, load_mesh
from sim.pad_sim import Cancelled, PadSimulator, SimResult, inputs_used
from sim.scene import Layout, LayoutObject, Timeline, robot_link_meshes

SLIDER_STEPS = 1000
FRAME_MS = 33
OBJECT_COLUMNS = ["Pieza", "X", "Y", "Z", "Giro Z°"]
POSE_COLUMNS = ["N°", "X", "Y", "Z", "U", "V", "W"]
DEFAULT_MODEL = "BRTIRUS1820A"
IMPORT_AT = (1200.0, 0.0)      # dónde aparece una pieza importada (frente al robot)
MAX_REPORTED_ISSUES = 12       # a la ventana de mensajes; el resto, en "Problemas"
COLOR_MOVEJ = (0.35, 0.75, 1.0)
COLOR_MOVEL = (1.0, 0.85, 0.2)
COLOR_OUTPUT_ON = (1.0, 0.45, 0.1)   # MOVEL con alguna salida prendida (p. ej. soldando)
COLOR_FAILED = (1.0, 0.15, 0.15)
INVALID_BRUSH = QBrush(QColor(220, 60, 60, 110))

Reporter = Callable[[str, str], None]  # (severidad "info"|"warning"|"error", mensaje)


def object_matrix(obj: LayoutObject):
    r = rot_axis((0, 0, 1), math.radians(obj.rz))
    return [r[0] + [obj.x], r[1] + [obj.y], r[2] + [obj.z], [0.0, 0.0, 0.0, 1.0]]


def parse_inputs(text: str) -> dict[int, bool]:
    """'X012=1, X013=0' -> {2: True, 3: False}."""
    inputs = {}
    for item in text.replace(";", ",").split(","):
        if not item.strip():
            continue
        name, _, value = item.partition("=")
        inputs[io_point(name.strip(), "X")] = value.strip().upper() not in ("0", "OFF", "")
    return inputs


def _number(text: str) -> float:
    value = float(text.strip().replace(",", "."))
    if not math.isfinite(value):
        raise ValueError(text)
    return value


def _fmt(value: float) -> str:
    return f"{(value or 0.0):g}"  # sin "-0"


class PoseTable(QWidget):
    """Tabla editable "número -> X, Y, Z, U, V, W" (herramientas o sistemas
    de coordenadas del pad). Es la única fuente de verdad: lo que no es válido
    se pinta de rojo y no se usa."""

    def __init__(self, title: str, help_text: str, on_change: Callable[[], None]) -> None:
        super().__init__()
        self._on_change = on_change
        self._loading = False
        self.table = QTableWidget(0, len(POSE_COLUMNS))
        self.table.setHorizontalHeaderLabels(POSE_COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.itemChanged.connect(self._edited)
        add_btn = QPushButton("Agregar")
        add_btn.clicked.connect(lambda: self.add_row())
        remove_btn = QPushButton("Quitar")
        remove_btn.clicked.connect(self._remove)
        note = QLabel(help_text)
        note.setWordWrap(True)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(title))
        layout.addWidget(self.table, 1)
        row = QHBoxLayout()
        row.addWidget(add_btn)
        row.addWidget(remove_btn)
        layout.addLayout(row)
        layout.addWidget(note)

    def _row_values(self, r: int) -> tuple[int, list[float]] | None:
        try:
            number = _number(self.table.item(r, 0).text())
            if number != int(number) or not 0 <= number <= MAX_TOOL_OR_COORD:
                return None
            return int(number), [_number(self.table.item(r, c).text()) for c in range(1, 7)]
        except (ValueError, AttributeError):
            return None

    def values(self) -> dict[int, list[float]]:
        """Solo filas válidas; con números repetidos vale la primera."""
        out: dict[int, list[float]] = {}
        for r in range(self.table.rowCount()):
            row = self._row_values(r)
            if row is not None and row[0] not in out:
                out[row[0]] = row[1]
        return out

    def invalid_rows(self) -> list[int]:
        seen, bad = set(), []
        for r in range(self.table.rowCount()):
            row = self._row_values(r)
            if row is None or row[0] in seen:
                bad.append(r)
            else:
                seen.add(row[0])
        return bad

    def set_values(self, values: dict[int, list[float]]) -> None:
        self._loading = True
        self.table.setRowCount(0)
        for number, pose in sorted(values.items()):
            self._append([str(number)] + [_fmt(v) for v in pose])
        self._loading = False
        self._paint()

    def add_row(self, number: int | None = None, blank: bool = False) -> None:
        if number is None:
            number = max(self.values(), default=0) + 1
        self._loading = True
        self._append([str(number)] + ([""] * 6 if blank else ["0"] * 6))
        self._loading = False
        self._paint()
        self._on_change()

    def _append(self, row_values: list[str]) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        for col, value in enumerate(row_values):
            self.table.setItem(row, col, QTableWidgetItem(value))

    def _remove(self) -> None:
        row = self.table.currentRow()
        if row >= 0:
            self.table.removeRow(row)
            self.table.clearSelection()
            self._paint()
            self._on_change()

    def _paint(self) -> None:
        self._loading = True
        bad = set(self.invalid_rows())
        for r in range(self.table.rowCount()):
            for c in range(self.table.columnCount()):
                item = self.table.item(r, c)
                if item is not None:
                    item.setBackground(INVALID_BRUSH if r in bad else QBrush())
        self._loading = False

    def _edited(self, _item: QTableWidgetItem) -> None:
        if self._loading:
            return
        self._paint()
        self._on_change()


class SimView(QWidget):
    def __init__(self, get_source: Callable[[], str], report: Reporter | None = None,
                 enable_3d: bool | None = None,
                 clear_reports: Callable[[], None] | None = None) -> None:
        super().__init__()
        self._get_source = get_source
        self._report = report or (lambda _severity, _msg: None)
        self._clear_reports = clear_reports or (lambda: None)
        self.layout_data = Layout()
        self.layout_path: Path | None = None
        self.backup: PadBackup | None = None      # respaldo abierto (None = editor)
        self.backup_path: Path | None = None
        self.result: SimResult | None = None
        self.timeline: Timeline | None = None
        self.stale = False
        self.current_q: list[float] = [0.0] * 6
        self.model: RobotModel | None = None
        self._input_boxes: dict[int, QCheckBox] = {}
        self._mesh_cache: dict[str, Mesh] = {}
        self._t = 0.0
        self._updating_table = False
        self._running = False
        self._cancel = False
        self._object_index: dict[int, int] = {}  # fila de la tabla -> objeto en la vista
        self._simulated_source: str | None = None
        # Archivo .ini (no el registro de Windows): los tests lo redirigen a una
        # carpeta temporal con QSettings.setPath (tests/conftest.py).
        self._settings = QSettings(QSettings.Format.IniFormat, QSettings.Scope.UserScope,
                                   "BorunteDSL", "BorunteDSL")

        if enable_3d is None:
            from gui.viewport3d import opengl_available

            enable_3d = opengl_available()
        self.viewport = None
        if enable_3d:
            from gui.viewport3d import Viewport3D

            self.viewport = Viewport3D()

        self._build()
        self._timer = QTimer(self)
        self._timer.setInterval(FRAME_MS)
        self._timer.timeout.connect(self._tick)
        # Barra espaciadora = play/pausa solo con la vista 3D enfocada: en el
        # resto de la pestaña tiene que seguir tildando casillas y apretando botones.
        if self.viewport is not None:
            shortcut = QShortcut(QKeySequence(Qt.Key.Key_Space), self.viewport.widget)
            shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
            shortcut.activated.connect(lambda: self.play_btn.toggle())

        for problem in RobotModel.problems():
            self._report("warning", problem)
        self._select_initial_model()
        self._update_axes()

    # -- construcción ----------------------------------------------------------------

    def _build(self) -> None:
        self.model_combo = QComboBox()
        self.model_combo.addItems(RobotModel.available())
        self.model_combo.currentTextChanged.connect(self.set_model)
        self.source_label = QLabel("Programa: el del editor")
        self.open_backup_btn = open_backup_btn = QPushButton("Abrir respaldo del pad…")
        open_backup_btn.clicked.connect(self._on_open_backup)
        self.use_editor_btn = QPushButton("Usar el editor")
        self.use_editor_btn.clicked.connect(self.use_editor)
        self.use_editor_btn.setEnabled(False)
        self.simulate_btn = QPushButton("Simular")
        self.simulate_btn.clicked.connect(self.simulate)
        self.cancel_btn = QPushButton("Cancelar")
        self.cancel_btn.clicked.connect(self._on_cancel)
        self.cancel_btn.setVisible(False)

        top = QHBoxLayout()
        top.addWidget(QLabel("Robot:"))
        top.addWidget(self.model_combo)
        top.addWidget(self.source_label, 1)
        top.addWidget(open_backup_btn)
        top.addWidget(self.use_editor_btn)
        top.addWidget(self.simulate_btn)
        top.addWidget(self.cancel_btn)

        # Entradas: una casilla por cada entrada que consulta el programa, en
        # una fila con scroll (un programa con 30 entradas no puede romper el
        # ancho de la ventana).
        inputs_widget = QWidget()
        self.inputs_row = QHBoxLayout(inputs_widget)
        self.inputs_row.setContentsMargins(0, 0, 0, 0)
        self.inputs_hint = QLabel("Entradas: (el programa no consulta ninguna)")
        self.inputs_row.addWidget(self.inputs_hint)
        self.inputs_row.addStretch(1)
        self.inputs_scroll = QScrollArea()
        self.inputs_scroll.setWidget(inputs_widget)
        self.inputs_scroll.setWidgetResizable(True)
        self.inputs_scroll.setFixedHeight(40)
        self.inputs_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.play_btn = QPushButton("▶")
        self.play_btn.setCheckable(True)
        self.play_btn.setToolTip("Reproducir / pausar (barra espaciadora)")
        self.play_btn.toggled.connect(self._on_play_toggled)
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["x0.5", "x1", "x2", "x5", "x10"])
        self.speed_combo.setCurrentText("x1")
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.slider.valueChanged.connect(self._on_slider)
        self.time_label = QLabel("—")
        bottom = QHBoxLayout()
        bottom.addWidget(self.play_btn)
        bottom.addWidget(self.speed_combo)
        bottom.addWidget(self.slider, 1)
        bottom.addWidget(self.time_label)

        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)
        if self.viewport is not None:
            cams = QHBoxLayout()
            mouse = "Mouse: botón izquierdo gira, derecho desplaza, rueda acerca."
            for label, view in (("Encuadrar", "fit"), ("Iso", "iso"), ("Arriba", "arriba"),
                                ("Frente", "frente"), ("Lado", "lado")):
                btn = QPushButton(label)
                btn.setToolTip(mouse)
                btn.clicked.connect(lambda _=False, v=view: self.set_camera(v))
                cams.addWidget(btn)
            cams.addStretch(1)
            center_layout.addLayout(cams)
            self.viewport.widget.setToolTip(mouse)
            center_layout.addWidget(self.viewport.widget, 1)
        else:
            view = QLabel("La vista 3D necesita OpenGL y no está disponible en esta PC.\n"
                          "La simulación, los avisos y el layout funcionan igual.")
            view.setAlignment(Qt.AlignmentFlag.AlignCenter)
            center_layout.addWidget(view, 1)
        center_layout.addLayout(bottom)
        self.summary = QLabel("Apretá «Simular» para simular el programa del editor.")
        self.summary.setWordWrap(True)
        self.summary.setTextFormat(Qt.TextFormat.RichText)
        self.load_missing_btn = QPushButton("Cargar las que faltan")
        self.load_missing_btn.clicked.connect(self.add_missing_frames)
        self.load_missing_btn.setVisible(False)
        summary_row = QHBoxLayout()
        summary_row.addWidget(self.summary, 1)
        summary_row.addWidget(self.load_missing_btn)
        center_layout.addLayout(summary_row)

        splitter = QSplitter()
        splitter.addWidget(center)
        splitter.addWidget(self._build_side())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1000, 460])

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(self.inputs_scroll)
        layout.addWidget(splitter, 1)

    def _build_side(self) -> QWidget:
        # -- problemas --
        self.issues_list = QListWidget()
        self.issues_list.setWordWrap(True)
        self.issues_list.itemActivated.connect(self._on_issue_activated)
        self.issues_list.itemClicked.connect(self._on_issue_activated)
        issues = QWidget()
        issues_layout = QVBoxLayout(issues)
        issues_layout.addWidget(QLabel("Clic en un problema para ir a ese momento."))
        issues_layout.addWidget(self.issues_list, 1)

        # -- piezas --
        self.objects_table = QTableWidget(0, len(OBJECT_COLUMNS))
        self.objects_table.setHorizontalHeaderLabels(OBJECT_COLUMNS)
        header = self.objects_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for col in range(1, len(OBJECT_COLUMNS)):
            header.setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)
        self.objects_table.verticalHeader().setVisible(False)
        self.objects_table.itemChanged.connect(self._on_object_edited)
        import_btn = QPushButton("Importar pieza…")
        import_btn.clicked.connect(self._on_import)
        remove_btn = QPushButton("Quitar")
        remove_btn.clicked.connect(self._on_remove)
        objects = QWidget()
        objects_layout = QVBoxLayout(objects)
        objects_note = QLabel("Piezas STEP/STL/OBJ. X, Y, Z: dónde queda el origen del CAD, en mm "
                              "respecto de la base del robot.")
        objects_note.setWordWrap(True)
        objects_layout.addWidget(objects_note)
        objects_layout.addWidget(self.objects_table, 1)
        row = QHBoxLayout()
        row.addWidget(import_btn)
        row.addWidget(remove_btn)
        objects_layout.addLayout(row)

        # -- herramientas y coordenadas --
        self.tools_table = PoseTable(
            "Herramientas del pad (punta respecto de la brida)",
            "Copiá los valores del menú de herramientas del pad. La 0 es la brida y no "
            "hace falta cargarla. Las filas en rojo están incompletas o repetidas y no se usan.",
            self._on_frames_changed)
        self.frames_table = PoseTable(
            "Sistemas de coordenadas del pad (respecto de la base)",
            "Copiá los valores del menú de coordenadas del pad. El 0 es la base del robot "
            "y no hace falta cargarlo. Se dibujan como ejes rojo/verde/azul.",
            self._on_frames_changed)

        tabs = QTabWidget()
        tabs.addTab(issues, "Problemas")
        tabs.addTab(objects, "Piezas")
        tabs.addTab(self.tools_table, "Herramientas")
        tabs.addTab(self.frames_table, "Coordenadas")
        self.side_tabs = tabs

        self.open_cell_btn = open_btn = QPushButton("Abrir celda…")
        open_btn.clicked.connect(self._on_open_layout)
        self.save_cell_btn = save_btn = QPushButton("Guardar celda…")
        save_btn.clicked.connect(self._on_save_layout)
        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.addWidget(tabs, 1)
        row2 = QHBoxLayout()
        row2.addWidget(open_btn)
        row2.addWidget(save_btn)
        side_layout.addLayout(row2)
        return side

    # -- modelo -----------------------------------------------------------------------

    def _select_initial_model(self) -> None:
        names = RobotModel.available()
        if not names:
            self._report("error", "No hay ningún modelo de robot válido instalado.")
            return
        last = self._settings.value("sim/model", "")
        name = last if last in names else (DEFAULT_MODEL if DEFAULT_MODEL in names else names[0])
        self.model_combo.blockSignals(True)
        self.model_combo.setCurrentText(name)
        self.model_combo.blockSignals(False)
        self.set_model(name)

    def set_model(self, name: str) -> bool:
        try:
            model = RobotModel.load(name)
            meshes = robot_link_meshes(model) if self.viewport is not None else None
        except Exception as e:  # noqa: BLE001 — JSON roto, mallas que faltan, etc.
            self._report("error", f"No se pudo cargar el robot {name}: {e}")
            if self.model is not None:
                self.model_combo.blockSignals(True)
                self.model_combo.setCurrentText(self.model.name)
                self.model_combo.blockSignals(False)
            return False
        previous = self.model
        self.model = model
        self.layout_data.model = name
        self._settings.setValue("sim/model", name)
        if meshes is not None:
            self.viewport.set_robot(meshes)
        self.show_pose(self.current_q)
        if previous is not None and previous.name != name:
            self._mark_stale(f"se cambió el robot a {name}")
        return True

    # -- qué se simula ------------------------------------------------------------------

    def open_backup(self, path: str | Path) -> bool:
        try:
            self.backup = PadBackup.read(path)
        except Exception as e:  # noqa: BLE001 — zip roto, otro formato, etc.
            self._report("error", f"No se pudo abrir el respaldo: {e}")
            return False
        self.backup_path = Path(path)
        self.source_label.setText(f"Programa: {self.backup.name} (respaldo del pad)")
        self.use_editor_btn.setEnabled(True)
        self._refresh_inputs(self.backup)
        self._clear_result()
        self._report("info", f"Abierto el respaldo {Path(path).name}: programa {self.backup.name}, "
                             f"{len(self.backup.act.modules)} módulo(s). Elegí las entradas y "
                             f"apretá Simular.")
        return True

    def use_editor(self) -> None:
        self.backup = None
        self.backup_path = None
        try:
            self._refresh_inputs(compile_to_pad_report(self._get_source(),
                                                       PadOptions(allow_unverified=True))[0])
        except Exception:  # noqa: BLE001 — si el editor no compila, se ve al simular
            pass
        self.source_label.setText("Programa: el del editor")
        self.use_editor_btn.setEnabled(False)
        self._clear_result()

    def _current_backup(self) -> PadBackup:
        if self.backup is not None:
            return self.backup
        # En la simulación se permite lo "sin confirmar": acá no hay riesgo.
        backup, warnings = compile_to_pad_report(self._get_source(),
                                                 PadOptions(allow_unverified=True))
        for warning in warnings:
            self._report("info", f"[compilador] {warning}")
        self._refresh_inputs(backup)
        return backup

    # -- entradas ---------------------------------------------------------------------

    def _refresh_inputs(self, backup: PadBackup) -> None:
        used = inputs_used(backup)
        if used == sorted(self._input_boxes):
            return
        previous = self.inputs()
        for box in self._input_boxes.values():
            self.inputs_row.removeWidget(box)
            box.deleteLater()
        self._input_boxes = {}
        self.inputs_hint.setText("Entradas activas:" if used else
                                 "Entradas: (el programa no consulta ninguna)")
        for i, point in enumerate(used):
            box = QCheckBox(io_name("X", point))
            box.setChecked(previous.get(point, False))
            box.toggled.connect(self._on_input_toggled)
            self.inputs_row.insertWidget(1 + i, box)
            self._input_boxes[point] = box

    def inputs(self) -> dict[int, bool]:
        return {point: box.isChecked() for point, box in self._input_boxes.items()}

    def set_input(self, name: str, state: bool) -> None:
        self._input_boxes[io_point(name, "X")].setChecked(state)

    def _on_input_toggled(self, _checked: bool) -> None:
        if self.result is not None and not self._running:
            self.simulate()

    # -- simulación --------------------------------------------------------------------

    def _clear_result(self) -> None:
        self.play_btn.setChecked(False)
        self.result = None
        self.timeline = None
        self.stale = False
        self.issues_list.clear()
        self.load_missing_btn.setVisible(False)
        self.time_label.setText("—")
        self.summary.setText("Apretá «Simular».")
        if self.viewport is not None:
            self.viewport.set_path([], [])

    def _mark_stale(self, reason: str) -> None:
        if self.result is None or self._running:
            return
        self.stale = True
        self.play_btn.setChecked(False)
        self.summary.setText(f"<b style='color:#d08000'>⚠ Desactualizado: {reason}. Volvé a "
                             f"simular.</b>")

    def simulate(self) -> SimResult | None:
        if self._running:
            return None
        self.play_btn.setChecked(False)
        self._clear_reports()
        try:
            backup = self._current_backup()
        except CompileError as e:
            self._clear_result()
            self._report("error", f"No se puede simular: {e}")
            self.summary.setText(f"<b style='color:#d04040'>No compila:</b> {e}")
            return None
        except Exception as e:  # noqa: BLE001
            self._clear_result()
            self._report("error", f"No se puede simular: {e}")
            self.summary.setText(f"<b style='color:#d04040'>No compila:</b> {e}")
            return None
        if self.model is None:
            self._report("error", "No hay un modelo de robot cargado.")
            return None

        self._simulated_source = None if self.backup is not None else self._get_source()
        self._running, self._cancel = True, False
        self._set_controls_enabled(False)
        self.cancel_btn.setVisible(True)
        QGuiApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            simulator = PadSimulator(self.model, self.inputs(), tools=self.layout_data.tools,
                                     frames=self.layout_data.frames, progress=self._progress)
            result = simulator.run(backup)
        except Cancelled:
            self._clear_result()
            self._report("warning", "Simulación cancelada.")
            return None
        except Exception as e:  # noqa: BLE001 — que nunca quede la interfaz trabada
            self._clear_result()
            self._report("error", f"Error interno simulando: {e!r}")
            self.summary.setText(f"<b style='color:#d04040'>Error interno simulando:</b> {e!r}")
            return None
        finally:
            QGuiApplication.restoreOverrideCursor()
            self._running = False
            self._set_controls_enabled(True)
            self.cancel_btn.setVisible(False)

        self.result = result
        self.stale = False
        self.timeline = Timeline(result)
        self._draw_path(simulator)
        self._show_issues(result)
        self._show_summary(result)
        self.set_time(0.0)
        return result

    def _set_controls_enabled(self, enabled: bool) -> None:
        """Mientras simula, nada de lo que la simulación usa se puede cambiar
        (el progreso procesa eventos: sin esto se podía cambiar el robot o abrir
        otro respaldo a mitad de camino y el resultado quedaba mal atribuido)."""
        for widget in (self.simulate_btn, self.model_combo, self.open_backup_btn,
                       self.side_tabs, self.open_cell_btn, self.save_cell_btn,
                       self.inputs_scroll):
            widget.setEnabled(enabled)
        self.use_editor_btn.setEnabled(enabled and self.backup is not None)

    def cancel(self) -> None:
        """Cortar una simulación en curso (p. ej. al cerrar la ventana)."""
        self._cancel = True

    def source_changed(self) -> None:
        """El programa del editor (quizás) cambió. Se compara el texto: el
        resaltado de sintaxis dispara la misma señal sin cambiar nada."""
        if self.backup is None and self._get_source() != self._simulated_source:
            self._mark_stale("cambió el programa")

    def _progress(self, _actions: int) -> None:
        QApplication.processEvents()
        if self._cancel:
            raise Cancelled

    def _on_cancel(self) -> None:
        self._cancel = True

    def _show_issues(self, result: SimResult) -> None:
        self.issues_list.clear()
        colors = {"error": "#d04040", "aviso": "#c08000", "info": "#4080c0"}
        for issue in result.issues:
            item = QListWidgetItem(f"[{issue.severity}] {issue.where}: {issue.message}")
            item.setForeground(QColor(colors.get(issue.severity, "#808080")))
            item.setData(Qt.ItemDataRole.UserRole, issue.time_s)
            self.issues_list.addItem(item)
        if result.issues:
            self.side_tabs.setCurrentIndex(0)
        severity = {"error": "error", "aviso": "warning", "info": "info"}
        for issue in result.issues[:MAX_REPORTED_ISSUES]:
            self._report(severity.get(issue.severity, "info"),
                         f"[{self.model.name}] {issue.where}: {issue.message}")
        rest = len(result.issues) - MAX_REPORTED_ISSUES
        if rest > 0:
            self._report("warning", f"… y {rest} problema(s) más: ver la pestaña Problemas de "
                                    f"Simulación 3D.")

    def _show_summary(self, result: SimResult) -> None:
        errors = sum(i.severity == "error" for i in result.issues)
        warnings = sum(i.severity == "aviso" for i in result.issues)
        done = result.total_moves - result.skipped_moves
        parts = [f"<b>{self.model.name}</b>: {done} de {result.total_moves} movimiento(s) "
                 f"simulado(s)"]
        if result.skipped_moves:
            causes = []
            if result.failed_moves:
                causes.append(f"{result.failed_moves} no llegan")
            if result.no_frames_moves:
                causes.append(f"{result.no_frames_moves} sin herramienta/coordenadas")
            if result.unevaluated_moves:
                causes.append(f"{result.unevaluated_moves} sin pose conocida")
            parts.append(f"<b style='color:#d04040'>⚠ {result.skipped_moves} sin simular "
                         f"({', '.join(causes)})</b>")
        time_note = " (parcial)" if result.skipped_moves or not result.complete else ""
        cycle = " (un ciclo: el programa se repite)" if result.cyclic else ""
        parts.append(f"tiempo de ciclo estimado {result.total_time_s:.1f} s{time_note}{cycle}, "
                     f"sin aceleraciones")
        parts.append(f"{errors} error(es), {warnings} aviso(s)")
        if not result.complete:
            parts.append("<b>simulación cortada (ver Problemas)</b>")
        missing = []
        if result.missing_tools:
            missing.append("herramienta " + ", ".join(map(str, result.missing_tools)))
        if result.missing_frames:
            missing.append("coordenadas " + ", ".join(map(str, result.missing_frames)))
        if missing:
            parts.append(f"<b>falta cargar {' y '.join(missing)}</b> con los valores del pad")
        self.load_missing_btn.setVisible(bool(missing))
        self.summary.setText(" · ".join(parts) + ". <i>Modelo con supuestos sin confirmar.</i>")
        self._report("info", f"Simulación en {self.model.name}: {done}/{result.total_moves} "
                             f"movimientos, {result.total_time_s:.1f} s{time_note}, {errors} "
                             f"error(es), {warnings} aviso(s).")

    def _draw_path(self, simulator: PadSimulator) -> None:
        if self.viewport is None or self.result is None:
            return
        points, colors = [], []
        t = 0.0
        for seg in self.result.segments:
            if seg.kind in ("MOVEJ", "MOVEL") and len(seg.samples) > 1:
                if seg.failed:
                    color = COLOR_FAILED
                elif seg.kind == "MOVEJ":
                    color = COLOR_MOVEJ
                else:
                    color = COLOR_OUTPUT_ON if self._any_output_on(t) else COLOR_MOVEL
                for q in seg.samples:
                    points.append(simulator.tcp(q, seg.tool))
                    colors.append(color)
            t += seg.duration_s
        self.viewport.set_path(points, colors)

    def _any_output_on(self, t: float) -> bool:
        state: dict[int, bool] = {}
        for when, point, on in self.result.outputs:
            if when > t + 1e-9:
                break
            state[point] = on
        return any(state.values())

    def _on_issue_activated(self, item: QListWidgetItem) -> None:
        t = item.data(Qt.ItemDataRole.UserRole)
        if t is not None and self.timeline is not None:
            self.play_btn.setChecked(False)
            self.set_time(float(t))

    # -- herramientas y coordenadas -------------------------------------------------------

    def _on_frames_changed(self) -> None:
        self.layout_data.tools = self.tools_table.values()
        self.layout_data.frames = self.frames_table.values()
        self._update_axes()
        self._mark_stale("cambiaron las herramientas o las coordenadas")

    def _update_axes(self) -> None:
        if self.viewport is None:
            return
        frames = [identity()] + [pose_matrix(*v) for v in self.layout_data.frames.values()]
        self.viewport.set_axes(frames)

    def set_tool(self, number: int, pose: list[float]) -> None:
        self.tools_table.set_values({**self.tools_table.values(), number: list(pose)})
        self._on_frames_changed()

    def set_frame(self, number: int, pose: list[float]) -> None:
        self.frames_table.set_values({**self.frames_table.values(), number: list(pose)})
        self._on_frames_changed()

    def add_missing_frames(self) -> None:
        """Agrega filas vacías (en rojo) con los números que faltan, y muestra
        la pestaña: el usuario solo tiene que copiar los valores del pad."""
        if self.result is None:
            return
        for number in self.result.missing_frames:
            if number not in self.frames_table.values():
                self.frames_table.add_row(number, blank=True)
        for number in self.result.missing_tools:
            if number not in self.tools_table.values():
                self.tools_table.add_row(number, blank=True)
        self.side_tabs.setCurrentWidget(self.tools_table if self.result.missing_tools
                                        else self.frames_table)

    # -- animación y cámara -------------------------------------------------------------

    def set_time(self, t: float) -> None:
        if self.timeline is None:
            return
        self._t = max(0.0, min(t, self.timeline.duration))
        q, where = self.timeline.at(self._t)
        self.show_pose(q)
        self.time_label.setText(f"{self._t:6.1f} / {self.timeline.duration:.1f} s  {where}")
        if self.timeline.duration > 0:
            self.slider.blockSignals(True)
            self.slider.setValue(round(self._t / self.timeline.duration * SLIDER_STEPS))
            self.slider.blockSignals(False)

    def show_pose(self, q: list[float]) -> None:
        self.current_q = list(q)
        if self.viewport is not None and self.model is not None:
            self.viewport.set_joint_frames(self.model.joint_frames(q))

    def set_camera(self, view: str) -> None:
        if self.viewport is None:
            return
        if view != "fit":
            self.viewport.set_view(view)
            return
        pts = []
        if self.result is not None and self.model is not None:
            for seg in self.result.segments:
                for q in seg.samples[:: max(1, len(seg.samples) // 4)]:
                    m = self.model.fk(q)
                    pts.append((m[0][3], m[1][3], m[2][3]))
        for obj in self.layout_data.objects:
            pts.append((obj.x, obj.y, obj.z))
        pts += [(0, 0, 0), (0, 0, self.model.d1 + self.model.a2 if self.model else 1200)]
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        center = tuple((a + b) / 2 for a, b in zip(lo, hi))
        size = max(b - a for a, b in zip(lo, hi))
        self.viewport.set_view("iso", center, size)

    def _on_slider(self, value: int) -> None:
        if self.timeline is not None:
            self.set_time(value / SLIDER_STEPS * self.timeline.duration)

    def _on_play_toggled(self, playing: bool) -> None:
        if playing and (self.timeline is None or self.stale):
            self.play_btn.setChecked(False)  # nada que reproducir
            return
        self.play_btn.setText("❚❚" if playing else "▶")
        if playing:
            if self._t >= self.timeline.duration:
                self._t = 0.0
            self._timer.start()
        else:
            self._timer.stop()

    def _tick(self) -> None:
        if self.timeline is None:
            self.play_btn.setChecked(False)
            return
        factor = float(self.speed_combo.currentText().lstrip("x"))
        self.set_time(self._t + FRAME_MS / 1000 * factor)
        if self._t >= self.timeline.duration:
            self.play_btn.setChecked(False)

    # -- piezas ---------------------------------------------------------------------------

    def _mesh_for(self, path: str) -> Mesh:
        if path not in self._mesh_cache:
            if not Path(path).exists():
                raise MeshError(f"No existe el archivo {path}")
            QGuiApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            try:
                self._mesh_cache[path] = load_mesh(path)
            except MeshError:
                raise
            except Exception as e:  # noqa: BLE001 — archivo corrupto de mil formas
                raise MeshError(f"No se pudo leer {Path(path).name}: {e}") from e
            finally:
                QGuiApplication.restoreOverrideCursor()
        return self._mesh_cache[path]

    def import_object(self, path: str | Path) -> LayoutObject | None:
        path = str(path)
        try:
            mesh = self._mesh_for(path)
            (xmin, ymin, zmin), (xmax, ymax, _) = mesh.bounds()
        except MeshError as e:
            self._report("error", str(e))
            return None
        # Frente al robot y apoyada en el piso (no adentro de la base).
        obj = LayoutObject(name=Path(path).stem, path=path,
                           x=round(IMPORT_AT[0] - (xmin + xmax) / 2, 1),
                           y=round(IMPORT_AT[1] - (ymin + ymax) / 2, 1),
                           z=round(-zmin, 1) or 0.0)
        self.layout_data.objects.append(obj)
        self._refresh_objects(rebuild=True)
        self._report("info", f"Importado {Path(path).name} ({len(mesh)} triángulos).")
        return obj

    def load_layout(self, path: str | Path) -> bool:
        try:
            layout = Layout.load(path)
        except Exception as e:  # noqa: BLE001 — JSON roto, claves de más, etc.
            self._report("error", f"No se pudo abrir la celda {Path(path).name}: {e}")
            return False
        self.layout_data = layout
        self.layout_path = Path(path)
        if layout.model in RobotModel.available():
            self.model_combo.setCurrentText(layout.model)
        else:
            self._report("warning", f"La celda usa el robot {layout.model}, que no está "
                                    f"instalado: se usa {self.model_combo.currentText()}.")
        self.layout_data.model = self.model_combo.currentText()
        self.tools_table.set_values(layout.tools)
        self.frames_table.set_values(layout.frames)
        self.layout_data.tools = self.tools_table.values()     # la tabla manda
        self.layout_data.frames = self.frames_table.values()
        if self.viewport is None:  # con vista 3D, el aviso sale al cargar la malla
            for obj in layout.objects:
                if not Path(obj.path).exists():
                    self._report("warning", f"La pieza «{obj.name}» apunta a {obj.path}, que no existe.")
        self._refresh_objects(rebuild=True)
        self._update_axes()
        self._mark_stale("se abrió otra celda")
        self._report("info", f"Celda abierta: {Path(path).name} ({len(layout.objects)} pieza(s), "
                             f"{len(layout.tools)} herramienta(s), {len(layout.frames)} sistema(s)).")
        return True

    def save_layout(self, path: str | Path) -> bool:
        self.layout_data.tools = self.tools_table.values()
        self.layout_data.frames = self.frames_table.values()
        try:
            self.layout_data.save(path)
        except Exception as e:  # noqa: BLE001 — sin permiso, disco lleno, etc.
            self._report("error", f"No se pudo guardar la celda: {e}")
            return False
        self.layout_path = Path(path)
        self._report("info", f"Celda guardada en {path}")
        return True

    def _refresh_objects(self, rebuild: bool) -> None:
        """`rebuild`: volver a crear las mallas (piezas agregadas/quitadas). Si
        solo se movieron, alcanza con reubicarlas."""
        self._updating_table = True
        self.objects_table.setRowCount(len(self.layout_data.objects))
        for row, obj in enumerate(self.layout_data.objects):
            values = [obj.name, _fmt(obj.x), _fmt(obj.y), _fmt(obj.z), _fmt(obj.rz)]
            for col, value in enumerate(values):
                self.objects_table.setItem(row, col, QTableWidgetItem(value))
        self._updating_table = False
        if self.viewport is None:
            return
        if rebuild or self.viewport.object_count() != len(self.layout_data.objects):
            self.viewport.clear_objects()
            self._object_index: dict[int, int] = {}
            for i, obj in enumerate(self.layout_data.objects):
                try:
                    self._object_index[i] = self.viewport.add_object(self._mesh_for(obj.path), obj.color)
                except MeshError as e:
                    self._report("error", str(e))
        for i, obj in enumerate(self.layout_data.objects):
            index = self._object_index.get(i)
            if index is not None:
                self.viewport.place_object(index, object_matrix(obj))

    def _on_object_edited(self, item: QTableWidgetItem) -> None:
        if self._updating_table:
            return
        obj = self.layout_data.objects[item.row()]
        field = ["name", "x", "y", "z", "rz"][item.column()]
        if field == "name":
            obj.name = item.text()
            return
        try:
            setattr(obj, field, _number(item.text()))
        except ValueError:
            self._report("warning", f"«{item.text()}» no es un número")
        self._refresh_objects(rebuild=False)

    # -- diálogos -------------------------------------------------------------------------

    def _on_open_backup(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Abrir respaldo del pad", filter="Respaldo del pad (*.zip)")
        if path and self.open_backup(path):
            self.simulate()

    def _on_import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Importar pieza 3D", filter="Modelos 3D (*.step *.stp *.stl *.obj)")
        if path:
            self.import_object(path)

    def _on_remove(self) -> None:
        row = self.objects_table.currentRow()
        if 0 <= row < len(self.layout_data.objects):
            del self.layout_data.objects[row]
            self.objects_table.clearSelection()
            self.objects_table.setCurrentCell(-1, -1)
            self._refresh_objects(rebuild=True)

    def _on_open_layout(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Abrir celda", filter="Celda (*.layout.json *.json)")
        if path:
            self.load_layout(path)

    def _on_save_layout(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Guardar celda", filter="Celda (*.layout.json)")
        if path:
            if not path.endswith(".json"):
                path += ".layout.json"
            self.save_layout(path)
