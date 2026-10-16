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
  programa, piezas y margen si se buscan choques), el resultado se marca
  desactualizado.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Callable

from PySide6.QtCore import QSettings, Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QGuiApplication, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QGroupBox,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QStackedWidget,
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
from sim import collision
from sim.kinematics import RobotModel, identity, mat_mul, matrix_to_pose, pose_matrix, rot_axis
from sim.meshes import Mesh, MeshError, load_mesh
from sim import placement
from sim.motion import MotionCurves, analyze
from sim.pad_sim import Cancelled, PadSimulator, SimResult, inputs_used
from sim.scene import Layout, LayoutObject, Timeline, has_real_meshes, robot_link_meshes

SLIDER_STEPS = 1000
FRAME_MS = 33
OBJECT_COLUMNS = ["Pieza", "X", "Y", "Z", "Giro Z°", "Se trabaja"]
WORKPIECE_COLUMN = 5
POSE_COLUMNS = ["N°", "X", "Y", "Z", "U", "V", "W"]
DEFAULT_MODEL = "BRTIRUS1510A"  # el robot de la celda
# Arranque y frenado de cada movimiento. HIPÓTESIS: el respaldo no trae la
# aceleración del controlador; ver sim/pad_sim.py.
DEFAULT_ACCEL_S = 0.25
IMPORT_AT = (1200.0, 0.0)      # dónde aparece una pieza importada (frente al robot)
MAX_REPORTED_ISSUES = 12       # a la ventana de mensajes; el resto, en "Problemas"
COLOR_MOVEJ = (0.35, 0.75, 1.0)
COLOR_MOVEL = (1.0, 0.85, 0.2)
COLOR_OUTPUT_ON = (1.0, 0.45, 0.1)   # MOVEL con alguna salida prendida (p. ej. soldando)
COLOR_FAILED = (1.0, 0.15, 0.15)
INVALID_BRUSH = QBrush(QColor(220, 60, 60, 110))
COLOR_HIT = "#e02828"     # pieza contra la que el robot choca en este momento
COLOR_NEAR = "#f0a020"    # pieza más cerca que el margen

Reporter = Callable[[str, str], None]  # (severidad "info"|"warning"|"error", mensaje)


def object_matrix(obj: LayoutObject):
    r = rot_axis((0, 0, 1), math.radians(obj.rz))
    return [r[0] + [obj.x], r[1] + [obj.y], r[2] + [obj.z], [0.0, 0.0, 0.0, 1.0]]


def parse_pose(text: str) -> list[float]:
    """'0, 0, 120, 0, 0, 0' -> 6 números (acepta ';' o espacios entre medio)."""
    items = [x for x in text.replace(";", " ").replace(",", " ").split() if x]
    if len(items) != 6:
        raise ValueError(f"hacen falta 6 números (X, Y, Z, U, V, W) y hay {len(items)}")
    return [_number(x) for x in items]


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


def _snap(degrees: float) -> float:
    """Ángulo redondeado: girar de a 90° deja 90, no 89.99999999."""
    value = round(degrees, 6)
    return 0.0 if value == 0 else value


def _vscroll(widget: QWidget) -> QScrollArea:
    """Panel con scroll vertical: si no entra en alto, no agranda la ventana
    ni empuja a los demás."""
    area = QScrollArea()
    area.setWidget(widget)
    area.setWidgetResizable(True)
    area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    area.setFrameShape(QScrollArea.Shape.NoFrame)
    return area


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
                 clear_reports: Callable[[], None] | None = None,
                 compile_pad: Callable[[PadOptions], tuple] | None = None) -> None:
        super().__init__()
        self._get_source = get_source
        # Programa del editor -> (respaldo, avisos). La ventana principal pasa
        # uno que junta .src, .dat y config.dat y dice archivo y línea.
        self._compile_pad = compile_pad or (
            lambda options: compile_to_pad_report(self._get_source(), options))
        self._report = report or (lambda _severity, _msg: None)
        self._clear_reports = clear_reports or (lambda: None)
        self.layout_data = Layout()
        self.layout_path: Path | None = None
        self.backup: PadBackup | None = None      # respaldo abierto (None = editor)
        self.backup_path: Path | None = None
        self.result: SimResult | None = None
        self.collisions: collision.CollisionReport | None = None
        self.curves: MotionCurves | None = None
        self.charts = None  # ventana de gráficas (se crea al abrirla)
        self._accel_used = 0.0
        self._measure_key = None            # robot armado para medir distancias
        self._measure_checker_obj = None
        self._painted: dict[int, str] = {}  # pieza -> color que se le puso por choque
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
        self.model_combo.setToolTip("Robots instalados y, debajo, los de la biblioteca de Drive "
                                    "(se bajan, se importan y se verifican al elegirlos)")
        self.model_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.model_combo.addItems(RobotModel.available())
        self.model_combo.currentIndexChanged.connect(self._on_model_chosen)
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

        # Avance de un robot que se baja e importa de la biblioteca (tarda
        # minutos): a la vista, al lado de la lista de robots.
        self.robot_progress = QLabel()
        self.robot_progress.setWordWrap(True)
        self.robot_progress.setVisible(False)

        top = QHBoxLayout()
        top.addWidget(QLabel("Robot:"))
        top.addWidget(self.model_combo)
        top.addWidget(self.robot_progress, 2)
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
        self.accel_spin = QDoubleSpinBox()
        self.accel_spin.setRange(0.0, 2.0)
        self.accel_spin.setDecimals(2)
        self.accel_spin.setSingleStep(0.05)
        self.accel_spin.setSuffix(" s")
        self.accel_spin.setToolTip(
            "Tiempo que tarda cada movimiento en acelerar (y en frenar). SUPUESTO: el respaldo "
            "del pad no trae la aceleración real del controlador. 0 = velocidad constante.")
        try:
            accel = float(self._settings.value("sim/accel_s", DEFAULT_ACCEL_S))
        except (TypeError, ValueError):
            accel = DEFAULT_ACCEL_S
        self.accel_spin.setValue(accel)
        self.accel_spin.valueChanged.connect(self._on_accel_changed)
        self.accel_auto = QCheckBox("del datasheet")
        self.accel_auto.setToolTip("Usar las aceleraciones máximas de cada eje cargadas en el "
                                   "modelo (planilla de parámetros del robot)")
        self.accel_auto.setChecked(True)
        self.accel_auto.toggled.connect(self._on_accel_auto)
        self.charts_btn = QPushButton("Gráficas…")
        self.charts_btn.setToolTip("Posición, velocidad y aceleración de cada eje y de la punta")
        self.charts_btn.clicked.connect(self.show_charts)
        bottom = QHBoxLayout()
        bottom.addWidget(self.play_btn)
        bottom.addWidget(self.speed_combo)
        bottom.addWidget(self.slider, 1)
        bottom.addWidget(self.time_label)
        bottom.addWidget(QLabel("Aceleración:"))
        bottom.addWidget(self.accel_spin)
        bottom.addWidget(self.accel_auto)
        bottom.addWidget(self.charts_btn)

        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)
        if self.viewport is not None:
            cams = QHBoxLayout()
            mouse = ("Como Inventor/AutoCAD: rueda = zoom hacia el cursor; botón del medio "
                     "(o derecho) = desplazar; Shift + medio (o izquierdo) = orbitar; doble clic "
                     "con el medio o F6 = encuadrar; flechas = desplazar.")
            self.viewport.on_fit = lambda: self.set_camera("fit")
            fit_btn = QPushButton("Encuadrar")
            fit_btn.setToolTip(mouse)
            fit_btn.clicked.connect(lambda: self.set_camera("fit"))
            self.view_combo = QComboBox()
            self.view_combo.setToolTip("Vistas estándar, encuadradas (como el ViewCube)")
            for label, view in (("Vista…", None), ("Iso", "iso"), ("Arriba", "arriba"),
                                ("Frente", "frente"), ("Lado", "lado"), ("Atrás", "atras"),
                                ("Izquierda", "izquierda")):
                self.view_combo.addItem(label, view)
            self.view_combo.activated.connect(self._on_view_chosen)
            hint = QLabel("Rueda: zoom · Medio: desplazar · Shift+medio: orbitar")
            hint.setToolTip(mouse)
            cams.addWidget(fit_btn)
            cams.addWidget(self.view_combo)
            cams.addWidget(hint, 1)
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
        objects_layout.addWidget(self._build_robot_base())
        objects_note = QLabel("<b>Piezas</b> STEP/STL/OBJ. X, Y, Z: dónde queda el origen del CAD, "
                              "en mm, en coordenadas de la celda (piso en Z = 0).")
        objects_note.setWordWrap(True)
        objects_layout.addWidget(objects_note)
        objects_layout.addWidget(self.objects_table, 1)
        row = QHBoxLayout()
        row.addWidget(import_btn)
        row.addWidget(remove_btn)
        objects_layout.addLayout(row)
        objects_layout.addWidget(self._build_placement())
        self.objects_table.itemSelectionChanged.connect(self._on_piece_selected)

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

        from gui.library_view import LibraryPanel

        self.library = LibraryPanel(on_insert=self._insert_from_library,
                                    on_tool=self._tool_from_library,
                                    on_robot=self.use_robot, report=self._report,
                                    settings=self._settings, on_catalog=self._on_catalog,
                                    on_progress=self._on_library_progress)
        # Biblioteca y Celda primero: con el panel angosto, las últimas
        # pestañas quedan escondidas detrás de las flechitas.
        tabs = QTabWidget()
        tabs.addTab(issues, "Problemas")
        self.cell_tab = _vscroll(objects)
        tabs.addTab(self.cell_tab, "Celda")
        self.library_tab_index = tabs.addTab(self.library, "Biblioteca")
        tabs.addTab(_vscroll(self._build_collisions()), "Choques")
        tabs.addTab(self.tools_table, "Herramientas")
        tabs.addTab(self.frames_table, "Coordenadas")
        tabs.currentChanged.connect(
            lambda i: self.library.ensure_loaded() if i == self.library_tab_index else None)
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

    def _build_robot_base(self) -> QWidget:
        """Dónde está parado el robot en la celda: corrido y girado."""
        group = QGroupBox("Robot en la celda (mm, °)")
        self.base_spins: list[QDoubleSpinBox] = []
        g = QGridLayout(group)
        for i, (label, lo, hi) in enumerate(
                (("X", -20000, 20000), ("Y", -20000, 20000), ("Z", -5000, 10000),
                 ("Rx", -180, 180), ("Ry", -180, 180), ("Rz", -180, 180))):
            box = QDoubleSpinBox()
            box.setRange(lo, hi)
            box.setDecimals(1)
            box.setMinimumWidth(50)
            box.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            if i >= 3:
                box.setWrapping(True)
            box.valueChanged.connect(self._on_base_spin)
            self.base_spins.append(box)
            # X | Rx, Y | Ry, Z | Rz: dos columnas, entra en el panel angosto.
            g.addWidget(QLabel(label), i % 3, (i // 3) * 2)
            g.addWidget(box, i % 3, (i // 3) * 2 + 1)
        for col in (1, 3):
            g.setColumnStretch(col, 1)
        buttons = QGridLayout()        # 2 x 2: entra en el panel angosto
        for k, axis in enumerate("XYZ"):
            btn = QPushButton(f"Girar 90° {axis}")
            btn.setToolTip(f"Gira el robot 90° alrededor del eje {axis} de la celda, sobre su "
                           f"base. Shift+clic: -90°.")
            btn.clicked.connect(lambda _c=False, a=axis: self.rotate_robot(
                a, -90.0 if QGuiApplication.keyboardModifiers() & Qt.KeyboardModifier.ShiftModifier
                else 90.0))
            buttons.addWidget(btn, k // 2, k % 2)
        reset = QPushButton("Al origen")
        reset.setToolTip("Parado en el origen de la celda, sin girar")
        reset.clicked.connect(lambda: self.set_robot_base([0.0] * 6))
        buttons.addWidget(reset, 1, 1)
        g.addLayout(buttons, 3, 0, 1, 4)
        note = QLabel("Los puntos del programa son respecto de la base: se mueven con el robot. "
                      "Las piezas y el piso, no.")
        note.setWordWrap(True)
        note.setToolTip("Rx, Ry, Rz: giro en X, después en Y, después en Z, sobre los ejes de la "
                        "celda. El simulador no sabe si el fabricante permite montar el robot "
                        "así (en pared, colgado): revisarlo en el datasheet.")
        g.addWidget(note, 4, 0, 1, 4)
        return group

    def _build_placement(self) -> QWidget:
        """Ubicar la pieza elegida por distancias (sim/placement.py) y medir."""
        def spin(lo: float, hi: float, value: float = 0.0, suffix: str = " mm") -> QDoubleSpinBox:
            box = QDoubleSpinBox()
            box.setRange(lo, hi)
            box.setDecimals(1)
            box.setSuffix(suffix)
            box.setValue(value)
            return box

        self.place_title = QLabel("<b>Ubicar</b>: elegí una pieza en la tabla.")
        self.place_mode = QComboBox()
        self.place_mode.addItems(["A una distancia del robot", "Corrida respecto de…",
                                  "Al lado de…", "Encima de…"])
        self.place_ref = QComboBox()
        self.place_stack = QStackedWidget()

        def grid(rows) -> QWidget:
            w = QWidget()
            g = QGridLayout(w)
            g.setContentsMargins(0, 0, 0, 0)
            for r, (label, widget) in enumerate(rows):
                g.addWidget(QLabel(label), r, 0)
                g.addWidget(widget, r, 1)
            g.setColumnStretch(1, 1)
            return w

        self.place_distance = spin(0, 20000, 800)
        self.place_angle = spin(-180, 180, 0, "°")
        self.place_angle.setToolTip("0° = adelante del robot (+X), 90° = a su izquierda (+Y)")
        self.place_to = QComboBox()
        self.place_to.addItems(["hasta la cara", "hasta el centro"])
        robot = grid([("Distancia", self.place_distance), ("Medida", self.place_to),
                      ("Ángulo", self.place_angle)])
        self.place_dx, self.place_dy, self.place_dz = (spin(-20000, 20000) for _ in range(3))
        offset = grid([("ΔX", self.place_dx), ("ΔY", self.place_dy), ("ΔZ", self.place_dz)])
        self.place_side = QComboBox()
        self.place_side.addItems(list(placement.SIDES))
        self.place_gap = spin(-5000, 20000, 200)
        side = grid([("Lado", self.place_side), ("Separación entre caras", self.place_gap)])
        self.place_top_dx, self.place_top_dy = spin(-20000, 20000), spin(-20000, 20000)
        top = grid([("Corrida ΔX", self.place_top_dx), ("Corrida ΔY", self.place_top_dy)])

        for w in (robot, offset, side, top):
            self.place_stack.addWidget(w)
        self.place_mode.currentIndexChanged.connect(self._on_place_mode)
        apply_btn = QPushButton("Ubicar")
        apply_btn.clicked.connect(self.apply_placement)
        measure_btn = QPushButton("Medir distancias")
        measure_btn.setToolTip("Distancia mínima real (entre superficies) al robot, en la pose "
                               "que se ve, y a cada pieza")
        measure_btn.clicked.connect(self.measure_selected)
        self.place_info = QLabel()
        self.place_info.setWordWrap(True)
        self.place_info.setTextFormat(Qt.TextFormat.RichText)
        self.place_info.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

        box = QWidget()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(0, 6, 0, 0)
        lay.addWidget(self.place_title)
        lay.addWidget(self.place_mode)
        lay.addWidget(self.place_ref)
        lay.addWidget(self.place_stack)
        row = QHBoxLayout()
        row.addWidget(apply_btn)
        row.addWidget(measure_btn)
        row.addStretch(1)
        lay.addLayout(row)
        lay.addWidget(self.place_info)
        note = QLabel("Las distancias se miden a la caja de cada pieza (con su giro), y el apoyo "
                      "es el centro de su base. Las distancias al robot se miden desde su "
                      "base; 0° es hacia donde mira el robot.")
        note.setWordWrap(True)
        lay.addWidget(note)
        self._on_place_mode(0)
        return box

    def _build_collisions(self) -> QWidget:
        self.collisions_check = QCheckBox("Buscar choques al simular")
        self.collisions_check.setChecked(collision.available())
        self.collisions_check.setEnabled(collision.available())
        if not collision.available():
            self.collisions_check.setToolTip("Falta python-fcl en esta instalación")
        self.collisions_check.toggled.connect(self._on_collision_settings)
        self.margin_spin = QDoubleSpinBox()
        self.margin_spin.setRange(0.0, 500.0)
        self.margin_spin.setDecimals(0)
        self.margin_spin.setSuffix(" mm")
        self.margin_spin.setValue(self.layout_data.margin_mm)
        self.margin_spin.valueChanged.connect(self._on_collision_settings)
        self.tool_mesh_label = QLabel()
        self.tool_mesh_label.setWordWrap(True)
        choose = QPushButton("Elegir modelo 3D…")
        choose.clicked.connect(self._on_choose_tool_mesh)
        self.remove_tool_mesh_btn = QPushButton("Quitar")
        self.remove_tool_mesh_btn.clicked.connect(lambda: self.set_tool_mesh(""))
        self.tool_mount_edit = QLineEdit()
        self.tool_mount_edit.setPlaceholderText("0, 0, 0, 0, 0, 0")
        self.tool_mount_edit.setToolTip("Dónde queda el origen del CAD de la herramienta respecto "
                                        "de la brida: X, Y, Z en mm y U, V, W en grados. Z sale "
                                        "de la brida.")
        self.tool_mount_edit.editingFinished.connect(self._on_tool_mount_edited)

        box = QWidget()
        lay = QVBoxLayout(box)
        lay.addWidget(self.collisions_check)
        row = QHBoxLayout()
        row.addWidget(QLabel("Margen de seguridad:"))
        row.addWidget(self.margin_spin)
        row.addStretch(1)
        lay.addLayout(row)
        lay.addWidget(QLabel("<b>Herramienta montada en la brida</b> (antorcha, pinza…):"))
        lay.addWidget(self.tool_mesh_label)
        row = QHBoxLayout()
        row.addWidget(choose)
        row.addWidget(self.remove_tool_mesh_btn)
        row.addStretch(1)
        lay.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel("Montaje:"))
        row.addWidget(self.tool_mount_edit, 1)
        lay.addLayout(row)
        note = QLabel(
            "Las piezas cuentan como sólidos. Se revisa el brazo y la herramienta contra las "
            "piezas, contra el piso (debajo de la base) y contra el mismo brazo, sin saltear "
            "nada entre muestras. Más cerca que el margen es un aviso; tocarse, un error.<br>"
            "En la pieza que se suelda o se agarra tildá <i>Se trabaja</i> (pestaña Piezas): "
            "para la herramienta solo cuenta tocarla. El brazo sí usa el margen.<br>"
            "<i>Mientras no estén las mallas del fabricante, el robot son cilindros "
            "aproximados: los choques son estimaciones.</i>")
        note.setWordWrap(True)
        note.setTextFormat(Qt.TextFormat.RichText)
        lay.addWidget(note)
        lay.addStretch(1)
        self._show_tool_mesh()
        return box

    # -- modelo -----------------------------------------------------------------------

    def _select_initial_model(self) -> None:
        names = RobotModel.available()
        if not names:
            self._report("error", "No hay ningún modelo de robot válido instalado.")
            return
        last = self._settings.value("sim/model", "")
        name = last if last in names else (DEFAULT_MODEL if DEFAULT_MODEL in names else names[0])
        self._fill_model_combo(name)
        self.set_model(name)

    def set_model(self, name: str) -> bool:
        try:
            model = RobotModel.load(name)
            meshes = self._robot_meshes(model) if self.viewport is not None else None
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
        if hasattr(self, "accel_auto"):
            self._update_accel_controls()
        self.show_pose(self.current_q)
        if previous is not None and previous.name != name:
            self._mark_stale(f"se cambió el robot a {name}")
        return True

    def _robot_meshes(self, model: RobotModel) -> list[Mesh]:
        """Eslabones para dibujar, con la herramienta pegada a la brida."""
        meshes = robot_link_meshes(model)
        tool = self._tool_mesh()
        if tool is not None:
            flange = Mesh(list(meshes[6].triangles))
            meshes[6] = flange.extend(collision.tool_in_flange_zero(
                model, tool, self.layout_data.tool_mount))
        return meshes

    def _tool_mesh(self) -> Mesh | None:
        path = self.layout_data.tool_mesh
        if not path:
            return None
        try:
            return self._mesh_for(path)
        except MeshError as e:
            self._report("error", f"Herramienta: {e}")
            return None

    def _insert_from_library(self, path: Path, item) -> None:
        obj = self.import_object(path)
        if obj is not None:
            obj.name = Path(item.name).stem
            obj.drive_id = item.file_id
            self._refresh_objects(rebuild=False)
            self.objects_table.selectRow(len(self.layout_data.objects) - 1)
            self.side_tabs.setCurrentWidget(self.cell_tab)   # a Celda, para ubicarla

    def _tool_from_library(self, path: Path, item) -> None:
        if self.set_tool_mesh(path):
            self.layout_data.tool_drive_id = item.file_id

    def _fetch_missing(self) -> None:
        """Piezas de la biblioteca que en esta PC no están bajadas: se bajan."""
        from sim.library import LibraryError

        wanted = [(obj, "path", obj.drive_id) for obj in self.layout_data.objects if obj.drive_id]
        if self.layout_data.tool_drive_id:
            wanted.append((self.layout_data, "tool_mesh", self.layout_data.tool_drive_id))
        for owner, attr, drive_id in wanted:
            current = getattr(owner, attr)
            if current and Path(current).exists():
                continue
            name = Path(current).name if current else drive_id
            QGuiApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            try:
                setattr(owner, attr, str(self.library.fetch_by_id(drive_id, name)))
            except LibraryError as e:
                self._report("warning", f"No se pudo bajar «{name}» de la biblioteca: {e}")
            finally:
                QGuiApplication.restoreOverrideCursor()

    def use_robot(self, name: str) -> bool:
        """Elegir un robot (p. ej. recién importado de la biblioteca)."""
        if name not in RobotModel.available():
            self._report("error", f"El robot {name} no está instalado.")
            return False
        self._fill_model_combo(name)
        return self.set_model(name)

    def _fill_model_combo(self, current: str | None = None) -> None:
        """Los instalados y, debajo, los de la biblioteca que todavía no se
        bajaron (o una entrada para ir a buscarlos)."""
        current = current or (self.model.name if self.model is not None else "")
        names = RobotModel.available()
        combo = self.model_combo
        combo.blockSignals(True)
        combo.clear()
        combo.addItems(names)
        remote = [item for item in self.library.robots() if item.model_name not in names]
        if remote or self.library.catalog is None:
            combo.insertSeparator(combo.count())
        for item in remote:
            combo.addItem(f"{item.model_name} (biblioteca)", ("biblioteca", item.file_id))
            combo.setItemData(combo.count() - 1, "Se baja de Google Drive, se importa del STEP del "
                              "fabricante y se verifica (unos minutos la primera vez)",
                              Qt.ItemDataRole.ToolTipRole)
        if self.library.catalog is None:
            combo.addItem("Robots de la biblioteca…", ("abrir",))
        if current in names:
            combo.setCurrentText(current)
        combo.blockSignals(False)

    def _on_catalog(self, _catalog) -> None:
        self._fill_model_combo()

    def _on_library_progress(self, text: str, robot: str | None, state: str) -> None:
        """Solo lo de preparar un robot (leer la carpeta no hace falta mostrarlo acá)."""
        if robot is None:
            return
        color = {"trabajando": "#d08000", "error": "#d04040"}.get(state, "#2f9e44")
        icon = {"trabajando": "⏳ ", "error": "✖ "}.get(state, "✔ ")
        self.robot_progress.setText(f"<span style='color:{color}'>{icon}{text}</span>")
        self.robot_progress.setToolTip(text)
        self.robot_progress.setVisible(True)

    def _on_model_chosen(self, index: int) -> None:
        data = self.model_combo.itemData(index)
        if data is None:
            if not self.library.busy:
                self.robot_progress.setVisible(False)
            self.set_model(self.model_combo.itemText(index))
            return
        # Una entrada de la biblioteca: el robot de la vista sigue siendo el
        # actual hasta que termine de bajarse e importarse.
        self._fill_model_combo()
        if data[0] == "abrir":
            self.side_tabs.setCurrentIndex(self.library_tab_index)
            return
        item = next((i for i in self.library.robots() if i.file_id == data[1]), None)
        if item is None:
            return
        if self.library.preparing == item.model_name:
            self._report("info", f"{item.model_name} ya se está preparando: "
                                 f"{self.library.status.text()}")
            return
        if self.library.busy:
            self.library._report_busy()
            return
        self._report("info", f"Preparando {item.model_name} desde la biblioteca: se baja el "
                             f"STEP y la planilla, se importa y se verifica (unos minutos la "
                             f"primera vez).")
        self.library.prepare_robot(item)

    def showEvent(self, event) -> None:  # noqa: N802 — API de Qt
        """La primera vez que se ve el simulador se lee la biblioteca (en un
        hilo), para que sus robots aparezcan en la lista de Robot."""
        super().showEvent(event)
        self.library.ensure_loaded(quiet=True)

    # -- dónde está el robot ---------------------------------------------------------------

    def set_robot_base(self, pose: list[float]) -> None:
        """X, Y, Z (mm), Rx, Ry, Rz (°) de la base del robot en la celda."""
        pose = [round(float(v), 3) for v in pose]
        changed = pose != list(self.layout_data.robot_base)
        self.layout_data.robot_base = pose
        self._show_robot_base()
        if changed:
            self._mark_collisions_stale("se movió el robot")
            if self.objects_table.rowCount():
                self._on_piece_selected()     # las distancias al robot cambiaron

    def rotate_robot(self, axis: str, degrees: float) -> None:
        """Girar el robot sobre su base alrededor de un eje FIJO de la celda."""
        m = self.layout_data.base_matrix()
        r = rot_axis({"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis],
                     math.radians(degrees))
        turn = [list(row) + [0.0] for row in r] + [[0.0, 0.0, 0.0, 1.0]]
        rotated = mat_mul(turn, m)
        x, y, z = (m[i][3] for i in range(3))            # la base no se corre
        _, _, _, u, v, w = matrix_to_pose(rotated)
        self.set_robot_base([x, y, z] + [_snap(a) for a in (u, v, w)])

    def _on_base_spin(self, _value: float) -> None:
        if not getattr(self, "_showing_base", False):
            self.set_robot_base([box.value() for box in self.base_spins])

    def _show_robot_base(self) -> None:
        self._showing_base = True
        for box, value in zip(self.base_spins, self.layout_data.robot_base):
            box.setValue(value)
        self._showing_base = False
        if self.viewport is not None:
            self.viewport.set_robot_base(self.layout_data.base_matrix())

    def _to_cell(self, point) -> tuple[float, float, float]:
        m = self.layout_data.base_matrix()
        return tuple(sum(m[i][k] * point[k] for k in range(3)) + m[i][3] for i in range(3))

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
            self._refresh_inputs(self._compile_pad(PadOptions(allow_unverified=True))[0])
        except Exception:  # noqa: BLE001 — si el editor no compila, se ve al simular
            pass
        self.source_label.setText("Programa: el del editor")
        self.use_editor_btn.setEnabled(False)
        self._clear_result()

    def _current_backup(self) -> PadBackup:
        if self.backup is not None:
            return self.backup
        # En la simulación se permite lo "sin confirmar": acá no hay riesgo.
        backup, warnings = self._compile_pad(PadOptions(allow_unverified=True))
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
        self.collisions = None
        self.curves = None
        self._paint_collisions(None)
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
                                     frames=self.layout_data.frames, progress=self._progress,
                                     accel_s=self.accel_setting())
            result = simulator.run(backup)
            report = self._check_collisions(result)
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
        self.collisions = report
        if report is not None:
            result.issues.extend(report.issues)
        self._paint_collisions(None)
        self.stale = False
        self.timeline = Timeline(result)
        self._accel_used = self.accel_setting()
        self.curves = analyze(self.timeline, self.model, simulator.tools, self._accel_used)
        self._refresh_charts()
        self._draw_path(simulator)
        self._show_issues(result)
        self._show_summary(result)
        self.set_time(0.0)
        return result

    def _check_collisions(self, result: SimResult) -> collision.CollisionReport | None:
        """Choques sobre la trayectoria simulada (None si no se buscan)."""
        if not self.collisions_check.isChecked() or not collision.available():
            return None
        obstacles = []
        for obj in self.layout_data.objects:
            try:
                mesh = self._mesh_for(obj.path)
            except MeshError as e:
                self._report("warning", f"Choques: «{obj.name}» no se revisa ({e})")
                continue
            obstacles.append(collision.Obstacle(obj.name, mesh, object_matrix(obj), obj.workpiece))
        try:
            checker = collision.CollisionChecker(
                self.model, robot_link_meshes(self.model, tool_axis=False), obstacles,
                tool_mesh=self._tool_mesh(), tool_mount=self.layout_data.tool_mount,
                margin_mm=self.layout_data.margin_mm,
                approximate_robot=not has_real_meshes(self.model),
                base=self.layout_data.base_matrix())
            return checker.check(result, progress=self._progress)
        except Cancelled:
            raise
        except Exception as e:  # noqa: BLE001 — la simulación sirve igual sin los choques
            self._report("error", f"No se pudieron revisar los choques: {e!r}")
            return None

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
        if self.charts is not None:
            self.charts.close()
        self.library.shutdown()

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
        accel = self._accel_used
        if accel is None:
            accel_note = "con las aceleraciones del datasheet"
        elif accel > 0:
            accel_note = f"con arranque/frenado supuesto de {accel:g} s"
        else:
            accel_note = "sin aceleraciones"
        parts.append(f"tiempo de ciclo estimado {result.total_time_s:.1f} s{time_note}{cycle}, "
                     f"{accel_note}")
        parts.append(f"{errors} error(es), {warnings} aviso(s)")
        report = self.collisions
        if report is not None:
            if report.collisions:
                parts.append(f"<b style='color:#d04040'>💥 {report.collisions} choque(s)</b>")
            if report.near_misses:
                parts.append(f"{report.near_misses} paso(s) a menos de "
                             f"{self.layout_data.margin_mm:g} mm")
            if not report.contacts:
                parts.append(f"sin choques (margen {self.layout_data.margin_mm:g} mm)")
        elif not collision.available():
            parts.append("choques sin revisar (falta python-fcl)")
        else:
            parts.append("choques sin revisar")
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
        self._paint_collisions(self.collisions.state_at(self._t) if self.collisions else None)
        if self.charts is not None:
            self.charts.set_time(self._t)
        self.time_label.setText(f"{self._t:6.1f} / {self.timeline.duration:.1f} s  {where}")
        if self.timeline.duration > 0:
            self.slider.blockSignals(True)
            self.slider.setValue(round(self._t / self.timeline.duration * SLIDER_STEPS))
            self.slider.blockSignals(False)

    # -- gráficas ---------------------------------------------------------------------

    def show_charts(self) -> None:
        from gui.motion_charts import MotionCharts

        if self.charts is None:
            self.charts = MotionCharts(on_time=self._chart_clicked)
        self._refresh_charts()
        self.charts.show()
        self.charts.raise_()
        self.charts.activateWindow()

    def _refresh_charts(self) -> None:
        if self.charts is None or self.curves is None or self.model is None:
            return
        self.charts.set_data(self.curves, self.model, self._accel_used, self.collisions)
        self.charts.set_time(self._t)

    def _chart_clicked(self, t: float) -> None:
        self.play_btn.setChecked(False)
        self.set_time(t)

    def accel_setting(self) -> float | None:
        """None = las aceleraciones del modelo; si no, el tiempo del campo."""
        usable = self.model is not None and self.model.has_accelerations
        if usable and self.accel_auto.isChecked():
            return None
        return self.accel_spin.value()

    def _update_accel_controls(self) -> None:
        usable = self.model is not None and self.model.has_accelerations
        self.accel_auto.setEnabled(usable)
        self.accel_auto.setVisible(usable)
        self.accel_spin.setEnabled(not (usable and self.accel_auto.isChecked()))

    def _on_accel_auto(self, _checked: bool) -> None:
        self._update_accel_controls()
        self._mark_stale("cambió de dónde salen las aceleraciones")

    def _on_accel_changed(self, value: float) -> None:
        self._settings.setValue("sim/accel_s", float(value))
        self._mark_stale("cambió el tiempo de aceleración")

    def _paint_collisions(self, state: dict[int, str] | None) -> None:
        """Pinta de rojo (choca) o naranja (cerca) las piezas en este instante."""
        state = state or {}
        if self.viewport is None or state == self._painted:
            return
        for i, obj in enumerate(self.layout_data.objects):
            index = self._object_index.get(i)
            if index is None or state.get(i) == self._painted.get(i):
                continue
            color = {"choque": COLOR_HIT, "cerca": COLOR_NEAR}.get(state.get(i), obj.color)
            self.viewport.set_object_color(index, color)
        self._painted = dict(state)

    def show_pose(self, q: list[float]) -> None:
        self.current_q = list(q)
        if self.viewport is not None and self.model is not None:
            self.viewport.set_joint_frames(self.model.joint_frames(q))

    def _on_view_chosen(self, index: int) -> None:
        view = self.view_combo.itemData(index)
        self.view_combo.setCurrentIndex(0)
        if view:
            self.set_camera(view)

    def set_camera(self, view: str) -> None:
        if self.viewport is None:
            return
        if view != "fit":
            # Vista estándar y encuadrada (como las caras del ViewCube de Inventor).
            self.viewport.set_view(view)
        pts = []
        if self.result is not None and self.model is not None:
            for seg in self.result.segments:
                for q in seg.samples[:: max(1, len(seg.samples) // 4)]:
                    m = self.model.fk(q)
                    pts.append(self._to_cell((m[0][3], m[1][3], m[2][3])))
        pts += self.scene_points()
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        self.viewport.fit(lo, hi)  # sin cambiar desde dónde se mira

    def scene_points(self) -> list[tuple[float, float, float]]:
        """Puntos que tienen que entrar en la vista: el robot en su pose actual
        (con todo su alcance alrededor de la base) y las esquinas de cada pieza."""
        pts = []
        if self.model is not None:
            robot = []      # en coordenadas de la base; después, a la celda
            r = self.model.reach_mm or (self.model.a1 + self.model.a2 + self.model.d4)
            top = self.model.d1 + self.model.a2 + self.model.a3
            robot += [(-r * 0.3, -r * 0.3, 0.0), (r * 0.3, r * 0.3, top)]
            frames = self.model.joint_frames(self.current_q)
            # Cada eje (punto de la posición cero) movido con su eslabón, y la brida.
            for (_axis, point), m in zip(self.model._screws(), frames):
                robot.append(tuple(sum(m[i][k] * point[k] for k in range(3)) + m[i][3]
                                   for i in range(3)))
            flange = self.model.fk(self.current_q)
            robot.append((flange[0][3], flange[1][3], flange[2][3]))
            pts += [self._to_cell(p) for p in robot]
        for obj in self.layout_data.objects:
            try:
                (x0, y0, z0), (x1, y1, z1) = self._mesh_for(obj.path).bounds()
            except MeshError:
                pts.append((obj.x, obj.y, obj.z))
                continue
            m = object_matrix(obj)
            for x in (x0, x1):
                for y in (y0, y1):
                    for z in (z0, z1):
                        pts.append(tuple(m[i][0] * x + m[i][1] * y + m[i][2] * z + m[i][3]
                                         for i in range(3)))
        return pts or [(0.0, 0.0, 0.0), (1000.0, 1000.0, 1000.0)]

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
        self._mark_collisions_stale("se agregó una pieza")
        self._report("info", f"Importado {Path(path).name} ({len(mesh)} triángulos).")
        self.objects_table.selectRow(len(self.layout_data.objects) - 1)  # lista para ubicar
        return obj

    def load_layout(self, path: str | Path) -> bool:
        try:
            layout = Layout.load(path)
        except Exception as e:  # noqa: BLE001 — JSON roto, claves de más, etc.
            self._report("error", f"No se pudo abrir la celda {Path(path).name}: {e}")
            return False
        self.layout_data = layout
        self.layout_path = Path(path)
        self._fetch_missing()
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
        self._show_collision_settings()
        self._show_robot_base()
        if self.viewport is not None and self.model is not None:
            self.viewport.set_robot(self._robot_meshes(self.model))  # con su herramienta
            self.show_pose(self.current_q)
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
            check = QTableWidgetItem()
            check.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled
                           | Qt.ItemFlag.ItemIsSelectable)
            check.setCheckState(Qt.CheckState.Checked if obj.workpiece else Qt.CheckState.Unchecked)
            check.setToolTip("La herramienta trabaja sobre esta pieza: para la herramienta solo "
                             "cuenta tocarla, no acercarse")
            self.objects_table.setItem(row, WORKPIECE_COLUMN, check)
        self._updating_table = False
        if hasattr(self, "place_ref"):
            self._fill_refs()
        if self.viewport is None:
            return
        if rebuild or self.viewport.object_count() != len(self.layout_data.objects):
            self.viewport.clear_objects()
            self._object_index: dict[int, int] = {}
            self._painted = {}
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
        if item.column() == WORKPIECE_COLUMN:
            obj.workpiece = item.checkState() == Qt.CheckState.Checked
            self._mark_collisions_stale("cambió qué pieza se trabaja")
            return
        field = ["name", "x", "y", "z", "rz"][item.column()]
        if field == "name":
            obj.name = item.text()
            return
        try:
            setattr(obj, field, _number(item.text()))
        except ValueError:
            self._report("warning", f"«{item.text()}» no es un número")
        self._refresh_objects(rebuild=False)
        self._mark_collisions_stale("se movió una pieza")

    # -- ubicar piezas -------------------------------------------------------------------------

    def selected_piece(self) -> int | None:
        rows = {i.row() for i in self.objects_table.selectedIndexes()}
        row = min(rows) if rows else self.objects_table.currentRow()
        return row if 0 <= row < len(self.layout_data.objects) else None

    def _on_place_mode(self, mode: int) -> None:
        self.place_stack.setCurrentIndex(mode)
        self.place_ref.setVisible(mode != 0)
        self._fill_refs()

    def _fill_refs(self) -> None:
        current = self.place_ref.currentData()
        sel = self.selected_piece()
        self.place_ref.blockSignals(True)
        self.place_ref.clear()
        if self.place_mode.currentIndex() == 1:
            self.place_ref.addItem("Base del robot", -1)
        for i, obj in enumerate(self.layout_data.objects):
            if i != sel:
                self.place_ref.addItem(f"«{obj.name}»", i)
        index = self.place_ref.findData(current)
        self.place_ref.setCurrentIndex(max(0, index))
        self.place_ref.blockSignals(False)

    def _on_piece_selected(self) -> None:
        sel = self.selected_piece()
        if sel is None:
            self.place_title.setText("<b>Ubicar</b>: elegí una pieza en la tabla.")
            self.place_info.setText("")
            return
        self.place_title.setText(f"<b>Ubicar «{self.layout_data.objects[sel].name}»</b>")
        self._fill_refs()
        self.place_info.setText(self._box_summary(sel))

    def _box(self, index: int) -> placement.Box:
        obj = self.layout_data.objects[index]
        return placement.world_box(obj, self._mesh_for(obj.path))

    def _box_summary(self, index: int) -> str:
        """Lo que se puede saber al instante, con las cajas."""
        try:
            box = self._box(index)
        except MeshError as e:
            return str(e)
        sx, sy, sz = box.size
        parts = [f"Tamaño {sx:.0f} × {sy:.0f} × {sz:.0f} mm",
                 f"apoyo en ({box.base[0]:.0f}, {box.base[1]:.0f}, {box.base[2]:.0f})",
                 f"al eje del robot: <b>"
                 f"{placement.axis_distance(box, self.layout_data.robot_base):.0f} mm</b>"]
        for i, other in enumerate(self.layout_data.objects):
            if i == index:
                continue
            try:
                g = placement.gaps(box, self._box(i))
            except MeshError:
                continue
            free = [f"{'XYZ'[k]} {g[k]:.0f}" for k in range(3) if g[k] >= 0]
            text = ", ".join(free) + " mm" if free else "las cajas se superponen"
            parts.append(f"a «{other.name}»: {text}")
        return " · ".join(parts)

    def apply_placement(self) -> bool:
        sel = self.selected_piece()
        if sel is None:
            self._report("warning", "Elegí en la tabla la pieza que querés ubicar.")
            return False
        obj = self.layout_data.objects[sel]
        mode = self.place_mode.currentIndex()
        try:
            mesh = self._mesh_for(obj.path)
            if mode == 0:
                xyz = placement.from_robot(obj, mesh, self.place_distance.value(),
                                           self.place_angle.value(),
                                           to_face=self.place_to.currentIndex() == 0,
                                           base=self.layout_data.robot_base)
            else:
                ref_index = self.place_ref.currentData()
                if ref_index is None:
                    self._report("warning", "No hay otra pieza para tomar de referencia.")
                    return False
                ref = (placement.robot_box(self.layout_data.robot_base) if ref_index == -1
                       else self._box(ref_index))
                if mode == 1:
                    xyz = placement.offset_from(obj, mesh, ref, self.place_dx.value(),
                                                self.place_dy.value(), self.place_dz.value())
                elif mode == 2:
                    xyz = placement.next_to(obj, mesh, ref, self.place_side.currentText(),
                                            self.place_gap.value())
                else:
                    xyz = placement.on_top(obj, mesh, ref, self.place_top_dx.value(),
                                           self.place_top_dy.value())
        except MeshError as e:
            self._report("error", str(e))
            return False
        obj.x, obj.y, obj.z = xyz
        self._refresh_objects(rebuild=False)
        self.objects_table.selectRow(sel)
        self._mark_collisions_stale("se movió una pieza")
        self.place_info.setText(self._box_summary(sel))
        return True

    def measure_selected(self) -> str:
        """Distancias mínimas reales (python-fcl) de la pieza elegida al robot
        en la pose que se ve y a las demás piezas."""
        sel = self.selected_piece()
        if sel is None:
            return ""
        if not collision.available():
            text = "Para medir entre superficies hace falta python-fcl. " + self._box_summary(sel)
            self.place_info.setText(text)
            return text
        obj = self.layout_data.objects[sel]
        try:
            mesh = self._mesh_for(obj.path)
        except MeshError as e:
            self._report("error", str(e))
            return ""
        parts = []
        if self.model is not None:
            checker = self._measure_checker()
            part, d = checker.robot_distance(
                self.current_q, collision.Obstacle(obj.name, mesh, object_matrix(obj)))
            parts.append(f"al robot (pose actual): <b>{d:.0f} mm</b> ({part})")
        for i, other in enumerate(self.layout_data.objects):
            if i == sel:
                continue
            try:
                d = placement.clearance(obj, mesh, other, self._mesh_for(other.path))
            except MeshError:
                continue
            parts.append(f"a «{other.name}»: <b>{d:.0f} mm</b>" if d else f"a «{other.name}»: se tocan")
        text = f"Distancia mínima entre superficies de «{obj.name}»: " + " · ".join(parts)
        self.place_info.setText(text)
        return text

    def _measure_checker(self):
        key = (self.model.name, self.layout_data.tool_mesh, tuple(self.layout_data.tool_mount),
               tuple(self.layout_data.robot_base))
        if self._measure_key != key:
            self._measure_checker_obj = collision.CollisionChecker(
                self.model, robot_link_meshes(self.model, tool_axis=False), [],
                tool_mesh=self._tool_mesh(), tool_mount=self.layout_data.tool_mount,
                margin_mm=0, floor=False, self_collision=False,
                base=self.layout_data.base_matrix())
            self._measure_key = key
        return self._measure_checker_obj

    # -- choques -----------------------------------------------------------------------------

    def _show_collision_settings(self) -> None:
        """Lleva a los controles lo que dice la celda (al abrir una)."""
        for widget in (self.collisions_check, self.margin_spin):
            widget.blockSignals(True)
        self.collisions_check.setChecked(self.layout_data.collisions and collision.available())
        self.margin_spin.setValue(self.layout_data.margin_mm)
        for widget in (self.collisions_check, self.margin_spin):
            widget.blockSignals(False)
        self._show_tool_mesh()

    def _show_tool_mesh(self) -> None:
        path = self.layout_data.tool_mesh
        self.tool_mesh_label.setText(
            Path(path).name if path else "Ninguna: se revisa hasta la brida. Cargá el STEP de la "
                                         "antorcha en coordenadas de la brida.")
        self.remove_tool_mesh_btn.setEnabled(bool(path))
        self.tool_mount_edit.setText(", ".join(_fmt(v) for v in self.layout_data.tool_mount))

    def _on_collision_settings(self, *_args) -> None:
        if collision.available():
            self.layout_data.collisions = self.collisions_check.isChecked()
        self.layout_data.margin_mm = float(self.margin_spin.value())
        self._mark_stale("cambió la búsqueda de choques")

    def _mark_collisions_stale(self, reason: str) -> None:
        if self.collisions_check.isChecked():
            self._mark_stale(reason)

    def set_tool_mesh(self, path: str | Path) -> bool:
        """Modelo 3D de la herramienta física, en coordenadas de la brida ("" = ninguna)."""
        path = str(path) if path else ""
        if path:
            try:
                self._mesh_for(path)
            except MeshError as e:
                self._report("error", f"Herramienta: {e}")
                return False
        self.layout_data.tool_mesh = path
        self.layout_data.tool_drive_id = ""
        self._show_tool_mesh()
        if self.viewport is not None and self.model is not None:
            self.viewport.set_robot(self._robot_meshes(self.model))
            self.show_pose(self.current_q)
        self._mark_collisions_stale("cambió la herramienta montada")
        return True

    def set_tool_mount(self, text: str) -> bool:
        try:
            mount = parse_pose(text)
        except ValueError as e:
            self._report("warning", f"Montaje de la herramienta: {e}")
            self._show_tool_mesh()
            return False
        if mount == self.layout_data.tool_mount:
            return True
        self.layout_data.tool_mount = mount
        self._show_tool_mesh()
        if self.layout_data.tool_mesh:
            if self.viewport is not None and self.model is not None:
                self.viewport.set_robot(self._robot_meshes(self.model))
                self.show_pose(self.current_q)
            self._mark_collisions_stale("cambió el montaje de la herramienta")
        return True

    def _on_tool_mount_edited(self) -> None:
        self.set_tool_mount(self.tool_mount_edit.text())

    def _on_choose_tool_mesh(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Modelo 3D de la herramienta (en coordenadas de la brida)",
            filter="Modelos 3D (*.step *.stp *.stl *.obj)")
        if path:
            self.set_tool_mesh(path)

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
            self._mark_collisions_stale("se quitó una pieza")

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
