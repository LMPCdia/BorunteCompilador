"""
Pestaña "Simulación 3D": simula sobre el modelo de robot elegido (`sim/`)
el programa del editor o un respaldo del pad (`HCBackupRobot_*.zip`), lo
anima, y arma la celda: piezas STEP/STL/OBJ, herramientas y sistemas de
coordenadas del pad. Todo eso se guarda junto en un `.layout.json`.

Todo menos la vista 3D funciona sin placa de video (y así se prueba). La vista
se crea solo si hay OpenGL: ver el aviso en gui/viewport3d.py.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Callable

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from compiler.codegen import CompileError
from compiler.pad_codegen import compile_to_pad, io_point
from pad.backup import PadBackup
from pad.listing import io_name
from sim.kinematics import RobotModel, rot_axis
from sim.meshes import Mesh, MeshError, load_mesh
from sim.pad_sim import PadSimulator, SimResult, inputs_used
from sim.scene import Layout, LayoutObject, Timeline, robot_link_meshes

SLIDER_STEPS = 1000
FRAME_MS = 33
OBJECT_COLUMNS = ["Objeto", "X", "Y", "Z", "Giro Z°"]
POSE_COLUMNS = ["N°", "X", "Y", "Z", "U", "V", "W"]
PATH_COLORS = {"MOVEJ": (0.35, 0.75, 1.0), "MOVEL": (1.0, 0.85, 0.2)}

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
    return float(text.strip().replace(",", "."))


class PoseTable(QWidget):
    """Tabla editable "número -> X, Y, Z, U, V, W" (herramientas o sistemas
    de coordenadas del pad)."""

    def __init__(self, title: str, help_text: str, on_change: Callable[[], None],
                 report: Reporter) -> None:
        super().__init__()
        self._on_change = on_change
        self._report = report
        self._loading = False
        self.table = QTableWidget(0, len(POSE_COLUMNS))
        self.table.setHorizontalHeaderLabels(POSE_COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.itemChanged.connect(self._edited)
        add_btn = QPushButton("Agregar")
        add_btn.clicked.connect(self.add_row)
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

    def values(self) -> dict[int, list[float]]:
        out = {}
        for r in range(self.table.rowCount()):
            try:
                number = int(_number(self.table.item(r, 0).text()))
                out[number] = [_number(self.table.item(r, c).text()) for c in range(1, 7)]
            except (ValueError, AttributeError):
                continue  # fila a medio escribir: se ignora hasta que esté completa
        return out

    def set_values(self, values: dict[int, list[float]]) -> None:
        self._loading = True
        self.table.setRowCount(0)
        for number, pose in sorted(values.items()):
            self._append([number, *pose])
        self._loading = False

    def add_row(self, number: int | None = None) -> None:
        used = self.values()
        if not isinstance(number, int):
            number = max(used, default=0) + 1
        self._loading = True
        self._append([number, 0, 0, 0, 0, 0, 0])
        self._loading = False
        self._on_change()

    def _append(self, row_values: list[float]) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        for col, value in enumerate(row_values):
            self.table.setItem(row, col, QTableWidgetItem(f"{value:g}"))

    def _remove(self) -> None:
        row = self.table.currentRow()
        if row >= 0:
            self.table.removeRow(row)
            self._on_change()

    def _edited(self, item: QTableWidgetItem) -> None:
        if self._loading:
            return
        try:
            _number(item.text())
        except ValueError:
            self._report("warning", f"«{item.text()}» no es un número")
            return
        self._on_change()


class SimView(QWidget):
    def __init__(self, get_source: Callable[[], str], report: Reporter | None = None,
                 enable_3d: bool | None = None) -> None:
        super().__init__()
        self._get_source = get_source
        self._report = report or (lambda _severity, _msg: None)
        self.layout_data = Layout()
        self.layout_path: Path | None = None
        self.backup: PadBackup | None = None      # respaldo abierto (None = editor)
        self.backup_path: Path | None = None
        self.result: SimResult | None = None
        self.timeline: Timeline | None = None
        self.current_q: list[float] = [0.0] * 6
        self._input_boxes: dict[int, QCheckBox] = {}
        self._mesh_cache: dict[str, Mesh] = {}
        self._t = 0.0
        self._updating_table = False

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
        self.set_model(self.model_combo.currentText())

    # -- construcción ----------------------------------------------------------------

    def _build(self) -> None:
        self.model_combo = QComboBox()
        self.model_combo.addItems(RobotModel.available())
        self.model_combo.currentTextChanged.connect(self.set_model)
        self.source_label = QLabel("Programa: el del editor")
        open_backup_btn = QPushButton("Abrir respaldo del pad…")
        open_backup_btn.clicked.connect(self._on_open_backup)
        self.use_editor_btn = QPushButton("Usar el editor")
        self.use_editor_btn.clicked.connect(self.use_editor)
        self.use_editor_btn.setEnabled(False)
        self.simulate_btn = QPushButton("Simular")
        self.simulate_btn.clicked.connect(self.simulate)

        top = QHBoxLayout()
        top.addWidget(QLabel("Robot:"))
        top.addWidget(self.model_combo)
        top.addWidget(self.source_label, 1)
        top.addWidget(open_backup_btn)
        top.addWidget(self.use_editor_btn)
        top.addWidget(self.simulate_btn)

        self.inputs_row = QHBoxLayout()
        self.inputs_hint = QLabel("Entradas: (el programa no consulta ninguna)")
        self.inputs_row.addWidget(self.inputs_hint)
        self.inputs_row.addStretch(1)

        self.play_btn = QPushButton("▶")
        self.play_btn.setCheckable(True)
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

        if self.viewport is not None:
            view = self.viewport.widget
        else:
            view = QLabel(
                "La vista 3D necesita OpenGL y no está disponible en esta PC.\n"
                "La simulación, los avisos y el layout funcionan igual."
            )
            view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.summary = QLabel("Apretá «Simular» para simular el programa del editor.")
        self.summary.setWordWrap(True)

        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.addWidget(view, 1)
        center_layout.addLayout(bottom)
        center_layout.addWidget(self.summary)

        splitter = QSplitter()
        splitter.addWidget(center)
        splitter.addWidget(self._build_side())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1000, 460])

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(self.inputs_row)
        layout.addWidget(splitter, 1)

    def _build_side(self) -> QWidget:
        # -- piezas --
        self.objects_table = QTableWidget(0, len(OBJECT_COLUMNS))
        self.objects_table.setHorizontalHeaderLabels(OBJECT_COLUMNS)
        header = self.objects_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for col in range(1, len(OBJECT_COLUMNS)):
            header.setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)
        self.objects_table.verticalHeader().setVisible(False)
        self.objects_table.itemChanged.connect(self._on_object_edited)
        import_btn = QPushButton("Importar objeto…")
        import_btn.clicked.connect(self._on_import)
        remove_btn = QPushButton("Quitar")
        remove_btn.clicked.connect(self._on_remove)
        objects = QWidget()
        objects_layout = QVBoxLayout(objects)
        objects_layout.addWidget(QLabel("Piezas (mm, respecto de la base del robot)"))
        objects_layout.addWidget(self.objects_table, 1)
        row = QHBoxLayout()
        row.addWidget(import_btn)
        row.addWidget(remove_btn)
        objects_layout.addLayout(row)

        # -- herramientas y coordenadas --
        self.tools_table = PoseTable(
            "Herramientas del pad (punta respecto de la brida)",
            "Copiá los valores del menú de herramientas del pad. La 0 es la brida y no "
            "hace falta cargarla.",
            self._on_frames_changed, self._report)
        self.frames_table = PoseTable(
            "Sistemas de coordenadas del pad (respecto de la base)",
            "Copiá los valores del menú de coordenadas del pad. El 0 es la base del "
            "robot y no hace falta cargarlo.",
            self._on_frames_changed, self._report)

        tabs = QTabWidget()
        tabs.addTab(objects, "Piezas")
        tabs.addTab(self.tools_table, "Herramientas")
        tabs.addTab(self.frames_table, "Coordenadas")
        self.side_tabs = tabs

        open_btn = QPushButton("Abrir celda…")
        open_btn.clicked.connect(self._on_open_layout)
        save_btn = QPushButton("Guardar celda…")
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
        self._report("info", f"Abierto el respaldo {Path(path).name}: programa {self.backup.name}, "
                             f"{len(self.backup.act.modules)} módulo(s).")
        return True

    def use_editor(self) -> None:
        self.backup = None
        self.backup_path = None
        self.source_label.setText("Programa: el del editor")
        self.use_editor_btn.setEnabled(False)

    def _current_backup(self) -> PadBackup:
        if self.backup is not None:
            return self.backup
        backup = compile_to_pad(self._get_source())
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
        if self.result is not None:
            self.simulate()

    # -- modelo y simulación ------------------------------------------------------------

    def set_model(self, name: str) -> None:
        self.model = RobotModel.load(name)
        self.layout_data.model = name
        if self.viewport is not None:
            self.viewport.set_robot(robot_link_meshes(self.model))
        self.show_pose(self.current_q)

    def simulate(self) -> SimResult | None:
        self.play_btn.setChecked(False)
        try:
            backup = self._current_backup()
        except CompileError as e:
            self._report("error", f"No se puede simular: {e}")
            self.summary.setText(f"No compila para el pad: {e}")
            return None
        except Exception as e:  # noqa: BLE001 — errores de parseo de Lark
            self._report("error", f"No se puede simular: {e}")
            self.summary.setText(f"No compila: {e}")
            return None

        QGuiApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            simulator = PadSimulator(self.model, self.inputs(),
                                     tools=self.layout_data.tools, frames=self.layout_data.frames)
            self.result = simulator.run(backup)
        finally:
            QGuiApplication.restoreOverrideCursor()
        self.timeline = Timeline(self.result)
        self._draw_path(simulator)

        for issue in self.result.issues:
            severity = "error" if issue.severity == "error" else "warning"
            self._report(severity, f"[{self.model.name}] {issue.where}: {issue.message}")
        errores = sum(i.severity == "error" for i in self.result.issues)
        avisos = len(self.result.issues) - errores
        moves = sum(s.kind in ("MOVEJ", "MOVEL") for s in self.result.segments)
        self.summary.setText(
            f"{self.model.name}: {moves} movimiento(s) simulado(s), tiempo de ciclo estimado "
            f"{self.result.total_time_s:.1f} s (sin aceleraciones). {errores} error(es), "
            f"{avisos} aviso(s) — detalle en la ventana de mensajes. Modelo con hipótesis: "
            f"ver docs/SIMULATOR.md."
        )
        self._report("info", f"Simulación en {self.model.name}: {moves} movimiento(s), "
                             f"{self.result.total_time_s:.1f} s, {errores} error(es), {avisos} aviso(s).")
        self.set_time(0.0)
        return self.result

    def _draw_path(self, simulator: PadSimulator) -> None:
        if self.viewport is None or self.result is None:
            return
        points, colors = [], []
        for seg in self.result.segments:
            if seg.kind not in PATH_COLORS:
                continue
            for q in seg.samples:
                points.append(simulator.tcp(q, seg.tool))
                colors.append(PATH_COLORS[seg.kind])
        self.viewport.set_path(points, colors)

    # -- herramientas y coordenadas -------------------------------------------------------

    def _on_frames_changed(self) -> None:
        self.layout_data.tools = self.tools_table.values()
        self.layout_data.frames = self.frames_table.values()

    def set_tool(self, number: int, pose: list[float]) -> None:
        self.layout_data.tools[number] = list(pose)
        self.tools_table.set_values(self.layout_data.tools)

    def set_frame(self, number: int, pose: list[float]) -> None:
        self.layout_data.frames[number] = list(pose)
        self.frames_table.set_values(self.layout_data.frames)

    # -- animación ---------------------------------------------------------------------

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
        if self.viewport is not None:
            self.viewport.set_joint_frames(self.model.joint_frames(q))

    def _on_slider(self, value: int) -> None:
        if self.timeline is not None:
            self.set_time(value / SLIDER_STEPS * self.timeline.duration)

    def _on_play_toggled(self, playing: bool) -> None:
        self.play_btn.setText("❚❚" if playing else "▶")
        if playing and self.timeline is not None:
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
            QGuiApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            try:
                self._mesh_cache[path] = load_mesh(path)
            finally:
                QGuiApplication.restoreOverrideCursor()
        return self._mesh_cache[path]

    def import_object(self, path: str | Path) -> LayoutObject | None:
        path = str(path)
        try:
            mesh = self._mesh_for(path)
        except MeshError as e:
            self._report("error", str(e))
            return None
        obj = LayoutObject(name=Path(path).stem, path=path)
        # Apoyado en el piso: que lo más bajo de la pieza quede en z = 0.
        (_, _, zmin), _ = mesh.bounds()
        obj.z = -zmin
        self.layout_data.objects.append(obj)
        self._refresh_objects()
        self._report("info", f"Importado {Path(path).name} ({len(mesh)} triángulos).")
        return obj

    def load_layout(self, path: str | Path) -> None:
        layout = Layout.load(path)
        self.layout_data = layout
        self.layout_path = Path(path)
        if layout.model in RobotModel.available():
            self.model_combo.setCurrentText(layout.model)
        else:
            self._report("warning", f"La celda usa el modelo {layout.model}, que no está "
                                    f"instalado: se usa {self.model_combo.currentText()}.")
            self.layout_data.model = self.model_combo.currentText()
        self.tools_table.set_values(layout.tools)
        self.frames_table.set_values(layout.frames)
        self._refresh_objects()

    def save_layout(self, path: str | Path) -> None:
        self.layout_data.tools = self.tools_table.values()
        self.layout_data.frames = self.frames_table.values()
        self.layout_data.save(path)
        self.layout_path = Path(path)

    def _refresh_objects(self) -> None:
        self._updating_table = True
        self.objects_table.setRowCount(len(self.layout_data.objects))
        for row, obj in enumerate(self.layout_data.objects):
            values = [obj.name, f"{obj.x:g}", f"{obj.y:g}", f"{obj.z:g}", f"{obj.rz:g}"]
            for col, value in enumerate(values):
                self.objects_table.setItem(row, col, QTableWidgetItem(value))
        self._updating_table = False
        if self.viewport is not None:
            self.viewport.clear_objects()
            for obj in self.layout_data.objects:
                try:
                    index = self.viewport.add_object(self._mesh_for(obj.path), obj.color)
                except MeshError as e:
                    self._report("error", str(e))
                    continue
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
        self._refresh_objects()

    # -- diálogos -------------------------------------------------------------------------

    def _on_open_backup(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Abrir respaldo del pad", filter="Respaldo del pad (*.zip)")
        if path and self.open_backup(path):
            self.simulate()

    def _on_import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Importar objeto 3D", filter="Modelos 3D (*.step *.stp *.stl *.obj)")
        if path:
            self.import_object(path)

    def _on_remove(self) -> None:
        row = self.objects_table.currentRow()
        if 0 <= row < len(self.layout_data.objects):
            del self.layout_data.objects[row]
            self._refresh_objects()

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
