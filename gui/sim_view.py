"""
Pestaña "Simulación 3D": compila el programa para el pad, lo simula sobre el
modelo de robot elegido (`sim/`), lo anima, y permite armar el layout de la
celda importando piezas STEP/STL/OBJ.

Todo menos la vista 3D funciona sin placa de video (y así se prueba). La vista
se crea solo si hay OpenGL: ver el aviso en gui/viewport3d.py.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Callable

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from compiler.codegen import CompileError
from compiler.pad_codegen import compile_to_pad, io_point
from sim.kinematics import RobotModel, rot_axis
from sim.meshes import Mesh, MeshError, load_mesh
from sim.pad_sim import SimResult, simulate
from sim.scene import Layout, LayoutObject, Timeline, robot_link_meshes

SLIDER_STEPS = 1000
FRAME_MS = 33
OBJECT_COLUMNS = ["Objeto", "X", "Y", "Z", "Giro Z°"]

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


class SimView(QWidget):
    def __init__(self, get_source: Callable[[], str], report: Reporter | None = None,
                 enable_3d: bool | None = None) -> None:
        super().__init__()
        self._get_source = get_source
        self._report = report or (lambda _severity, _msg: None)
        self.layout_data = Layout()
        self.result: SimResult | None = None
        self.timeline: Timeline | None = None
        self.current_q: list[float] = [0.0] * 6
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
        self.inputs_edit = QLineEdit()
        self.inputs_edit.setPlaceholderText("Entradas activas, ej.: X012=1, X013=0")
        self.simulate_btn = QPushButton("Simular")
        self.simulate_btn.clicked.connect(self.simulate)
        self.play_btn = QPushButton("▶")
        self.play_btn.setCheckable(True)
        self.play_btn.toggled.connect(self._on_play_toggled)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.slider.valueChanged.connect(self._on_slider)
        self.time_label = QLabel("—")

        top = QHBoxLayout()
        top.addWidget(QLabel("Robot:"))
        top.addWidget(self.model_combo)
        top.addWidget(self.inputs_edit, 1)
        top.addWidget(self.simulate_btn)
        bottom = QHBoxLayout()
        bottom.addWidget(self.play_btn)
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
        open_btn = QPushButton("Abrir layout…")
        open_btn.clicked.connect(self._on_open_layout)
        save_btn = QPushButton("Guardar layout…")
        save_btn.clicked.connect(self._on_save_layout)
        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.addWidget(QLabel("Layout de la celda (mm, respecto de la base del robot)"))
        side_layout.addWidget(self.objects_table, 1)
        row1, row2 = QHBoxLayout(), QHBoxLayout()
        row1.addWidget(import_btn)
        row1.addWidget(remove_btn)
        row2.addWidget(open_btn)
        row2.addWidget(save_btn)
        side_layout.addLayout(row1)
        side_layout.addLayout(row2)

        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.addWidget(view, 1)
        center_layout.addLayout(bottom)
        center_layout.addWidget(self.summary)

        splitter = QSplitter()
        splitter.addWidget(center)
        splitter.addWidget(side)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1000, 420])

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(splitter, 1)

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
            backup = compile_to_pad(self._get_source())
            inputs = parse_inputs(self.inputs_edit.text())
        except CompileError as e:
            self._report("error", f"No se puede simular: {e}")
            self.summary.setText(f"No compila para el pad: {e}")
            return None
        except Exception as e:  # noqa: BLE001 — errores de parseo de Lark
            self._report("error", f"No se puede simular: {e}")
            self.summary.setText(f"No compila: {e}")
            return None

        self.result = simulate(backup, self.model, inputs)
        self.timeline = Timeline(self.result)
        for issue in self.result.issues:
            severity = "error" if issue.severity == "error" else "warning"
            self._report(severity, f"[{self.model.name}] {issue.where}: {issue.message}")
        errores = sum(i.severity == "error" for i in self.result.issues)
        avisos = len(self.result.issues) - errores
        self.summary.setText(
            f"{self.model.name}: tiempo de ciclo estimado {self.result.total_time_s:.1f} s "
            f"(sin aceleraciones). {errores} error(es), {avisos} aviso(s). "
            f"Modelo con hipótesis: ver docs/SIMULATOR.md."
        )
        self._report("info", f"Simulación en {self.model.name}: "
                             f"{self.result.total_time_s:.1f} s, {errores} error(es), {avisos} aviso(s).")
        self.set_time(0.0)
        return self.result

    # -- animación ---------------------------------------------------------------------

    def set_time(self, t: float) -> None:
        if self.timeline is None:
            return
        self._t = max(0.0, min(t, self.timeline.duration))
        q, where = self.timeline.at(self._t)
        self.show_pose(q)
        self.time_label.setText(f"{self._t:5.1f} / {self.timeline.duration:.1f} s  {where}")
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
        self.set_time(self._t + FRAME_MS / 1000)
        if self._t >= self.timeline.duration:
            self.play_btn.setChecked(False)

    # -- layout ---------------------------------------------------------------------------

    def _mesh_for(self, path: str) -> Mesh:
        if path not in self._mesh_cache:
            self._mesh_cache[path] = load_mesh(path)
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
        if layout.model in RobotModel.available():
            self.model_combo.setCurrentText(layout.model)
        self._refresh_objects()

    def save_layout(self, path: str | Path) -> None:
        self.layout_data.save(path)

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
            setattr(obj, field, float(item.text().replace(",", ".")))
        except ValueError:
            self._report("warning", f"«{item.text()}» no es un número")
        self._refresh_objects()

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
        path, _ = QFileDialog.getOpenFileName(self, "Abrir layout", filter="Layout (*.layout.json *.json)")
        if path:
            self.load_layout(path)

    def _on_save_layout(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Guardar layout", filter="Layout (*.layout.json)")
        if path:
            self.save_layout(path)
