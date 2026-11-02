"""
Pestaña "Fresado" del simulador: elegir la pieza, delimitar el área en 2D y
generar el programa. El cálculo vive en sim/cam.py; acá solo la interfaz.
Ver docs/FRESADO.md.
"""

from __future__ import annotations

from typing import Callable

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, QThread, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from compiler.program_files import ProgramSources
from sim import cam
from sim.cam import CamError, CamParams, CutterTool, HeightMap, Polygon, RobotPath, ToolPath
from sim.kinematics import RobotModel
from sim.meshes import Mesh, MeshError
from sim.scene import Layout, LayoutObject, object_matrix

CANVAS_CELLS = 500         # el dibujo no necesita más detalle que la pantalla
PATH_COLORS = {"rapid": (0.25, 0.55, 1.0), "plunge": (1.0, 0.85, 0.1), "cut": (1.0, 0.35, 0.1)}


class AreaCanvas(QWidget):
    """La pieza vista en planta. Clic izquierdo agrega un vértice; clic derecho
    (o doble clic) cierra el área. Lo más claro es lo más alto."""

    changed = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(280, 340)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setCursor(Qt.CursorShape.CrossCursor)
        self.hm: HeightMap | None = None
        self.polygons: list[Polygon] = []
        self.draft: Polygon = []
        self.path: ToolPath | None = None
        self._image: QImage | None = None

    def set_map(self, hm: HeightMap | None) -> None:
        self.hm = hm
        self._image = None
        if hm is not None:
            solid = np.isfinite(hm.h)
            gray = np.full(hm.h.shape, 25, dtype=np.uint8)
            if solid.any():
                lo, hi = hm.h[solid].min(), hm.h[solid].max()
                gray[solid] = (70 + 170 * (hm.h[solid] - lo) / max(hi - lo, 1e-9)).astype(np.uint8)
            flipped = np.ascontiguousarray(gray[::-1])   # v crece hacia arriba
            self._image = QImage(flipped.data, flipped.shape[1], flipped.shape[0],
                                 flipped.shape[1], QImage.Format.Format_Grayscale8).copy()
        self.update()

    def _rect(self) -> QRectF:
        """Dónde cae el mapa en el widget, sin deformarlo."""
        assert self.hm is not None
        ny, nx = self.hm.h.shape
        scale = min((self.width() - 16) / max(nx, 1), (self.height() - 16) / max(ny, 1))
        w, h = nx * scale, ny * scale
        return QRectF((self.width() - w) / 2, (self.height() - h) / 2, w, h)

    def to_screen(self, u: float, v: float) -> QPointF:
        rect, (u0, v0), (u1, v1) = self._rect(), self.hm.lo, self.hm.hi()
        return QPointF(rect.left() + (u - u0) / max(u1 - u0, 1e-9) * rect.width(),
                       rect.bottom() - (v - v0) / max(v1 - v0, 1e-9) * rect.height())

    def to_plane(self, x: float, y: float) -> tuple[float, float]:
        rect, (u0, v0), (u1, v1) = self._rect(), self.hm.lo, self.hm.hi()
        return (u0 + (x - rect.left()) / rect.width() * (u1 - u0),
                v0 + (rect.bottom() - y) / rect.height() * (v1 - v0))

    def close_draft(self) -> None:
        if len(self.draft) >= 3:
            self.polygons.append(self.draft)
            self.draft = []
            self.path = None
            self.changed.emit()
            self.update()

    def undo(self) -> None:
        if self.draft:
            self.draft.pop()
        elif self.polygons:
            self.polygons.pop()
        self.path = None
        self.changed.emit()
        self.update()

    def clear(self) -> None:
        self.polygons, self.draft, self.path = [], [], None
        self.changed.emit()
        self.update()

    def whole_piece(self) -> None:
        """El rectángulo que encierra la pieza."""
        if self.hm is None:
            return
        rows, cols = np.nonzero(np.isfinite(self.hm.h))
        if len(rows) == 0:
            return
        res, (u0, v0) = self.hm.res, self.hm.lo
        a, b = u0 + cols.min() * res, u0 + cols.max() * res
        c, d = v0 + rows.min() * res, v0 + rows.max() * res
        self.polygons, self.draft, self.path = [[(a, c), (b, c), (b, d), (a, d)]], [], None
        self.changed.emit()
        self.update()

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if self.hm is None:
            return
        if event.button() == Qt.MouseButton.RightButton:
            self.close_draft()
        elif event.button() == Qt.MouseButton.LeftButton:
            p = event.position()
            if self._rect().contains(p):
                self.draft.append(self.to_plane(p.x(), p.y()))
                self.update()

    def mouseDoubleClickEvent(self, _event) -> None:  # noqa: N802
        self.close_draft()

    def paintEvent(self, _event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#191c20"))
        if self.hm is None or self._image is None:
            painter.setPen(QColor("#9aa4ad"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Elegí una pieza de la celda")
            return
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.drawImage(self._rect(), self._image)
        if self.path is not None:
            for start, end, kind in zip(self.path.pos[:-1], self.path.pos[1:], self.path.kinds[1:]):
                r, g, b = PATH_COLORS[kind]
                pen = QPen(QColor.fromRgbF(r, g, b), 1)
                if kind == "rapid":
                    pen.setStyle(Qt.PenStyle.DotLine)
                painter.setPen(pen)
                painter.drawLine(self.to_screen(start[0], start[1]), self.to_screen(end[0], end[1]))
        painter.setBrush(QColor(255, 180, 40, 45))
        painter.setPen(QPen(QColor("#ffb428"), 2))
        for poly in self.polygons:
            painter.drawPolygon(QPolygonF([self.to_screen(*p) for p in poly]))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor("#3fd0ff"), 2))
        if self.draft:
            painter.drawPolyline(QPolygonF([self.to_screen(*p) for p in self.draft]))
            for p in self.draft:
                painter.drawEllipse(self.to_screen(*p), 3, 3)


class _Worker(QThread):
    """El cálculo tarda segundos: fuera del hilo de la interfaz."""

    message = Signal(str)
    finished_ok = Signal(object)
    failed = Signal(str)

    def __init__(self, job: Callable[[Callable[[str], None]], object]) -> None:
        super().__init__()
        self.job = job
        self.stop = False

    def _say(self, text: str) -> None:
        if self.stop:
            raise cam.Cancelled
        self.message.emit(text)

    def run(self) -> None:
        try:
            self.finished_ok.emit(self.job(self._say))
        except cam.Cancelled:
            pass
        except CamError as e:
            self.failed.emit(str(e))
        except Exception as e:  # noqa: BLE001 — que el error se vea, no que muera el hilo
            self.failed.emit(f"Error inesperado: {e}")


class CamPanel(QWidget):
    def __init__(self, layout: Callable[[], Layout], model: Callable[[], RobotModel | None],
                 mesh_for: Callable[[str], Mesh],
                 on_program: Callable[[ProgramSources], bool],
                 on_preview: Callable[[list, list], None],
                 report: Callable[[str, str], None]) -> None:
        super().__init__()
        self._layout, self._model, self._mesh_for = layout, model, mesh_for
        self._on_program, self._on_preview, self._report = on_program, on_preview, report
        self._worker: _Worker | None = None
        self._map_key = None
        self.robot_path: RobotPath | None = None
        self._job_params: tuple[CutterTool, CamParams] | None = None
        self._job_object: LayoutObject | None = None
        self._build()

    # -- interfaz ---------------------------------------------------------------------------

    @staticmethod
    def _spin(value: float, low: float, high: float, step: float = 1.0, decimals: int = 2,
              suffix: str = " mm") -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    def _build(self) -> None:
        root = QVBoxLayout(self)
        form = QFormLayout()
        self.piece_combo = QComboBox()
        self.piece_combo.currentIndexChanged.connect(self._selection_changed)
        self.view_combo = QComboBox()
        self.view_combo.addItems(cam.VIEWS)
        self.view_combo.setToolTip("Desde dónde se mira la pieza: el área se dibuja en ese plano "
                                   "y se proyecta sobre la superficie en esa dirección")
        self.view_combo.currentIndexChanged.connect(self._selection_changed)
        form.addRow("Pieza", self.piece_combo)
        form.addRow("Vista", self.view_combo)
        root.addLayout(form)

        self.canvas = AreaCanvas()
        self.canvas.changed.connect(self._area_changed)
        root.addWidget(self.canvas, 1)
        hint = QLabel("Clic: agrega un punto. Clic derecho o doble clic: cierra el área.")
        hint.setWordWrap(True)
        root.addWidget(hint)
        row = QHBoxLayout()
        whole = QPushButton("Toda la pieza")
        whole.clicked.connect(self.canvas.whole_piece)
        undo = QPushButton("Deshacer")
        undo.clicked.connect(self.canvas.undo)
        clear = QPushButton("Borrar áreas")
        clear.clicked.connect(self.canvas.clear)
        for button in (whole, undo, clear):
            row.addWidget(button)
        root.addLayout(row)

        tool_box = QGroupBox("Fresa")
        tool_form = QFormLayout(tool_box)
        self.kind_combo = QComboBox()
        self.kind_combo.addItems(["bola", "plana"])
        self.diameter = self._spin(10, 0.5, 200)
        self.cut_length = self._spin(40, 0, 500)
        self.holder = self._spin(0, 0, 500)
        self.holder.setToolTip("Diámetro de la porta o del husillo, por encima del filo. 0 = no se revisa")
        self.tool_number = QSpinBox()
        self.tool_number.setRange(0, 15)
        self.tool_number.setToolTip("Herramienta del pad, cargada en la pestaña Herramientas: "
                                    "X, Y, Z hasta la punta de la fresa y +Z por el eje de la fresa")
        tool_form.addRow("Tipo", self.kind_combo)
        tool_form.addRow("Diámetro", self.diameter)
        tool_form.addRow("Largo de corte", self.cut_length)
        tool_form.addRow("Diámetro porta", self.holder)
        tool_form.addRow("N° de herramienta", self.tool_number)
        root.addWidget(tool_box)

        cut_box = QGroupBox("Corte")
        cut_form = QFormLayout(cut_box)
        self.strategy = QComboBox()
        self.strategy.addItem("Zigzag (llena el área)", "zigzag")
        self.strategy.addItem("Contorno, por dentro", ("contorno", "interior"))
        self.strategy.addItem("Contorno, por fuera", ("contorno", "exterior"))
        self.strategy.addItem("Contorno, sobre la línea", ("contorno", "sobre"))
        self.stepover = self._spin(3, 0.1, 200)
        self.angle = self._spin(0, -180, 180, 15, 1, " °")
        self.depth = self._spin(0, 0, 500)
        self.depth.setToolTip("Debajo de la superficie, midiendo sobre ella. 0 = copiarla")
        self.stepdown = self._spin(2, 0.05, 100)
        self.axis_combo = QComboBox()
        self.axis_combo.addItem("Fijo, como una CNC de 3 ejes", "fijo")
        self.axis_combo.addItem("Normal a la superficie (solo bola)", "normal")
        self.tilt = self._spin(30, 0, 60, 5, 0, " °")
        self.safe = self._spin(30, 1, 1000)
        cut_form.addRow("Estrategia", self.strategy)
        cut_form.addRow("Entre pasadas", self.stepover)
        cut_form.addRow("Dirección", self.angle)
        cut_form.addRow("Profundidad", self.depth)
        cut_form.addRow("Por nivel", self.stepdown)
        cut_form.addRow("Eje de la fresa", self.axis_combo)
        cut_form.addRow("Inclinación máx.", self.tilt)
        cut_form.addRow("Altura segura", self.safe)
        root.addWidget(cut_box)

        run_box = QGroupBox("Velocidades y husillo")
        run_form = QFormLayout(run_box)
        self.cut_speed = self._spin(2, 0.1, 100, 0.5, 1, " %")
        self.plunge_speed = self._spin(1, 0.1, 100, 0.5, 1, " %")
        self.rapid_speed = self._spin(10, 0.1, 100, 1, 1, " %")
        self.spindle = QLineEdit()
        self.spindle.setPlaceholderText("Y034 (vacío: a mano)")
        self.spindle_wait = self._spin(3, 0, 60, 1, 1, " s")
        run_form.addRow("Corte (SPEED)", self.cut_speed)
        run_form.addRow("Entrada (SPEED)", self.plunge_speed)
        run_form.addRow("Aire (SPEED)", self.rapid_speed)
        run_form.addRow("Salida del husillo", self.spindle)
        run_form.addRow("Espera al arrancar", self.spindle_wait)
        root.addWidget(run_box)

        self.calc_btn = QPushButton("Calcular trayectoria")
        self.calc_btn.clicked.connect(lambda: self.calculate())
        self.code_btn = QPushButton("Generar código")
        self.code_btn.setEnabled(False)
        self.code_btn.clicked.connect(self.generate)
        row = QHBoxLayout()
        row.addWidget(self.calc_btn)
        row.addWidget(self.code_btn)
        root.addLayout(row)
        self.status = QLabel("Elegí una pieza y delimitá el área.")
        self.status.setWordWrap(True)
        root.addWidget(self.status)

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        self.refresh()

    # -- pieza y vista ----------------------------------------------------------------------

    def _object(self) -> LayoutObject | None:
        index = self.piece_combo.currentIndex()
        objects = self._layout().objects
        return objects[index] if 0 <= index < len(objects) else None

    def refresh(self) -> None:
        """La celda pudo cambiar: otras piezas, o la pieza en otro lugar."""
        names = [obj.name for obj in self._layout().objects]
        if names != [self.piece_combo.itemText(i) for i in range(self.piece_combo.count())]:
            current = self.piece_combo.currentText()
            self.piece_combo.blockSignals(True)
            self.piece_combo.clear()
            self.piece_combo.addItems(names)
            if current in names:
                self.piece_combo.setCurrentText(current)
            self.piece_combo.blockSignals(False)
        self._update_map()

    def _selection_changed(self) -> None:
        self.canvas.clear()     # el área se dibujó en otro plano
        self._update_map()

    def _update_map(self) -> None:
        obj = self._object()
        if obj is None:
            self._map_key = None
            self.canvas.set_map(None)
            return
        matrix = object_matrix(obj)
        key = (obj.path, str(matrix), self.view_combo.currentText())
        if key == self._map_key:
            return
        try:
            mesh = self._mesh_for(obj.path)
            size = max(a - b for a, b in zip(*mesh.bounds()))
            self.canvas.set_map(cam.build_heightmap(mesh, matrix, self.view_combo.currentText(),
                                                    res=size / CANVAS_CELLS))
            self._map_key = key
        except (MeshError, CamError) as e:
            self._report("error", f"Fresado: {e}")
            self.canvas.set_map(None)

    def _area_changed(self) -> None:
        self.robot_path = None
        self.code_btn.setEnabled(False)

    # -- cálculo ----------------------------------------------------------------------------

    def tool(self) -> CutterTool:
        return CutterTool(self.kind_combo.currentText(), self.diameter.value(),
                          self.cut_length.value(), self.holder.value())

    def params(self) -> CamParams:
        data = self.strategy.currentData()
        strategy, side = (data, "interior") if data == "zigzag" else data
        return CamParams(
            strategy=strategy, side=side, stepover=self.stepover.value(), depth=self.depth.value(),
            stepdown=self.stepdown.value(), angle_deg=self.angle.value(),
            axis_mode=self.axis_combo.currentData(), max_tilt_deg=self.tilt.value(),
            safe_mm=self.safe.value(), cut_speed=self.cut_speed.value(),
            plunge_speed=self.plunge_speed.value(), rapid_speed=self.rapid_speed.value(),
            spindle_output=self.spindle.text().strip().upper(),
            spindle_wait_s=self.spindle_wait.value(), tool_number=self.tool_number.value())

    def _prepare(self) -> Callable[[Callable[[str], None]], RobotPath]:
        """Junta los datos en el hilo de la interfaz y devuelve el trabajo."""
        self.canvas.close_draft()
        obj = self._object()
        model = self._model()
        if obj is None:
            raise CamError("Falta una pieza: importala en la pestaña Celda")
        if model is None:
            raise CamError("Falta elegir el robot")
        cell = self._layout()
        tool, params = self.tool(), self.params()
        number = params.tool_number
        if number and number not in cell.tools:
            raise CamError(f"La herramienta {number} no está cargada en la pestaña Herramientas")
        tool_pose = cell.tools.get(number, [0.0] * 6)
        frame_pose = cell.frames.get(params.coord_number, [0.0] * 6)
        mesh, matrix = self._mesh_for(obj.path), object_matrix(obj)
        view, polygons, base = self.view_combo.currentText(), list(self.canvas.polygons), list(cell.robot_base)
        self._job_params, self._job_object = (tool, params), obj

        def job(say: Callable[[str], None]) -> RobotPath:
            say("Leyendo la superficie…")
            hm = cam.build_heightmap(mesh, matrix, view, params.res_mm or cam.auto_resolution(tool))
            path = cam.compute_path(hm, polygons, tool, params, say)
            return cam.solve_robot(path, model, base, tool_pose, frame_pose, params.roll_deg, say)

        return job

    def calculate(self, synchronous: bool = False) -> None:
        if self._worker is not None:
            return
        self.robot_path = None
        self.code_btn.setEnabled(False)
        try:
            job = self._prepare()
        except (CamError, MeshError) as e:
            self._failed(str(e))
            return
        if synchronous:
            try:
                self._finished(job(lambda _text: None))
            except CamError as e:
                self._failed(str(e))
            return
        self.calc_btn.setEnabled(False)
        worker = self._worker = _Worker(job)
        worker.message.connect(self.status.setText)
        worker.finished_ok.connect(self._finished)
        worker.failed.connect(self._failed)
        worker.finished.connect(self._thread_done)
        worker.start()

    def _thread_done(self) -> None:
        worker, self._worker = self._worker, None
        if worker is not None:
            worker.wait()      # recién ahora se puede soltar la referencia al QThread
        self.calc_btn.setEnabled(True)

    def shutdown(self) -> None:
        if self._worker is not None:
            self._worker.stop = True
            self._worker.wait()

    def _failed(self, text: str) -> None:
        self.status.setText(text)
        self._report("error", f"Fresado: {text}")

    def _finished(self, robot: RobotPath) -> None:
        self.robot_path = robot
        path = robot.path
        self.canvas.path = path
        self.canvas.update()
        self._preview(robot)
        for warning in path.warnings + robot.warnings:
            self._report("warning", f"Fresado: {warning}")
        if self._job_params and not self._job_params[1].tool_number:
            self._report("warning", "Fresado: herramienta 0 es la brida: el programa pone la brida "
                                    "sobre la pieza. Cargá la fresa en Herramientas y elegí su número.")
        summary = (f"{len(path)} puntos, {path.cut_length_mm / 1000:.2f} m de corte, "
                   f"fresa girada {robot.roll_deg:.0f}°.")
        if robot.unreachable:
            self.status.setText(f"{summary} El robot NO alcanza {len(robot.unreachable)} puntos: "
                                f"acercá o girá la pieza.")
        else:
            self.status.setText(f"{summary} El robot alcanza todos.")
            self.code_btn.setEnabled(True)

    def _preview(self, robot: RobotPath) -> None:
        """La trayectoria en la vista 3D, en coordenadas de la base del robot."""
        path = robot.path
        cell = path.hm.to_cell(path.pos)
        base = np.array(self._layout().base_matrix())
        local = (cell - base[:3, 3]) @ base[:3, :3]
        self._on_preview([tuple(p) for p in local.tolist()], [PATH_COLORS[k] for k in path.kinds])

    def generate(self) -> None:
        if self.robot_path is None or self._job_params is None:
            return
        tool, params = self._job_params
        stem = "".join(c if c.isalnum() else "_" for c in self._job_object.name).strip("_")
        try:
            sources = cam.generate_program(self.robot_path, tool, params, f"fresado_{stem or 'pieza'}")
        except CamError as e:
            self._failed(str(e))
            return
        if self._on_program(sources):
            self._job_object.machined = True
            self.status.setText(f"Programa {sources.src.name} cargado en el editor. Simulalo antes "
                                f"de exportar: la fresa ya no cuenta como choque con esta pieza.")
