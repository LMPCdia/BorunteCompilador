"""
Ventana "Gráficas de movimiento": posición, velocidad y aceleración de cada
eje, y velocidad y aceleración lineal de la punta, a lo largo de la
simulación. Los choques y las cercanías se marcan como franjas rojas y
naranjas; una línea vertical sigue a la animación, y un clic en cualquier
gráfica lleva la animación a ese instante.

Usa QtCharts (viene con PySide6): se dibuja sin OpenGL, así que se prueba
igual que el resto.
"""

from __future__ import annotations

from typing import Callable

from PySide6.QtCharts import QAreaSeries, QChart, QChartView, QLineSeries, QValueAxis
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QHeaderView,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from sim.kinematics import RobotModel
from sim.motion import MotionCurves

JOINT_COLORS = ["#e6553a", "#2f9e44", "#1c7ed6", "#c2255c", "#f08c00", "#7048e8"]
TCP_COLOR = "#0c8599"
HIT = QColor(224, 40, 40, 70)
NEAR = QColor(240, 160, 32, 60)
MAX_BANDS = 60

TABS = [
    ("Ejes: posición", "Ángulo (°)", "angle"),
    ("Ejes: velocidad", "Velocidad angular (°/s)", "speed"),
    ("Ejes: aceleración", "Aceleración angular (°/s²)", "accel"),
    ("Punta: velocidad", "Velocidad lineal (mm/s)", "tcp_speed"),
    ("Punta: aceleración", "Aceleración lineal (mm/s²)", "tcp_accel"),
]


class _ChartView(QChartView):
    """Gráfica que avisa en qué instante se hizo clic."""

    def __init__(self, chart: QChart, on_click: Callable[[float], None]) -> None:
        super().__init__(chart)
        self._on_click = on_click
        self.setRenderHint(QPainter.RenderHint.Antialiasing)

    def mousePressEvent(self, event) -> None:  # noqa: N802 — API de Qt
        if event.button() == Qt.MouseButton.LeftButton:
            area = self.chart().plotArea()
            pos = self.chart().mapFromScene(self.mapToScene(event.position().toPoint()))
            if area.contains(pos):
                self._on_click(self.chart().mapToValue(pos).x())
                return
        super().mousePressEvent(event)


class MotionCharts(QWidget):
    def __init__(self, on_time: Callable[[float], None] | None = None) -> None:
        super().__init__()
        self.setWindowTitle("Gráficas de movimiento")
        self._on_time = on_time or (lambda _t: None)
        self.curves: MotionCurves | None = None
        self.note = QLabel("Simulá un programa para ver sus gráficas.")
        self.note.setWordWrap(True)
        self.note.setTextFormat(Qt.TextFormat.RichText)
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            ["", "Recorrido", "Velocidad máx.", "% del límite", "Aceleración máx.", ""])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setMaximumHeight(230)
        self.tabs = QTabWidget()
        self.views: dict[str, _ChartView] = {}
        self._cursors: dict[str, QLineSeries] = {}
        self._ranges: dict[str, tuple[float, float]] = {}
        self._keep: list[QLineSeries] = []
        for title, _unit, key in TABS:
            chart = QChart()
            chart.legend().setAlignment(Qt.AlignmentFlag.AlignRight)
            view = _ChartView(chart, self._clicked)
            self.views[key] = view
            self.tabs.addTab(view, title)
        layout = QVBoxLayout(self)
        layout.addWidget(self.note)
        layout.addWidget(self.table)
        layout.addWidget(self.tabs, 1)
        self.resize(1000, 750)

    # -- datos -----------------------------------------------------------------------

    def set_data(self, curves: MotionCurves, model: RobotModel, accel_s: float,
                 collisions=None) -> None:
        self.curves = curves
        dark = self.palette().window().color().lightness() < 128
        bands = []
        if collisions is not None:
            for c in collisions.contacts[:MAX_BANDS]:
                start, end = c.start_s, max(c.end_s, c.start_s + 0.05)
                bands.append((start, end, HIT if c.colliding else NEAR))
        for _title, unit, key in TABS:
            if key in ("angle", "speed", "accel"):
                series = [(f"J{j + 1}", getattr(curves, key)[j], JOINT_COLORS[j]) for j in range(6)]
            else:
                series = [("Punta", getattr(curves, key), TCP_COLOR)]
            self._fill(key, unit, curves.t, series, bands, dark)
        self._fill_table(curves, model)
        parts = [f"<b>{model.name}</b> · {curves.t[-1]:.1f} s" if curves.t else model.name]
        if curves.accelerations_valid:
            parts.append(f"aceleraciones con un arranque y frenado <b>supuesto</b> de "
                         f"{accel_s:g} s por movimiento (el pad no informa el real)")
        else:
            parts.append("<b>sin perfil de aceleración</b>: las aceleraciones no significan nada "
                         "(poné un tiempo de aceleración en Simulación 3D)")
        if collisions is not None and collisions.contacts:
            parts.append("<span style='color:#d04040'>franjas rojas: choques</span>, "
                         "<span style='color:#d08000'>naranjas: más cerca que el margen</span>")
        parts.append("velocidad de la punta en MOVEL según los ejes (falta la velocidad lineal "
                     "máxima del robot). Clic en una gráfica: la animación va a ese instante")
        self.note.setText(" · ".join(parts) + ".")

    def _fill(self, key: str, unit: str, ts, series, bands, dark: bool) -> None:
        chart = self.views[key].chart()
        chart.removeAllSeries()
        if key == TABS[0][2]:
            self._keep = []
        for axis in chart.axes():
            chart.removeAxis(axis)
        chart.setTheme(QChart.ChartTheme.ChartThemeDark if dark else QChart.ChartTheme.ChartThemeLight)
        ax_t, ax_y = QValueAxis(), QValueAxis()
        ax_t.setTitleText("Tiempo (s)")
        ax_y.setTitleText(unit)
        chart.addAxis(ax_t, Qt.AlignmentFlag.AlignBottom)
        chart.addAxis(ax_y, Qt.AlignmentFlag.AlignLeft)
        lo = min((min(ys) for _n, ys, _c in series if ys), default=0.0)
        hi = max((max(ys) for _n, ys, _c in series if ys), default=1.0)
        if hi - lo < 1e-9:
            lo, hi = lo - 1, hi + 1
        pad = (hi - lo) * 0.05
        lo, hi = lo - pad, hi + pad
        self._ranges[key] = (lo, hi)
        t_end = ts[-1] if ts else 1.0
        for start, end, color in bands:
            low, high = QLineSeries(), QLineSeries()
            low.append(start, lo)
            low.append(end, lo)
            high.append(start, hi)
            high.append(end, hi)
            band = QAreaSeries(high, low)
            band.setColor(color)
            band.setBorderColor(color)
            chart.addSeries(band)
            band.attachAxis(ax_t)
            band.attachAxis(ax_y)
            chart.legend().markers(band)[0].setVisible(False)
            self._keep += [low, high]  # el área no se queda con sus bordes: que no se liberen
        for name, ys, color in series:
            s = QLineSeries()
            s.setName(name)
            s.replace([QPointF(t, y) for t, y in zip(ts, ys)])
            pen = QPen(QColor(color))
            pen.setWidthF(1.6)
            s.setPen(pen)
            chart.addSeries(s)
            s.attachAxis(ax_t)
            s.attachAxis(ax_y)
            marker = chart.legend().markers(s)[0]
            marker.clicked.connect(lambda s=s: s.setVisible(not s.isVisible()))
        cursor = QLineSeries()
        cursor.setPen(QPen(QColor("#888888"), 1, Qt.PenStyle.DashLine))
        cursor.append(0.0, lo)
        cursor.append(0.0, hi)
        chart.addSeries(cursor)
        cursor.attachAxis(ax_t)
        cursor.attachAxis(ax_y)
        chart.legend().markers(cursor)[0].setVisible(False)
        self._cursors[key] = cursor
        ax_t.setRange(0.0, t_end)
        ax_y.setRange(lo, hi)
        ax_t.applyNiceNumbers()   # marcas en valores redondos (1, 2, 5...)
        ax_y.applyNiceNumbers()
        self._ranges[key] = (ax_y.min(), ax_y.max())
        cursor.replace([QPointF(0.0, ax_y.min()), QPointF(0.0, ax_y.max())])

    def _fill_table(self, curves: MotionCurves, model: RobotModel) -> None:
        peaks = curves.peaks(model)
        self.table.setRowCount(len(peaks) + 1)
        for row, p in enumerate(peaks):
            joint = model.joints[row]
            values = [p["eje"], f"{p['min_deg']:.1f}° a {p['max_deg']:.1f}°",
                      f"{p['v_max']:.0f} °/s", f"{p['v_pct']:.0f} % de {joint.max_speed_dps:g} °/s",
                      f"{p['a_max']:.0f} °/s²" if curves.accelerations_valid else "—",
                      f"rango {joint.min_deg:g}° a {joint.max_deg:g}°"]
            self._row(row, values, JOINT_COLORS[row])
        vmax = max(curves.tcp_speed, default=0.0)
        amax = max(curves.tcp_accel, default=0.0)
        self._row(len(peaks), ["Punta", "", f"{vmax:.0f} mm/s", "",
                               f"{amax:.0f} mm/s²" if curves.accelerations_valid else "—", ""],
                  TCP_COLOR)

    def _row(self, row: int, values: list[str], color: str) -> None:
        for col, text in enumerate(values):
            item = QTableWidgetItem(text)
            if col == 0:
                item.setForeground(QColor(color))
            self.table.setItem(row, col, item)

    # -- tiempo ------------------------------------------------------------------------

    def set_time(self, t: float) -> None:
        for key, cursor in self._cursors.items():
            lo, hi = self._ranges[key]
            cursor.replace([QPointF(t, lo), QPointF(t, hi)])

    def _clicked(self, t: float) -> None:
        if self.curves is None or not self.curves.t:
            return
        self._on_time(max(0.0, min(t, self.curves.t[-1])))
