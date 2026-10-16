"""
Navegación de la vista 3D como en Autodesk Inventor / AutoCAD.

| Acción | Mouse | Teclado |
|---|---|---|
| Zoom hacia el cursor | rueda | |
| Desplazar (pan) | botón del medio, o derecho | flechas |
| Orbitar | Shift + botón del medio, o izquierdo | |
| Encuadrar todo | doble clic con el botón del medio | F6 (vista inicial) |
| Vistas estándar | botones Arriba / Frente / Lado / Iso | |

La órbita es "restringida" (como el 3DORBIT de AutoCAD): gira alrededor del
eje Z del mundo y nunca pone el piso de costado; el pivote es el centro de
la vista. El zoom acerca el punto que está bajo el cursor (no el centro), que
es lo que hacen los dos programas.

La matemática (`CameraNav`) no depende de Qt3D: se prueba sin placa de
video. `NavigationFilter` traduce los eventos del mouse de la ventana 3D.
"""

from __future__ import annotations

import math
from typing import Callable

from PySide6.QtCore import QEvent, QObject, Qt

Vec = tuple[float, float, float]

ORBIT_DEG_PER_PX = 0.4
ZOOM_PER_NOTCH = 0.85      # cada "click" de la rueda acerca un 15 %
MIN_DISTANCE = 50.0        # mm: no atravesar el punto que se mira
MAX_DISTANCE = 100_000.0
MAX_PITCH_DEG = 89.0


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    n = math.sqrt(_dot(a, a)) or 1.0
    return (a[0] / n, a[1] / n, a[2] / n)


class CameraNav:
    """Estado de la cámara: ojo, punto mirado, campo visual y tamaño de la vista."""

    def __init__(self, eye: Vec, center: Vec, fov_deg: float = 40.0,
                 width: int = 800, height: int = 600) -> None:
        self.eye, self.center = tuple(eye), tuple(center)
        self.fov_deg = fov_deg
        self.width, self.height = max(1, width), max(1, height)

    # -- ejes de la pantalla ----------------------------------------------------------

    @property
    def distance(self) -> float:
        return math.dist(self.eye, self.center)

    def basis(self) -> tuple[Vec, Vec, Vec]:
        """(adelante, derecha, arriba) de la pantalla, en el mundo."""
        forward = _norm(_sub(self.center, self.eye))
        right = _cross(forward, (0.0, 0.0, 1.0))
        if math.sqrt(_dot(right, right)) < 1e-6:     # mirando justo para abajo/arriba
            right = (1.0, 0.0, 0.0)
        right = _norm(right)
        up = _cross(right, forward)
        return forward, right, up

    @property
    def up(self) -> Vec:
        return self.basis()[2]

    def mm_per_pixel(self) -> float:
        """Cuánto se mueve el mundo, en el plano del punto mirado, por píxel."""
        return 2 * self.distance * math.tan(math.radians(self.fov_deg) / 2) / self.height

    def point_under(self, x: float, y: float) -> Vec:
        """Punto del plano que pasa por el centro de la vista (perpendicular a
        la mirada) que se ve en el píxel (x, y)."""
        _f, right, up = self.basis()
        k = self.mm_per_pixel()
        dx, dy = (x - self.width / 2) * k, (self.height / 2 - y) * k
        return _add(self.center, _add(_mul(right, dx), _mul(up, dy)))

    # -- movimientos -----------------------------------------------------------------------

    def pan(self, dx_px: float, dy_px: float) -> None:
        """Arrastrar la escena con el mouse: lo que estaba bajo el cursor lo sigue."""
        _f, right, up = self.basis()
        k = self.mm_per_pixel()
        move = _add(_mul(right, -dx_px * k), _mul(up, dy_px * k))
        self.eye, self.center = _add(self.eye, move), _add(self.center, move)

    def orbit(self, dx_px: float, dy_px: float) -> None:
        """Gira alrededor del centro: horizontal = alrededor de Z, vertical =
        sube o baja la mirada (sin pasar de la vertical)."""
        offset = _sub(self.eye, self.center)
        r = math.sqrt(_dot(offset, offset))
        yaw = math.atan2(offset[1], offset[0]) - math.radians(dx_px * ORBIT_DEG_PER_PX)
        pitch = math.asin(max(-1.0, min(1.0, offset[2] / r))) + math.radians(dy_px * ORBIT_DEG_PER_PX)
        limit = math.radians(MAX_PITCH_DEG)
        pitch = max(-limit, min(limit, pitch))
        self.eye = _add(self.center, (r * math.cos(pitch) * math.cos(yaw),
                                      r * math.cos(pitch) * math.sin(yaw), r * math.sin(pitch)))

    def zoom(self, notches: float, x: float | None = None, y: float | None = None) -> None:
        """Acercar (notches > 0) o alejar, manteniendo quieto el punto bajo el cursor."""
        factor = ZOOM_PER_NOTCH ** notches
        d = self.distance
        factor = max(MIN_DISTANCE / d, min(MAX_DISTANCE / d, factor))
        anchor = self.center if x is None or y is None else self.point_under(x, y)
        self.eye = _add(anchor, _mul(_sub(self.eye, anchor), factor))
        self.center = _add(anchor, _mul(_sub(self.center, anchor), factor))

    def fit(self, lo: Vec, hi: Vec) -> None:
        """Encuadrar una caja sin cambiar la dirección de la mirada."""
        forward, _r, _u = self.basis()
        center = tuple((a + b) / 2 for a, b in zip(lo, hi))
        radius = max(250.0, math.dist(lo, hi) / 2)
        fov = math.radians(min(self.fov_deg, self.fov_deg * self.width / self.height)) / 2
        d = radius / math.sin(fov)
        self.center = center
        self.eye = _sub(center, _mul(forward, d))


class NavigationFilter(QObject):
    """Filtro de eventos para la ventana 3D (Qt3DWindow es una QWindow: los
    eventos del mouse llegan a ella, no al widget que la contiene)."""

    def __init__(self, nav: CameraNav, apply: Callable[[], None],
                 on_fit: Callable[[], None] | None = None) -> None:
        super().__init__()
        self.nav = nav
        self._apply = apply
        self._on_fit = on_fit or (lambda: None)
        self._last = None
        self._mode = None   # "orbit" | "pan"

    def eventFilter(self, obj, event) -> bool:  # noqa: N802 — API de Qt
        kind = event.type()
        if kind == QEvent.Type.Resize:
            self.nav.width, self.nav.height = max(1, obj.width()), max(1, obj.height())
            return False
        if kind == QEvent.Type.MouseButtonDblClick and event.button() == Qt.MouseButton.MiddleButton:
            self._on_fit()
            return True
        if kind == QEvent.Type.MouseButtonPress:
            mode = self.mode_for(event.button(), event.modifiers())
            if mode is None:
                return False
            self._mode, self._last = mode, event.position()
            return True
        if kind == QEvent.Type.MouseMove and self._mode is not None:
            pos = event.position()
            dx, dy = pos.x() - self._last.x(), pos.y() - self._last.y()
            self._last = pos
            if self._mode == "orbit":
                self.nav.orbit(dx, dy)
            else:
                self.nav.pan(dx, dy)
            self._apply()
            return True
        if kind == QEvent.Type.MouseButtonRelease and self._mode is not None:
            self._mode = None
            return True
        if kind == QEvent.Type.Wheel:
            notches = event.angleDelta().y() / 120.0
            if notches:
                pos = event.position()
                self.nav.zoom(notches, pos.x(), pos.y())
                self._apply()
            return True
        if kind == QEvent.Type.KeyPress:
            step = 40
            moves = {Qt.Key.Key_Left: (-step, 0), Qt.Key.Key_Right: (step, 0),
                     Qt.Key.Key_Up: (0, -step), Qt.Key.Key_Down: (0, step)}
            if event.key() in moves:
                self.nav.pan(*moves[event.key()])
                self._apply()
                return True
            if event.key() == Qt.Key.Key_F6:
                self._on_fit()
                return True
        return False

    @staticmethod
    def mode_for(button, modifiers) -> str | None:
        """Qué hace cada botón (ver la tabla del módulo)."""
        shift = bool(modifiers & Qt.KeyboardModifier.ShiftModifier)
        if button == Qt.MouseButton.MiddleButton:
            return "orbit" if shift else "pan"
        if button == Qt.MouseButton.LeftButton:
            return "pan" if shift else "orbit"
        if button == Qt.MouseButton.RightButton:
            return "pan"
        return None
