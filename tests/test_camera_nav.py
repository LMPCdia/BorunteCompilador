"""Navegación 3D tipo Inventor/AutoCAD (gui/camera_nav.py): solo la
matemática; la vista Qt3D no se construye en los tests (sin OpenGL)."""

import math

import pytest
from PySide6.QtCore import Qt

from gui.camera_nav import CameraNav, NavigationFilter


def nav():
    return CameraNav((2000.0, 0.0, 0.0), (0.0, 0.0, 0.0), fov_deg=40, width=800, height=600)


def screen_x(n: CameraNav, p) -> float:
    """Dónde cae un punto en la pantalla (coordenada x, en mm del plano focal)."""
    _f, right, _u = n.basis()
    return sum((a - b) * r for a, b, r in zip(p, n.center, right))


def test_wheel_zooms_toward_the_point_under_the_cursor():
    n = nav()
    target = n.point_under(700, 150)
    d0 = n.distance
    n.zoom(2, 700, 150)
    assert n.distance == pytest.approx(d0 * 0.85 ** 2)
    # El punto bajo el cursor sigue bajo el cursor.
    assert n.point_under(700, 150) == pytest.approx(target, abs=1e-6)
    n.zoom(-2, 700, 150)
    assert n.distance == pytest.approx(d0)


def test_zoom_never_goes_through_the_target():
    n = nav()
    n.zoom(200)
    assert n.distance >= 49.9


def test_pan_drags_the_scene_with_the_mouse():
    n = nav()
    p = (0.0, 0.0, 0.0)
    before = screen_x(n, p)
    n.pan(100, 0)                    # mouse a la derecha
    assert screen_x(n, p) > before    # la escena se fue a la derecha
    assert n.distance == pytest.approx(2000)
    # 100 px = 100 × (mm por píxel en el plano mirado)
    assert screen_x(n, p) - before == pytest.approx(100 * n.mm_per_pixel())


def test_orbit_turns_around_z_and_keeps_the_floor_level():
    n = nav()
    n.orbit(-90 / 0.4, 0)            # 90° de giro
    assert n.eye == pytest.approx((0.0, 2000.0, 0.0), abs=1e-6)
    assert n.center == (0.0, 0.0, 0.0)
    assert n.up[2] > 0.99            # Z sigue para arriba
    n.orbit(0, 1000)                 # mucho para arriba: se frena antes de la vertical
    assert n.eye[2] < 2000 and n.eye[2] > 1990
    assert n.distance == pytest.approx(2000)


def test_fit_frames_a_box_without_turning():
    n = nav()
    forward = n.basis()[0]
    n.fit((1000, -500, 0), (2000, 500, 1500))
    assert n.center == pytest.approx((1500, 0, 750))
    assert n.basis()[0] == pytest.approx(forward)
    radius = math.dist((1000, -500, 0), (2000, 500, 1500)) / 2
    assert n.distance * math.sin(math.radians(20)) == pytest.approx(radius)


@pytest.mark.parametrize("button, shift, mode", [
    (Qt.MouseButton.MiddleButton, False, "pan"),
    (Qt.MouseButton.MiddleButton, True, "orbit"),
    (Qt.MouseButton.LeftButton, False, "orbit"),
    (Qt.MouseButton.RightButton, False, "pan"),
])
def test_buttons_do_what_inventor_does(button, shift, mode):
    mods = Qt.KeyboardModifier.ShiftModifier if shift else Qt.KeyboardModifier.NoModifier
    assert NavigationFilter.mode_for(button, mods) == mode
