"""Ubicar piezas por distancias y medir (sim/placement.py + pestaña Piezas)."""

import math

import pytest

from sim import placement
from sim.meshes import box, write_stl
from sim.scene import LayoutObject

MESA = box((50, 25, 10), (1000, 600, 750))      # origen del CAD en una esquina rara


def mesa(**kw):
    return LayoutObject("mesa", "mesa.stl", **kw)


def test_world_box_follows_position_and_rotation():
    b = placement.world_box(mesa(x=100, y=0, z=0), MESA)
    assert b.size == pytest.approx((1000, 600, 750))
    assert b.base == pytest.approx((150, 25, -365))
    turned = placement.world_box(mesa(rz=90), MESA)
    assert turned.size == pytest.approx((600, 1000, 750))


def test_from_robot_measures_to_the_face_or_the_center():
    obj = mesa()
    obj.x, obj.y, obj.z = placement.from_robot(obj, MESA, 800, 0, to_face=True)
    b = placement.world_box(obj, MESA)
    assert b.lo[0] == pytest.approx(800, abs=0.1)        # cara que mira al robot
    assert b.center[1] == pytest.approx(0, abs=0.1)
    assert b.lo[2] == pytest.approx(0, abs=0.1)          # apoyada en el piso
    assert placement.axis_distance(b) == pytest.approx(800, abs=0.1)
    obj.x, obj.y, obj.z = placement.from_robot(obj, MESA, 1200, 90, to_face=False)
    c = placement.world_box(obj, MESA).center
    assert (c[0], c[1]) == pytest.approx((0, 1200), abs=0.1)


def test_next_to_leaves_the_gap_between_faces():
    ref = placement.world_box(mesa(x=1000), MESA)
    other = LayoutObject("caja", "caja.stl")
    caja = box((0, 0, 100), (200, 200, 200))
    for side, axis, sign in (("+X", 0, 1), ("-X", 0, -1), ("+Y", 1, 1), ("-Y", 1, -1)):
        other.x, other.y, other.z = placement.next_to(other, caja, ref, side, 150)
        b = placement.world_box(other, caja)
        gap = b.lo[axis] - ref.hi[axis] if sign > 0 else ref.lo[axis] - b.hi[axis]
        assert gap == pytest.approx(150, abs=0.1), side
        assert b.lo[2] == pytest.approx(ref.lo[2], abs=0.1)
        assert placement.gaps(b, ref)[axis] == pytest.approx(150, abs=0.1)
    with pytest.raises(ValueError):
        placement.next_to(other, caja, ref, "arriba", 10)


def test_on_top_and_offset():
    ref = placement.world_box(mesa(x=1000), MESA)
    pieza = LayoutObject("pieza", "p.stl")
    chica = box((0, 0, 0), (100, 100, 40))
    pieza.x, pieza.y, pieza.z = placement.on_top(pieza, chica, ref, 50, 0)
    b = placement.world_box(pieza, chica)
    assert b.lo[2] == pytest.approx(ref.hi[2], abs=0.1)
    assert b.center[0] == pytest.approx(ref.center[0] + 50, abs=0.1)
    pieza.x, pieza.y, pieza.z = placement.offset_from(pieza, chica, placement.ROBOT_BASE, 900, -300, 0)
    assert placement.world_box(pieza, chica).base == pytest.approx((900, -300, 0), abs=0.1)


def test_clearance_between_surfaces():
    pytest.importorskip("fcl")
    a = mesa()
    b = LayoutObject("b", "b.stl", x=a.x + 1000 + 250)    # 250 mm entre caras
    assert placement.clearance(a, MESA, b, MESA) == pytest.approx(250, abs=0.5)
    b.x = a.x + 500
    assert placement.clearance(a, MESA, b, MESA) == 0


# --- pestaña Piezas -------------------------------------------------------------------------


@pytest.fixture(scope="module")
def app():
    from PySide6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def test_place_and_measure_from_the_gui(app, tmp_path):
    pytest.importorskip("fcl")
    from gui.sim_view import SimView

    write_stl(MESA, tmp_path / "mesa.stl")
    write_stl(box((0, 0, 100), (300, 300, 200)), tmp_path / "caja.stl")
    view = SimView(lambda: "", enable_3d=False)
    view.import_object(tmp_path / "mesa.stl")
    assert view.selected_piece() == 0                   # recién importada: elegida
    view.place_mode.setCurrentIndex(0)
    view.place_distance.setValue(900)
    view.place_angle.setValue(0)
    assert view.apply_placement()
    mesa_box = view._box(0)
    assert mesa_box.lo[0] == pytest.approx(900, abs=0.2)
    assert view.objects_table.item(0, 1).text() == f"{view.layout_data.objects[0].x:g}"
    assert "al eje del robot: <b>900 mm</b>" in view.place_info.text()

    view.import_object(tmp_path / "caja.stl")
    assert view.selected_piece() == 1
    view.place_mode.setCurrentIndex(2)                   # al lado de…
    assert view.place_ref.currentText() == "«mesa»"
    view.place_side.setCurrentText("+Y")
    view.place_gap.setValue(120)
    assert view.apply_placement()
    assert placement.gaps(view._box(1), mesa_box)[1] == pytest.approx(120, abs=0.2)

    text = view.measure_selected()
    assert "a «mesa»: <b>120 mm</b>" in text
    assert "al robot (pose actual)" in text

    view.place_mode.setCurrentIndex(3)                   # encima de la mesa
    assert view.apply_placement()
    assert view._box(1).lo[2] == pytest.approx(mesa_box.hi[2], abs=0.2)


def test_offset_from_the_robot_base(app, tmp_path):
    from gui.sim_view import SimView

    write_stl(MESA, tmp_path / "mesa.stl")
    view = SimView(lambda: "", enable_3d=False)
    view.import_object(tmp_path / "mesa.stl")
    view.place_mode.setCurrentIndex(1)
    assert view.place_ref.currentText() == "Base del robot"
    view.place_dx.setValue(1500)
    view.place_dy.setValue(-200)
    assert view.apply_placement()
    assert view._box(0).base == pytest.approx((1500, -200, 0), abs=0.2)


def test_placing_without_a_selection_warns(app):
    from gui.sim_view import SimView

    reports = []
    view = SimView(lambda: "", report=lambda s, m: reports.append((s, m)), enable_3d=False)
    assert not view.apply_placement()
    assert reports and reports[-1][0] == "warning"
