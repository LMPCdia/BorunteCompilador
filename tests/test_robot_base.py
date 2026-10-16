"""
El robot ubicado en la celda (`Layout.robot_base`): corrido y girado. Las
piezas y el piso quedan en coordenadas de la celda; el robot (y los puntos
del programa, que son respecto de su base) se mueve con su base.
"""

import math

import pytest

from sim import placement
from sim.kinematics import RobotModel, mat_mul, pose_matrix
from sim.meshes import box, write_stl
from sim.scene import Layout, LayoutObject, base_matrix, robot_link_meshes

HOME = [0.347, 45.894, -44.865, -0.792, -75.952, -0.859]


def test_layout_keeps_the_robot_base(tmp_path):
    layout = Layout(robot_base=[1000.0, -250.0, 400.0, 0.0, 0.0, 90.0])
    layout.save(tmp_path / "celda.json")
    assert Layout.load(tmp_path / "celda.json").robot_base == [1000.0, -250.0, 400.0, 0, 0, 90]
    m = layout.base_matrix()
    assert [m[i][3] for i in range(3)] == [1000.0, -250.0, 400.0]
    assert m[0][0] == pytest.approx(0, abs=1e-12) and m[1][0] == pytest.approx(1)  # su +X mira a +Y


def test_old_cells_have_the_robot_at_the_origin(tmp_path):
    (tmp_path / "vieja.json").write_text('{"model": "BRTIRUS1510A", "objects": []}',
                                         encoding="utf-8")
    assert Layout.load(tmp_path / "vieja.json").robot_base == [0.0] * 6


@pytest.mark.parametrize("bad", ['[1, 2, 3]', '[0, 0, 0, 0, 0, "x"]', '[0, 0, 0, 0, 0, NaN]'])
def test_bad_robot_base_is_rejected(tmp_path, bad):
    (tmp_path / "mala.json").write_text('{"robot_base": %s}' % bad, encoding="utf-8")
    with pytest.raises(ValueError):
        Layout.load(tmp_path / "mala.json")


# --- choques -----------------------------------------------------------------------------


@pytest.fixture(scope="module")
def robot():
    pytest.importorskip("fcl")
    model = RobotModel.load("BRTIRUS1820A")
    return model, robot_link_meshes(model, tool_axis=False)


def _min_distance(model, links, obstacle, base, q=HOME):
    from sim.collision import CollisionChecker

    checker = CollisionChecker(model, links, [], margin_mm=0, floor=False, self_collision=False,
                               base=base)
    return checker.robot_distance(q, obstacle)[1]


def test_pieces_stay_in_the_cell_when_the_robot_moves(robot):
    from sim.collision import Obstacle
    from sim.kinematics import identity

    model, links = robot
    flange = model.fk(HOME)
    x, y, z = flange[0][3], flange[1][3], flange[2][3]
    # Un bloque justo donde está la brida en HOME: con el robot en el origen, lo toca.
    block = Obstacle("bloque", box((x, y, z), (80, 80, 80)), identity())
    assert _min_distance(model, links, block, None) == 0
    # El robot corrido 2 m: el bloque queda lejos.
    far = base_matrix([0, 2000, 0, 0, 0, 0])
    assert _min_distance(model, links, block, far) > 500
    # Robot y bloque corridos y girados juntos: lo vuelve a tocar.
    pose = [700, -300, 250, 0, 0, 90]
    moved = Obstacle("bloque", box((x, y, z), (80, 80, 80)), base_matrix(pose))
    assert _min_distance(model, links, moved, base_matrix(pose)) == 0


def test_the_floor_is_the_cells(robot):
    from sim.collision import FLOOR, CollisionChecker

    model, links = robot
    def floor_hits(base):
        checker = CollisionChecker(model, links, [], margin_mm=0, self_collision=False,
                                   base=base)
        return [p for p, o, d in checker.distances_at(HOME) if o == FLOOR and d <= 0]

    assert floor_hits(None) == []
    # Colgado del techo pero con la base en el piso: el brazo queda abajo del piso.
    assert floor_hits(base_matrix([0, 0, 0, 180, 0, 0]))
    # Colgado a 3 m: libre.
    assert floor_hits(base_matrix([0, 0, 3000, 180, 0, 0])) == []


# --- ubicar piezas respecto del robot ---------------------------------------------------------

MESA = box((0, 0, 375), (1000, 600, 750))


def test_distances_from_the_robot_follow_where_it_stands():
    base = [1000.0, 500.0, 0.0, 0.0, 0.0, 90.0]       # corrido y mirando a +Y de la celda
    obj = LayoutObject("mesa", "mesa.stl")
    obj.x, obj.y, obj.z = placement.from_robot(obj, MESA, 800, 0, to_face=True, base=base)
    b = placement.world_box(obj, MESA)
    assert b.lo[1] == pytest.approx(500 + 800, abs=0.1)      # adelante del robot = +Y
    assert b.center[0] == pytest.approx(1000, abs=0.1)
    assert placement.axis_distance(b, base) == pytest.approx(800, abs=0.1)
    assert placement.robot_box(base).base == (1000.0, 500.0, 0.0)


# --- GUI -------------------------------------------------------------------------------------


@pytest.fixture
def view():
    from PySide6.QtWidgets import QApplication

    QApplication.instance() or QApplication([])
    from gui.sim_view import SimView

    return SimView(lambda: "", enable_3d=False)


def test_rotate_the_robot_90_degrees_about_each_axis(view):
    view.rotate_robot("X", 90)
    assert view.layout_data.robot_base == [0, 0, 0, 90, 0, 0]
    assert [b.value() for b in view.base_spins][3] == 90
    view.rotate_robot("X", -90)
    assert view.layout_data.robot_base == [0, 0, 0, 0, 0, 0]
    # Girar sobre ejes FIJOS de la celda: primero X, después Z.
    view.set_robot_base([500, 0, 800, 0, 0, 0])
    view.rotate_robot("X", 90)
    view.rotate_robot("Z", 90)
    expected = mat_mul(pose_matrix(0, 0, 0, 0, 0, 90), pose_matrix(500, 0, 800, 90, 0, 0))
    got = view.layout_data.base_matrix()
    assert got[0][3] == pytest.approx(500) and got[2][3] == pytest.approx(800)  # la base no se corre
    for i in range(3):
        for j in range(3):
            assert got[i][j] == pytest.approx(expected[i][j], abs=1e-9)


def test_typing_the_position_moves_the_robot_and_is_saved(view, tmp_path):
    view.base_spins[0].setValue(1200)
    view.base_spins[5].setValue(-45)
    assert view.layout_data.robot_base == [1200, 0, 0, 0, 0, -45]
    assert view.save_layout(tmp_path / "celda.json")
    other = type(view)(lambda: "", enable_3d=False)
    assert other.load_layout(tmp_path / "celda.json")
    assert [b.value() for b in other.base_spins] == [1200, 0, 0, 0, 0, -45]
    view.set_robot_base([0] * 6)                     # "Al origen"
    assert [b.value() for b in view.base_spins] == [0] * 6


def test_moving_the_robot_changes_the_measured_distances(view, tmp_path):
    pytest.importorskip("fcl")
    write_stl(MESA, tmp_path / "mesa.stl")
    view.import_object(tmp_path / "mesa.stl")
    view.place_mode.setCurrentIndex(0)
    view.place_distance.setValue(900)
    assert view.apply_placement()
    near = view.measure_selected()
    view.set_robot_base([-1500, 0, 0, 0, 0, 0])
    assert "al eje del robot: <b>2400 mm</b>" in view.place_info.text()
    far = view.measure_selected()
    d = [float(t.split("<b>")[1].split(" mm")[0]) for t in (near, far)]
    assert d[1] > d[0] + 1000
    # Ubicar "a una distancia del robot" mide desde donde está ahora.
    assert view.apply_placement()
    assert view._box(0).lo[0] == pytest.approx(-1500 + 900, abs=0.2)
    # Y "Corrida respecto de la base del robot" también.
    view.place_mode.setCurrentIndex(1)
    view.place_dx.setValue(0)
    view.place_dy.setValue(0)
    assert view.apply_placement()
    assert view._box(0).base == pytest.approx((-1500, 0, 0), abs=0.2)


def test_moving_the_robot_marks_the_collisions_stale(view):
    view.result = object()                # algo simulado
    view.collisions_check.setChecked(True)
    view.set_robot_base([0, 0, 500, 0, 0, 0])
    assert view.stale
    assert math.isclose(view.layout_data.base_matrix()[2][3], 500)
