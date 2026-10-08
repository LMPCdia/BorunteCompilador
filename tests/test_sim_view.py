"""
Mallas, layout, línea de tiempo y la pestaña "Simulación 3D".

La vista 3D (Qt3D) NO se construye en ningún test: sin OpenGL hace caer el
proceso entero (ver gui/viewport3d.py). Todo lo demás se prueba igual.
"""

import struct

import pytest
from PySide6.QtWidgets import QApplication

from sim.kinematics import RobotModel
from sim.meshes import Mesh, MeshError, box, cylinder, load_mesh, load_step, write_stl
from sim.pad_sim import simulate
from sim.scene import Layout, LayoutObject, Timeline, robot_link_meshes

HOME = "JOINT(0.347, 45.894, -44.865, -0.792, -75.952, -0.859)"


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(scope="module")
def model():
    return RobotModel.load("BRTIRUS1820A")


# --- mallas --------------------------------------------------------------------------


def test_box_and_cylinder_bounds():
    (lo, hi) = box((0, 0, 10), (100, 50, 20)).bounds()
    assert lo == pytest.approx((-50, -25, 0)) and hi == pytest.approx((50, 25, 20))
    (lo, hi) = cylinder((0, 0, 0), (0, 0, 100), 10).bounds()
    assert lo[2] == pytest.approx(0) and hi[2] == pytest.approx(100)
    assert hi[0] == pytest.approx(10, abs=0.1)


def test_interleaved_has_position_and_normal_per_vertex():
    mesh = Mesh()
    mesh.add((0, 0, 0), (1, 0, 0), (0, 1, 0))
    data = mesh.interleaved()
    assert len(data) == 3 * 6 * 4
    assert struct.unpack_from("<6f", data, 0) == (0, 0, 0, 0, 0, 1)


def test_binary_stl_roundtrip(tmp_path):
    path = tmp_path / "caja.stl"
    write_stl(box((0, 0, 0), (10, 10, 10)), path)
    mesh = load_mesh(path)
    assert len(mesh) == 12
    assert mesh.bounds()[1] == pytest.approx((5, 5, 5))


def test_binary_stl_starting_with_solid_is_still_binary(tmp_path):
    path = tmp_path / "solid.stl"
    write_stl(box((0, 0, 0), (10, 10, 10)), path)
    data = bytearray(path.read_bytes())
    data[:5] = b"solid"
    path.write_bytes(bytes(data))
    assert len(load_mesh(path)) == 12


def test_ascii_stl(tmp_path):
    path = tmp_path / "tri.stl"
    path.write_text("solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\n"
                    "vertex 0 1 0\nendloop\nendfacet\nendsolid t\n")
    assert load_mesh(path).triangles == [((0, 0, 0), (1, 0, 0), (0, 1, 0))]


def test_obj_with_quads_and_texture_indices(tmp_path):
    path = tmp_path / "quad.obj"
    path.write_text("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nf 1/1 2/1 3/1 4/1\n")
    assert len(load_mesh(path)) == 2


def test_step_through_gmsh(tmp_path):
    gmsh = pytest.importorskip("gmsh")
    path = tmp_path / "caja.step"
    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.addBox(0, 0, 0, 100, 50, 20)
        gmsh.model.occ.synchronize()
        gmsh.write(str(path))
    finally:
        gmsh.finalize()
    mesh = load_step(path)
    lo, hi = mesh.bounds()
    assert lo == pytest.approx((0, 0, 0), abs=1e-3)
    assert hi == pytest.approx((100, 50, 20), abs=1e-3)


def test_unsupported_and_broken_files(tmp_path):
    with pytest.raises(MeshError, match="Formato"):
        load_mesh(tmp_path / "x.dwg")
    broken = tmp_path / "roto.step"
    broken.write_text("esto no es un STEP")
    with pytest.raises(MeshError):
        load_mesh(broken)


# --- escena ---------------------------------------------------------------------------


def test_robot_has_a_mesh_per_link(model):
    meshes = robot_link_meshes(model)
    assert len(meshes) == 7 and all(len(m) > 0 for m in meshes)


def test_flange_mesh_ends_at_the_flange(model):
    # El último eslabón (brida) termina en la brida + el eje de herramienta.
    (_, hi) = robot_link_meshes(model)[6].bounds()
    assert hi[0] == pytest.approx(model.a1 + model.d4 + model.d6 + 60, abs=0.5)


def test_layout_roundtrip_keeps_relative_paths(tmp_path):
    piece = tmp_path / "piezas" / "mesa.stl"
    piece.parent.mkdir()
    write_stl(box((0, 0, 0), (10, 10, 10)), piece)
    layout = Layout(objects=[LayoutObject("mesa", str(piece), x=1200, y=-300, z=0, rz=90)])
    path = tmp_path / "celda.layout.json"
    layout.save(path)
    assert '"piezas/mesa.stl"' in path.read_text(encoding="utf-8").replace("\\\\", "/")
    again = Layout.load(path)
    assert again.objects[0].x == 1200 and again.objects[0].rz == 90
    assert again.objects[0].path == str(tmp_path / "piezas" / "mesa.stl")


def test_timeline_interpolates_and_holds_waits(model):
    from compiler.pad_codegen import compile_to_pad

    result = simulate(compile_to_pad(f"MOVEJ {HOME} SPEED 50\nWAIT 1s\n"), model)
    timeline = Timeline(result)
    move = result.segments[0].duration_s
    assert timeline.duration == pytest.approx(move + 1)
    assert timeline.at(0)[0] == [0.0] * 6
    mid, where = timeline.at(move / 2)
    assert mid[1] == pytest.approx(45.894 / 2, abs=1.5) and where == "MAIN[3]"
    assert timeline.at(move + 0.5)[0] == pytest.approx(timeline.at(move)[0])


# --- pestaña --------------------------------------------------------------------------


def _view(app, source, reports=None):
    from gui.sim_view import SimView

    return SimView(lambda: source, report=(lambda s, m: reports.append((s, m))) if reports is not None else None,
                   enable_3d=False)


def test_simulate_from_the_editor_source(app):
    reports = []
    view = _view(app, f"MOVEJ {HOME} SPEED 50\nMOVEJ JOINT(0, 80, 0, 0, 0, 0) SPEED 10\n", reports)
    result = view.simulate()
    assert result is not None and not result.ok
    assert any(s == "error" and "J2=80.0°" in m for s, m in reports)
    assert "BRTIRUS1820A" in view.summary.text()
    view.set_time(view.timeline.duration)
    assert view.current_q[1] == pytest.approx(80)
    assert view.slider.value() == 1000


def test_compile_error_is_reported_not_raised(app):
    reports = []
    view = _view(app, "WAIT_IN(X010, 5)\n", reports)
    assert view.simulate() is None
    assert any(s == "error" and "WAIT_IN" in m for s, m in reports)


def test_inputs_box_drives_the_if(app):
    from gui.sim_view import parse_inputs

    assert parse_inputs("X012=1, X013=0") == {2: True, 3: False}
    view = _view(app, "PROC p()\nWAIT 1s\nENDPROC\nIF X012 == 1 THEN\np()\nENDIF\n")
    view.inputs_edit.setText("X012=1")
    assert view.simulate().total_time_s == pytest.approx(1)
    view.inputs_edit.setText("")
    assert view.simulate().total_time_s == 0


def test_import_object_rests_on_the_floor_and_is_editable(app, tmp_path):
    piece = tmp_path / "mesa.stl"
    write_stl(box((0, 0, -100), (800, 600, 40)), piece)
    view = _view(app, "")
    obj = view.import_object(piece)
    assert obj.z == pytest.approx(120)
    assert view.objects_table.rowCount() == 1
    view.objects_table.item(0, 1).setText("1500")
    assert view.layout_data.objects[0].x == 1500


def test_main_window_has_the_sim_tab(app):
    from gui.main_window import MainWindow

    window = MainWindow()
    assert window.tabs.indexOf(window.sim_view) >= 0
