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

    src = f"MOVEJ {HOME} SPEED 50\nMOVEJ JOINT(40, 45.894, -44.865, -0.792, -75.952, -0.859) SPEED 50\nWAIT 1s\n"
    result = simulate(compile_to_pad(src), model)
    timeline = Timeline(result)
    move = result.segments[1].duration_s
    assert timeline.duration == pytest.approx(move + 1)
    assert timeline.at(0)[0][0] == pytest.approx(0.347)       # arranca en el primer MOVEJ
    mid, where = timeline.at(move / 2)
    assert mid[0] == pytest.approx((0.347 + 40) / 2, abs=0.5) and where.startswith("MAIN[2]")
    assert timeline.at(move + 0.5)[0] == pytest.approx(timeline.at(move)[0])
    pose, _ = timeline.at(0)
    pose[0] = 999  # at() devuelve copias
    assert timeline.at(0)[0][0] != 999


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


def test_input_checkboxes_appear_and_drive_the_if(app):
    from gui.sim_view import parse_inputs

    assert parse_inputs("X012=1, X013=0") == {2: True, 3: False}
    view = _view(app, "PROC p()\nWAIT 1s\nENDPROC\nIF X012 == 1 THEN\np()\nENDIF\n")
    assert view.simulate().total_time_s == 0
    assert [b.text() for b in view._input_boxes.values()] == ["X012"]
    view.set_input("X012", True)  # tildar re-simula solo
    assert view.result.total_time_s == pytest.approx(1)


def _backup_zip(tmp_path, source):
    from compiler.pad_codegen import PadOptions, compile_to_pad

    return compile_to_pad(source, PadOptions(program_name="Prueba")).write(tmp_path)


def test_open_backup_simulates_it_instead_of_the_editor(app, tmp_path):
    path = _backup_zip(tmp_path, f"MOVEJ {HOME} SPEED 50\nIF X030 == 0 THEN\nWAIT 2s\nENDIF\n")
    reports = []
    view = _view(app, "esto no compila", reports)
    assert view.open_backup(path)
    assert "Prueba" in view.source_label.text()
    assert [b.text() for b in view._input_boxes.values()] == ["X030"]
    assert view.simulate().segments[-1].kind == "WAIT"
    view.set_input("X030", True)  # con X030 prendida se saltea el WAIT
    assert view.result.segments[-1].kind == "MOVEJ"
    view.use_editor()
    assert view.simulate() is None  # vuelve al editor, que no compila


def test_open_broken_backup_reports(app, tmp_path):
    reports = []
    bad = tmp_path / "HCBackupRobot_20260101000000.zip"
    bad.write_bytes(b"no es un zip")
    assert not _view(app, "", reports).open_backup(bad)
    assert reports and reports[0][0] == "error"


def test_tools_and_frames_tables_feed_the_simulation(app):
    view = _view(app, f"MOVEJ {HOME} SPEED 50\nTOOL 2\nMOVEL WORLD(1556, 7, 900, 180, 0, 0) SPEED 20\n")
    result = view.simulate()
    assert "Falta cargar herramienta 2" in result.issues[0].message
    assert view.load_missing_btn.isVisibleTo(view)
    view.add_missing_frames()                       # agrega la fila 2, vacía (en rojo)
    assert view.tools_table.table.rowCount() == 1 and view.tools_table.values() == {}
    assert view.side_tabs.currentWidget() is view.tools_table
    view.tools_table.set_values({})
    view.set_tool(2, [0, 0, 100, 0, 0, 0])
    assert view.tools_table.values() == {2: [0, 0, 100, 0, 0, 0]}
    assert view.stale
    result = view.simulate()
    assert not any("falta cargar" in i.message for i in result.issues)
    assert result.segments[-1].kind == "MOVEL"


def test_cell_with_tools_frames_and_pieces_roundtrips(app, tmp_path):
    piece = tmp_path / "mesa.stl"
    write_stl(box((0, 0, 0), (100, 100, 100)), piece)
    view = _view(app, "")
    view.import_object(piece)
    view.set_tool(2, [10, 0, 250, 0, 0, 0])
    view.set_frame(1, [800, -300, 200, 0, 0, 30])
    path = tmp_path / "celda.layout.json"
    view.save_layout(path)

    other = _view(app, "")
    other.load_layout(path)
    assert other.tools_table.values() == {2: [10, 0, 250, 0, 0, 0]}
    assert other.frames_table.values() == {1: [800, -300, 200, 0, 0, 30]}
    assert other.objects_table.rowCount() == 1


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
    assert window.tabs.indexOf(window.sim_tab) >= 0



# --- robustez (hallazgos de la prueba de usuario) ---------------------------------


def test_simulator_messages_are_replaced_not_stacked(app):
    cleared = []
    from gui.sim_view import SimView

    view = SimView(lambda: f"MOVEJ {HOME} SPEED 50\n", enable_3d=False,
                   clear_reports=lambda: cleared.append(1))
    view.simulate()
    view.simulate()
    assert len(cleared) == 2


def test_many_issues_go_to_the_problems_tab_not_all_to_the_messages(app):
    reports = []
    src = f"MOVEJ {HOME} SPEED 50\n" + "MOVEJ JOINT(0, 80, 0, 0, 0, 0) SPEED 10\n" * 30
    view = _view(app, src, reports)
    result = view.simulate()
    assert view.issues_list.count() == len(result.issues) == 30
    assert sum(1 for s, _ in reports if s == "error") <= 13
    assert any("problema(s) más" in m for _, m in reports)
    item = view.issues_list.item(5)
    view._on_issue_activated(item)                  # clic: va a ese momento
    assert view._t == pytest.approx(result.issues[5].time_s)


def test_changing_the_model_marks_the_result_stale(app, tmp_path, monkeypatch):
    import json

    import sim.kinematics as kin

    data = json.loads((kin.MODELS_DIR / "BRTIRUS1820A.json").read_text(encoding="utf-8"))
    (tmp_path / "Otro.json").write_text(json.dumps(data), encoding="utf-8")
    monkeypatch.setattr(kin, "USER_MODELS_DIR", tmp_path)
    view = _view(app, f"MOVEJ {HOME} SPEED 50\n")
    view.simulate()
    view.set_model("Otro")
    assert view.stale and "Desactualizado" in view.summary.text()
    view.play_btn.setChecked(True)                  # no reproduce algo desactualizado
    assert not view.play_btn.isChecked()


def test_broken_model_files_do_not_break_the_tab(app, tmp_path, monkeypatch):
    import sim.kinematics as kin

    (tmp_path / "A_ROTO.json").write_text("{roto", encoding="utf-8")
    monkeypatch.setattr(kin, "USER_MODELS_DIR", tmp_path)
    reports = []
    view = _view(app, "", reports)
    assert view.model is not None and view.model.name == "BRTIRUS1820A"
    assert any(s == "warning" and "A_ROTO.json" in m for s, m in reports)
    assert not view.set_model("A_ROTO")
    assert view.model.name == "BRTIRUS1820A"


def test_play_without_a_simulation_does_nothing(app):
    view = _view(app, "")
    view.play_btn.setChecked(True)
    assert not view.play_btn.isChecked()


def test_bad_files_are_reported_not_raised(app, tmp_path):
    reports = []
    view = _view(app, "", reports)
    bad_stl = tmp_path / "roto.stl"
    bad_stl.write_text("solid x\nvertex a b c\n")
    assert view.import_object(bad_stl) is None
    assert view.import_object(tmp_path / "no_existe.step") is None
    bad_obj = tmp_path / "roto.obj"
    bad_obj.write_text("v 0 0 0\nf 1 2 3\n")
    assert view.import_object(bad_obj) is None
    cell = tmp_path / "celda.layout.json"
    cell.write_text("{roto")
    assert not view.load_layout(cell)
    cell.write_text('{"objects": [{"name": "a", "path": "x.stl", "inventado": 1}]}')
    assert not view.load_layout(cell)
    assert not view.save_layout(tmp_path / "no" / "existe" / "c.layout.json")
    assert sum(1 for s, _ in reports if s == "error") == 6


def test_invalid_and_duplicate_tool_rows_are_not_used(app):
    view = _view(app, "")
    table = view.tools_table
    table.set_values({2: [0, 0, 300, 0, 0, 0]})
    table.add_row(2)                                 # repetida: vale la primera
    assert table.values() == {2: [0, 0, 300, 0, 0, 0]} and table.invalid_rows() == [1]
    table.table.item(0, 3).setText("12mm")           # inválida: no se usa
    assert table.values() == {2: [0, 0, 0, 0, 0, 0]}
    assert view.layout_data.tools == table.values()  # la tabla es la fuente de verdad


def test_imported_piece_appears_in_front_of_the_robot(app, tmp_path):
    piece = tmp_path / "mesa.stl"
    write_stl(box((0, 0, 0), (800, 600, 40)), piece)
    obj = _view(app, "").import_object(piece)
    assert (obj.x, obj.y, obj.z) == (1200, 0, 20)



# --- segunda ronda de la prueba de usuario ----------------------------------------


def test_main_window_fits_a_1600_pixel_screen(app):
    from gui.main_window import MainWindow

    window = MainWindow()
    # Los paneles anchos se desplazan en vez de agrandar la ventana: el mínimo
    # tiene que ser chico en cualquier plataforma (en Windows pedía 2328 px; ahora 1100).
    assert window.minimumSizeHint().width() <= 1500  # Windows: 1100, Linux: ~630


def test_controls_are_locked_while_simulating(app):
    view = _view(app, f"MOVEJ {HOME} SPEED 50\n")
    seen = []

    def progress(_n):
        seen.append((view.model_combo.isEnabled(), view.open_backup_btn.isEnabled(),
                     view.side_tabs.isEnabled()))

    from sim import pad_sim

    original = pad_sim.PROGRESS_EVERY
    pad_sim.PROGRESS_EVERY = 1
    view._progress = progress
    try:
        view.simulate()
    finally:
        pad_sim.PROGRESS_EVERY = original
    assert seen and all(state == (False, False, False) for state in seen)
    assert view.model_combo.isEnabled() and view.open_backup_btn.isEnabled()


def test_editing_the_program_marks_the_result_stale(app):
    from gui.main_window import MainWindow

    window = MainWindow()
    window.sim_view.simulate()
    window.editor.appendPlainText("; cambio")
    assert window.sim_view.stale


def test_cell_with_an_incomplete_tool_is_rejected_cleanly(app, tmp_path):
    reports = []
    view = _view(app, "", reports)
    cell = tmp_path / "c.layout.json"
    cell.write_text('{"tools": {"2": [0, 0, "300"]}}', encoding="utf-8")
    assert not view.load_layout(cell)
    assert view.layout_data.tools == {}
    assert any(s == "error" and "6 números" in m for s, m in reports)


def test_cell_paths_are_relative_even_outside_the_cell_folder(tmp_path):
    piece = tmp_path / "piezas" / "mesa.stl"
    piece.parent.mkdir()
    write_stl(box((0, 0, 0), (10, 10, 10)), piece)
    (tmp_path / "celdas").mkdir()
    path = tmp_path / "celdas" / "c.layout.json"
    Layout(objects=[LayoutObject("mesa", str(piece))]).save(path)
    assert "../piezas/mesa.stl" in path.read_text(encoding="utf-8").replace("\\\\", "/")
    assert Layout.load(path).objects[0].path.endswith("mesa.stl")


def test_highlighting_does_not_mark_the_result_stale(app):
    from gui.main_window import MainWindow

    window = MainWindow()
    window.sim_view.simulate()
    window.highlighter.rehighlight()   # cambia formatos, no el texto
    QApplication.processEvents()
    assert not window.sim_view.stale


# --- choques --------------------------------------------------------------------------

WELD_SOURCE = f"""POINT p_pieza = WORLD(1097.1, -150.0, 721.9, 180.0, -10.0, 180.0)
MOVEJ {HOME} SPEED 80
MOVEL p_pieza + OFFSET(0, 0, 100, 0, 0, 0) SPEED 50
MOVEL p_pieza SPEED 10
MOVEJ {HOME} SPEED 80
"""


def _table_view(app, tmp_path, reports, top=730.0):
    pytest.importorskip("fcl")
    piece = tmp_path / "mesa.stl"
    write_stl(box((0, 0, top / 2), (600, 400, top)), piece)
    view = _view(app, WELD_SOURCE, reports)
    obj = view.import_object(piece)
    obj.x, obj.y, obj.z = 1100.0, -150.0, 0.0
    view._refresh_objects(rebuild=False)
    return view


def test_collision_with_a_piece_is_an_error_in_the_simulation(app, tmp_path):
    reports = []
    view = _table_view(app, tmp_path, reports)
    result = view.simulate()
    assert view.collisions is not None and view.collisions.collisions >= 1
    assert any(i.severity == "error" and "contra «mesa»" in i.message for i in result.issues)
    assert any(s == "error" and "Choque: brida (J6) contra «mesa»" in m for s, m in reports)
    assert "choque(s)" in view.summary.text()
    # Clic en el problema: va al momento del choque.
    hit = view.collisions.contacts[0]
    view.set_time(hit.time_s)
    assert view.collisions.state_at(view._t) == {0: "choque"}


def test_clear_cell_says_so_and_settings_drive_the_check(app, tmp_path):
    reports = []
    view = _table_view(app, tmp_path, reports, top=600.0)
    view.simulate()
    assert view.collisions.contacts == []
    assert "sin choques (margen 20 mm)" in view.summary.text()
    # Margen grande: ahora la mesa está "cerca". Cambiarlo deja el resultado viejo.
    view.margin_spin.setValue(200)
    assert view.stale
    view.simulate()
    assert view.collisions.near_misses >= 1 and view.collisions.collisions == 0
    # Sin buscar choques no hay reporte.
    view.collisions_check.setChecked(False)
    view.simulate()
    assert view.collisions is None and "choques sin revisar" in view.summary.text()


def test_moving_a_piece_marks_the_result_stale(app, tmp_path):
    view = _table_view(app, tmp_path, [], top=600.0)
    view.simulate()
    assert not view.stale
    view.objects_table.item(0, 1).setText("1300")
    assert view.stale


def test_workpiece_column_and_tool_mesh(app, tmp_path):
    from gui.sim_view import WORKPIECE_COLUMN
    from PySide6.QtCore import Qt

    reports = []
    view = _table_view(app, tmp_path, reports, top=680.0)
    torch = tmp_path / "antorcha.stl"
    write_stl(cylinder((0, 0, 0), (0, 0, 30), 12), torch)
    assert view.set_tool_mesh(torch)
    assert view.tool_mesh_label.text() == "antorcha.stl"
    view.simulate()
    assert any("herramienta" in c.parts and not c.colliding for c in view.collisions.contacts)

    view.objects_table.item(0, WORKPIECE_COLUMN).setCheckState(Qt.CheckState.Checked)
    assert view.layout_data.objects[0].workpiece and view.stale
    view.simulate()
    assert not any("herramienta" in c.parts for c in view.collisions.contacts)

    assert view.set_tool_mount("0; 0; 10; 0; 0; 0")
    assert view.layout_data.tool_mount == [0, 0, 10, 0, 0, 0]
    assert not view.set_tool_mount("1, 2, 3")
    assert view.layout_data.tool_mount == [0, 0, 10, 0, 0, 0]
    assert any("Montaje" in m for _s, m in reports)
    assert view.set_tool_mesh("")
    assert view.layout_data.tool_mesh == ""


def test_collision_settings_travel_with_the_cell(app, tmp_path):
    view = _table_view(app, tmp_path, [], top=600.0)
    torch = tmp_path / "antorcha.stl"
    write_stl(cylinder((0, 0, 0), (0, 0, 30), 12), torch)
    view.set_tool_mesh(torch)
    view.set_tool_mount("0, 0, 5, 0, 0, 90")
    view.margin_spin.setValue(45)
    path = tmp_path / "celda.layout.json"
    view.save_layout(path)

    other = _view(app, "")
    assert other.load_layout(path)
    assert other.margin_spin.value() == 45
    assert other.layout_data.tool_mesh == str(torch)
    assert other.tool_mount_edit.text() == "0, 0, 5, 0, 0, 90"
    assert other.collisions_check.isChecked()


def test_missing_piece_file_does_not_stop_the_collision_check(app, tmp_path):
    reports = []
    view = _table_view(app, tmp_path, reports)
    view.layout_data.objects.append(LayoutObject("fantasma", str(tmp_path / "no.stl")))
    result = view.simulate()
    assert result is not None and view.collisions is not None
    assert any(s == "warning" and "«fantasma» no se revisa" in m for s, m in reports)
