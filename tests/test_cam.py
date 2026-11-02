"""
Fresado (sim/cam.py): área 2D -> pasadas sobre la superficie -> programa.
"""

import math

import numpy as np
import pytest

from compiler import program_files as pf
from sim.cam import (
    CamError,
    CamParams,
    CutterTool,
    _dilate_coarse,
    _inside,
    build_heightmap,
    compute_path,
    generate_program,
    offset_polygon,
    solve_robot,
    tip_map,
)
from sim.kinematics import RobotModel, identity
from sim.meshes import Mesh, box, load_mesh
from sim.pad_sim import simulate

SQUARE = [(-40.0, -40.0), (40.0, -40.0), (40.0, 40.0), (-40.0, 40.0)]


def dome(radius: float = 300.0, height: float = 40.0, half: float = 100.0, n: int = 80) -> Mesh:
    """Casquete esférico sobre un cuadrado: pieza de muchos polígonos, con la
    altura conocida en cualquier punto."""
    def z(x, y):
        return math.sqrt(radius ** 2 - x ** 2 - y ** 2) - radius + height

    mesh = Mesh()
    xs = [-half + 2 * half * i / n for i in range(n + 1)]
    for i in range(n):
        for j in range(n):
            a = (xs[i], xs[j], z(xs[i], xs[j]))
            b = (xs[i + 1], xs[j], z(xs[i + 1], xs[j]))
            c = (xs[i + 1], xs[j + 1], z(xs[i + 1], xs[j + 1]))
            d = (xs[i], xs[j + 1], z(xs[i], xs[j + 1]))
            mesh.add(a, b, c)
            mesh.add(a, c, d)
    return mesh


def test_heightmap_of_a_box_has_its_top():
    hm = build_heightmap(box((10, 20, 25), (100, 60, 50)), identity(), "Arriba", res=2.0)
    assert hm.lo == pytest.approx((-40, -10))
    assert hm.h[hm.h.shape[0] // 2, hm.h.shape[1] // 2] == pytest.approx(50)


def test_view_from_the_front_sees_the_x_face():
    hm = build_heightmap(box((0, 0, 0), (100, 60, 50)), identity(), "Frente (+X)", res=2.0)
    assert hm.h.max() == pytest.approx(50)     # cara en X = +50
    assert hm.to_cell(np.array([[0.0, 0.0, 50.0]]))[0] == pytest.approx((50, 0, 0))


def test_piece_placement_moves_the_map():
    m = identity()
    m[0][3], m[2][3] = 500.0, 100.0
    hm = build_heightmap(box((0, 0, 25), (100, 60, 50)), m, "Arriba", res=2.0)
    assert hm.lo[0] == pytest.approx(450)
    assert hm.h.max() == pytest.approx(150)


def test_huge_mesh_size_stays_under_the_cell_cap():
    hm = build_heightmap(box((0, 0, 0), (5000, 5000, 10)), identity(), "Arriba", res=0.1)
    assert hm.h.size <= 6_100_000


def test_ball_rests_on_the_edge_of_a_step_without_digging():
    """Fresa de bola R5 al lado de un escalón de 20 mm: la punta sube antes de
    llegar, justo por el perfil de la bola."""
    mesh = box((-50, 0, 5), (100, 100, 10)).extend(box((25, 0, 20), (50, 100, 40)))
    hm = build_heightmap(mesh, identity(), "Arriba", res=0.25).padded(10)
    tip = tip_map(hm, CutterTool("bola", 10.0))
    row = hm.h.shape[0] // 2
    at = lambda x: tip[row, int(round((x - hm.lo[0]) / hm.res))]
    assert at(-20) == pytest.approx(10, abs=0.01)      # lejos: sobre el piso
    assert at(-6) == pytest.approx(10, abs=0.01)       # todavía no toca
    assert at(-2.5) == pytest.approx(40 - (5 - math.sqrt(25 - 2.5 ** 2)), abs=0.1)  # sube con la bola
    assert at(20) == pytest.approx(40, abs=0.01)       # arriba del escalón


def test_flat_tool_never_goes_below_the_highest_point_under_it():
    mesh = box((0, 0, 5), (100, 100, 10)).extend(box((0, 0, 11), (4, 4, 2)))
    hm = build_heightmap(mesh, identity(), "Arriba", res=0.5).padded(8)
    tip = tip_map(hm, CutterTool("plana", 10.0))
    x, y = int(round((4 - hm.lo[0]) / hm.res)), int(round((0 - hm.lo[1]) / hm.res))
    assert tip[y, x] == pytest.approx(12)


def test_coarse_holder_dilation_never_misses_material():
    h = np.full((200, 200), -np.inf)
    h[100, 100] = 5.0
    out = _dilate_coarse(h, 1.0, 60.0)
    assert out[100, 45] == 5.0 and out[100, 155] == 5.0


def test_zigzag_stays_inside_the_area_by_the_tool_radius():
    hm = build_heightmap(dome(), identity(), res=1.0)
    path = compute_path(hm, [SQUARE], CutterTool("bola", 10.0), CamParams(stepover=5.0))
    cut = path.pos[[k != "rapid" for k in path.kinds]]
    assert cut[:, 0].min() >= -35.01 and cut[:, 0].max() <= 35.01
    assert cut[:, 1].min() >= -35.01 and cut[:, 1].max() <= 35.01


def test_path_is_extrapolated_onto_the_curved_surface():
    """Cada punto de corte queda a la altura real del casquete, no en un plano."""
    hm = build_heightmap(dome(), identity(), res=0.5)
    path = compute_path(hm, [SQUARE], CutterTool("bola", 10.0), CamParams(stepover=7.0))
    cut = path.pos[[k == "cut" for k in path.kinds]]
    for x, y, z in cut[::10]:
        # Offset de una esfera: centro de la bola a R + 5 del centro de la esfera.
        centre = math.sqrt(305.0 ** 2 - x ** 2 - y ** 2) - 300.0 + 40.0
        assert z == pytest.approx(centre - 5.0, abs=0.15)
    assert cut[:, 2].max() - cut[:, 2].min() > 3      # de verdad sigue la curva


def test_depth_makes_levels_and_reaches_the_total():
    hm = build_heightmap(box((0, 0, 25), (200, 200, 50)), identity(), res=1.0)
    params = CamParams(stepover=5.0, depth=5.0, stepdown=2.0)
    path = compute_path(hm, [SQUARE], CutterTool("plana", 10.0), params)
    levels = sorted({round(float(z), 3) for z, k in zip(path.pos[:, 2], path.kinds) if k == "cut"})
    assert levels == [45.0, 46.0, 48.0]


def test_a_flat_surface_needs_few_points():
    hm = build_heightmap(box((0, 0, 25), (200, 200, 50)), identity(), res=0.5)
    path = compute_path(hm, [SQUARE], CutterTool("plana", 10.0), CamParams(stepover=10.0))
    assert len(path) < 40


def test_concave_area_does_not_cut_the_notch():
    l_shape = [(-50, -50), (50, -50), (50, 0), (0, 0), (0, 50), (-50, 50)]
    hm = build_heightmap(box((0, 0, 25), (200, 200, 50)), identity(), res=1.0)
    path = compute_path(hm, [l_shape], CutterTool("plana", 10.0), CamParams(stepover=5.0))
    cut = path.pos[[k != "rapid" for k in path.kinds]]
    assert not np.any((cut[:, 0] > 0) & (cut[:, 1] > 0))


def test_contour_is_offset_by_the_tool_radius():
    hm = build_heightmap(box((0, 0, 25), (200, 200, 50)), identity(), res=1.0)
    params = CamParams(strategy="contorno", side="interior")
    path = compute_path(hm, [SQUARE], CutterTool("plana", 10.0), params)
    cut = path.pos[[k != "rapid" for k in path.kinds]]
    assert np.abs(cut[:, :2]).max() == pytest.approx(35, abs=0.01)


def test_contour_too_narrow_is_an_error():
    with pytest.raises(CamError, match="no entra"):
        offset_polygon([(0, 0), (6, 0), (6, 100), (0, 100)], -5.0)


def test_several_areas_are_all_cut():
    hm = build_heightmap(box((0, 0, 25), (400, 200, 50)), identity(), res=1.0)
    left = [(-150, -30), (-90, -30), (-90, 30), (-150, 30)]
    right = [(90, -30), (150, -30), (150, 30), (90, 30)]
    path = compute_path(hm, [left, right], CutterTool("plana", 10.0), CamParams(stepover=8.0))
    assert path.pos[:, 0].min() < -80 and path.pos[:, 0].max() > 80
    assert not np.any(np.abs(path.pos[[k == "cut" for k in path.kinds], 0]) < 80)


def test_area_outside_the_piece_makes_no_passes():
    hm = build_heightmap(box((0, 0, 25), (100, 100, 50)), identity(), res=1.0)
    away = [(500, 500), (600, 500), (600, 600), (500, 600)]
    with pytest.raises(CamError, match="no pasa por encima"):
        compute_path(hm, [away], CutterTool("plana", 10.0), CamParams())


def test_flat_tool_cannot_follow_the_normal():
    hm = build_heightmap(dome(), identity(), res=2.0)
    with pytest.raises(CamError, match="bola"):
        compute_path(hm, [SQUARE], CutterTool("plana", 10.0), CamParams(axis_mode="normal"))


def test_normal_axis_leans_with_the_surface_and_keeps_the_ball_centre():
    hm = build_heightmap(dome(), identity(), res=0.5)
    area = [(40.0, -10.0), (80.0, -10.0), (80.0, 10.0), (40.0, 10.0)]
    params = CamParams(axis_mode="normal", stepover=5.0)
    path = compute_path(hm, [area], CutterTool("bola", 10.0), params)
    cuts = [i for i, k in enumerate(path.kinds) if k == "cut"]
    p, a = path.pos[cuts[0]], path.axis[cuts[0]]
    expected = -np.array((p[0], p[1], 0.0 + math.sqrt(300.0 ** 2 - p[0] ** 2 - p[1] ** 2))) / 300.0
    assert np.degrees(np.arccos(np.clip(a @ expected, -1, 1))) < 1.5
    centre = p - a * 5.0          # centro de la bola, a 305 del centro de la esfera
    assert np.linalg.norm(centre - (0, 0, -260.0)) == pytest.approx(305, abs=0.2)


def test_holder_limits_the_depth_and_warns():
    """Bolsillo hondo con filo corto: la porta apoya en la superficie de arriba."""
    hm = build_heightmap(box((0, 0, 25), (300, 300, 50)), identity(), res=1.0)
    tool = CutterTool("plana", 10.0, cut_length=5.0, holder_diameter=30.0)
    path = compute_path(hm, [SQUARE], tool, CamParams(stepover=8.0, depth=12.0, stepdown=12.0))
    assert path.pos[[k == "cut" for k in path.kinds], 2].min() == pytest.approx(45)
    assert any("porta" in w for w in path.warnings)


def test_inside_handles_concave_polygons():
    poly = np.array([(0, 0), (10, 0), (10, 10), (5, 3), (0, 10)], dtype=float)
    u, v = np.array([5.0, 5.0, 2.0]), np.array([1.0, 8.0, 2.0])
    assert _inside(poly, u, v).tolist() == [True, False, True]


# --- del robot al programa ---------------------------------------------------------------------


@pytest.fixture(scope="module")
def model():
    return RobotModel.load("BRTIRUS1510A")


def placed(x=850.0, z=300.0):
    m = identity()
    m[0][3], m[2][3] = x, z
    return m


def job(model, mesh, matrix, polygons, tool=CutterTool("bola", 10.0), params=CamParams(stepover=20.0),
        base=(0, 0, 0, 0, 0, 0), tool_pose=(0, 0, 100, 0, 0, 0)):
    hm = build_heightmap(mesh, matrix, res=1.0)
    path = compute_path(hm, polygons, tool, params)
    return solve_robot(path, model, base, tool_pose, (0, 0, 0, 0, 0, 0), params.roll_deg)


AREA = [(810.0, -40.0), (890.0, -40.0), (890.0, 40.0), (810.0, 40.0)]


def test_generated_program_compiles_and_simulates_on_the_surface(model):
    robot = job(model, dome(), placed(), [AREA])
    assert robot.unreachable == []
    params = CamParams(stepover=20.0, tool_number=1)
    sources = generate_program(robot, CutterTool("bola", 10.0), params)
    backup, _warnings = pf.compile_pad(sources)

    result = simulate(backup, model, tools={1: [0, 0, 100, 0, 0, 0]})
    assert [i for i in result.issues if i.severity == "error"] == []
    assert result.skipped_moves == 0

    # La punta simulada pasa por los puntos pedidos.
    wanted = robot.path.hm.to_cell(robot.path.pos)[-2]
    segment = [s for s in result.segments if s.kind == "MOVEL"][-2]
    flange = np.array(model.fk(segment.samples[-1]))
    tip = (flange @ np.array(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 100), (0, 0, 0, 1.0))))[:3, 3]
    assert tip == pytest.approx(wanted, abs=0.05)


def test_robot_standing_elsewhere_gets_points_in_its_own_base(model):
    base = (1000.0, 0.0, 0.0, 0.0, 0.0, 180.0)   # mirando hacia -X de la celda
    moved = placed(x=150.0)
    area = [(110.0, -40.0), (190.0, -40.0), (190.0, 40.0), (110.0, 40.0)]
    robot = job(model, dome(), moved, [area], base=base)
    assert robot.unreachable == []
    assert robot.poses[0][0] > 0        # adelante del robot, aunque en la celda esté atrás


def test_unreachable_piece_gives_an_error_not_a_program(model):
    far = placed(x=5000.0)
    area = [(4960.0, -40.0), (5040.0, -40.0), (5040.0, 40.0), (4960.0, 40.0)]
    with pytest.raises(CamError, match="no llega"):
        job(model, dome(), far, [area])


def test_partly_unreachable_refuses_to_generate(model):
    robot = job(model, dome(), placed(), [AREA])
    robot.unreachable = [3, 4]
    with pytest.raises(CamError, match="no alcanza 2 de"):
        generate_program(robot, CutterTool("bola", 10.0), CamParams())


def test_spindle_output_wraps_the_program(model):
    robot = job(model, dome(), placed(), [AREA])
    params = CamParams(stepover=20.0, spindle_output="Y034", spindle_wait_s=2.0)
    sources = generate_program(robot, CutterTool("bola", 10.0), params)
    lines = sources.src.text.splitlines()
    assert "SET_OUT(Y034, ON)" in lines and "WAIT 2 s" in lines
    assert lines.index("SET_OUT(Y034, ON)") < lines.index("SET_OUT(Y034, OFF)")
    pf.compile_pad(sources)


# --- pestaña Fresado (sin vista 3D, como todos los tests de GUI) ------------------------------


@pytest.fixture(scope="module")
def app():
    from PySide6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


@pytest.fixture()
def panel(app, tmp_path, model):
    from PySide6.QtCore import QEvent

    from gui.cam_panel import CamPanel
    from sim.meshes import write_stl
    from sim.scene import Layout, LayoutObject

    stl = tmp_path / "pieza.stl"
    write_stl(dome(n=30), stl)
    cell = Layout(tools={1: [0, 0, 100, 0, 0, 0]})
    cell.objects.append(LayoutObject("Pieza", str(stl), x=850.0, z=300.0))
    programs, previews, reports = [], [], []

    def accept(sources):
        programs.append(sources)
        return True

    p = CamPanel(lambda: cell, lambda: model, load_mesh,
                 accept, lambda pts, colors: previews.append(pts),
                 lambda severity, text: reports.append((severity, text)))
    p.resize(400, 400)
    p.cell, p.programs, p.previews, p.reports = cell, programs, previews, reports
    p.refresh()
    yield p
    # Que no quede para el cierre del proceso: un widget destruido después de la
    # QApplication hace caer a Python en Windows, con todos los tests ya pasados.
    p.shutdown()
    p.deleteLater()
    app.sendPostedEvents(None, QEvent.Type.DeferredDelete)


def click(canvas, button, u, v):
    from PySide6.QtCore import QPoint
    from PySide6.QtTest import QTest

    at = canvas.to_screen(u, v)
    QTest.mouseClick(canvas, button, pos=QPoint(round(at.x()), round(at.y())))


def test_panel_draws_an_area_and_generates_the_program(panel):
    from PySide6.QtCore import Qt

    panel.canvas.resize(400, 400)
    for u, v in AREA:
        click(panel.canvas, Qt.MouseButton.LeftButton, u, v)
    click(panel.canvas, Qt.MouseButton.RightButton, 0, 0)
    assert len(panel.canvas.polygons) == 1
    assert panel.canvas.polygons[0][0] == pytest.approx(AREA[0], abs=3)

    panel.stepover.setValue(20)
    panel.tool_number.setValue(1)
    panel.calculate(synchronous=True)
    assert panel.code_btn.isEnabled(), panel.status.text()
    assert panel.previews and len(panel.previews[0]) == len(panel.robot_path.path)

    panel.generate()
    assert panel.programs
    pf.compile_pad(panel.programs[0])
    assert panel.cell.objects[0].machined


def test_panel_calculates_in_its_thread_and_lets_it_finish(panel):
    from PySide6.QtTest import QTest

    panel.canvas.polygons = [AREA]
    panel.stepover.setValue(20)
    panel.calculate()
    for _ in range(600):
        if panel._worker is None and panel.robot_path is not None:
            break
        QTest.qWait(50)
    assert panel.robot_path is not None and panel._worker is None
    assert panel.calc_btn.isEnabled()


def test_panel_missing_tool_is_reported_not_guessed(panel):
    panel.canvas.whole_piece()
    panel.tool_number.setValue(7)
    panel.calculate(synchronous=True)
    assert not panel.code_btn.isEnabled()
    assert "herramienta 7" in panel.status.text()


def test_panel_redraws_when_the_piece_moves(panel):
    before = panel.canvas.hm.lo[0]
    panel.cell.objects[0].x += 100
    panel.refresh()
    assert panel.canvas.hm.lo[0] == pytest.approx(before + 100)


def test_machined_piece_is_saved_with_the_cell(tmp_path):
    from sim.scene import Layout, LayoutObject

    path = tmp_path / "celda.json"
    cell = Layout()
    cell.objects.append(LayoutObject("Pieza", str(tmp_path / "p.stl"), machined=True))
    cell.save(path)
    assert Layout.load(path).objects[0].machined


def test_tool_does_not_collide_with_the_machined_piece(model):
    from sim import collision
    from sim.scene import robot_link_meshes

    if not collision.available():
        pytest.skip("Falta python-fcl")
    def checker(machined):
        piece = collision.Obstacle("Pieza", box((800, 0, 300), (300, 300, 300)), identity(), True, machined)
        return collision.CollisionChecker(model, robot_link_meshes(model, tool_axis=False), [piece],
                                          tool_mesh=box((0, 0, 50), (20, 20, 100)))
    tool_pairs = lambda c: [p for p in c.pairs if p[0].name == collision.TOOL_PART and hasattr(p[1], "machined")]
    assert len(tool_pairs(checker(False))) == 1
    assert tool_pairs(checker(True)) == []
