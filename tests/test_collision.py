"""
Choques del robot contra la celda (sim/collision.py, con python-fcl).

Todo sobre el BRTIRUS1820A con eslabones aproximados (cilindros): lo que se
prueba es el método (que no se escape ningún choque, que las piezas sean
sólidos, el margen), no la forma exacta del robot real.
"""

import math

import pytest

pytest.importorskip("fcl")

from compiler.pad_codegen import PadOptions, compile_to_pad  # noqa: E402
from sim import collision  # noqa: E402
from sim.collision import FLOOR, TOOL_PART, CollisionChecker, Obstacle  # noqa: E402
from sim.kinematics import RobotModel, identity, rot_axis  # noqa: E402
from sim.meshes import box, cylinder  # noqa: E402
from sim.pad_sim import Cancelled, Segment, SimResult, simulate  # noqa: E402
from sim.scene import Layout, LayoutObject, robot_link_meshes  # noqa: E402

HOME = [0.347, 45.894, -44.865, -0.792, -75.952, -0.859]
# Cordón de 300 mm con la brida apuntando abajo a z = 721.9 (el ejemplo de la GUI).
WELD = """POINT p_home = JOINT(0.347, 45.894, -44.865, -0.792, -75.952, -0.859)
POINT p_pieza = WORLD(1097.1, -150.0, 721.9, 180.0, -10.0, 180.0)
MOVEJ p_home SPEED 80
MOVEL p_pieza + OFFSET(0, 0, 100, 0, 0, 0) SPEED 50
MOVEL p_pieza SPEED 10
MOVEL p_pieza + OFFSET(0, 300, 0, 0, 0, 0) SPEED 5
MOVEL p_pieza + OFFSET(0, 300, 100, 0, 0, 0) SPEED 50
MOVEJ p_home SPEED 80
"""


@pytest.fixture(scope="module")
def model():
    return RobotModel.load("BRTIRUS1820A")


@pytest.fixture(scope="module")
def links(model):
    return robot_link_meshes(model, tool_axis=False)


@pytest.fixture(scope="module")
def weld(model):
    return simulate(compile_to_pad(WELD, PadOptions(allow_unverified=True)), model)


def table(top: float) -> Obstacle:
    """Mesa de 600 x 400 debajo del cordón, con la tapa a `top` mm."""
    return Obstacle("Mesa", box((1100, -150, top / 2), (600, 400, top)), identity())


def moves(*poses: list[float], kind: str = "MOVEJ") -> SimResult:
    """Un tramo por par de poses, interpolado en ejes cada 2° como el simulador."""
    result = SimResult(model="BRTIRUS1820A")
    for i, (a, b) in enumerate(zip(poses, poses[1:])):
        n = max(2, int(max(abs(y - x) for x, y in zip(a, b)) / 2) + 1)
        samples = [[x + (y - x) * k / (n - 1) for x, y in zip(a, b)] for k in range(n)]
        result.segments.append(Segment(kind, f"MAIN[{i}]", samples, 1.0,
                                       times=[k / (n - 1) for k in range(n)]))
    return result


def jump(a: list[float], b: list[float]) -> SimResult:
    """Un solo tramo de dos muestras: todo el giro sin nada en el medio."""
    result = SimResult(model="BRTIRUS1820A")
    result.segments.append(Segment("MOVEJ", "MAIN[0]", [a, b], 1.0, times=[0.0, 1.0]))
    return result


# --- piezas ------------------------------------------------------------------------------


def test_clear_cell_reports_nothing(model, links, weld):
    report = CollisionChecker(model, links, [table(680)], margin_mm=20).check(weld)
    assert report.contacts == [] and report.issues == []
    assert report.evaluations > 0


def test_closer_than_the_margin_is_a_warning(model, links, weld):
    # Brida a 721.9 inclinada 10°: el borde (radio 38) baja 6.6 mm -> ~5 mm de la mesa.
    report = CollisionChecker(model, links, [table(710)], margin_mm=20).check(weld)
    assert report.collisions == 0 and report.near_misses >= 1
    near = report.contacts[0]
    assert near.obstacle == "Mesa" and "brida (J6)" in near.parts
    assert near.distance_mm == pytest.approx(5.3, abs=1.0)
    warnings = [i for i in report.issues if i.severity == "aviso"]
    assert "pasa a 5 mm de «Mesa»" in warnings[0].message and "margen 20 mm" in warnings[0].message
    # Y con un margen menor ya no molesta.
    assert CollisionChecker(model, links, [table(710)], margin_mm=3).check(weld).contacts == []


def test_touching_is_an_error_with_the_moment(model, links, weld):
    report = CollisionChecker(model, links, [table(730)], margin_mm=20).check(weld)
    hit = next(c for c in report.contacts if c.colliding)
    assert hit.segment == 2 and hit.kind == "MOVEL"   # la bajada al cordón
    # La bajada empieza en z = 821.9 y la mesa está a 730 (+6.6 del borde de la brida):
    # el choque es cerca del 80 % de la recta, no al final.
    assert 0.6 < hit.fraction < 0.95
    errors = [i for i in report.issues if i.severity == "error"]
    assert errors[0].message.startswith("Choque: brida (J6) contra «Mesa» al ")
    assert errors[0].time_s == pytest.approx(hit.time_s)
    # Aviso de que el robot es aproximado (sin mallas del fabricante).
    assert any(i.severity == "info" and "aproximados" in i.message for i in report.issues)


def test_issue_time_points_into_the_segment(model, links, weld):
    report = CollisionChecker(model, links, [table(730)], margin_mm=0).check(weld)
    hit = report.contacts[0]
    start = sum(s.duration_s for s in weld.segments[:hit.segment])
    assert start <= hit.time_s <= start + weld.segments[hit.segment].duration_s
    assert report.state_at(hit.time_s) == {0: "choque"}
    assert report.state_at(0.0) == {}


def test_a_fast_sweep_between_two_samples_still_hits(model, links):
    # J1 de -60° a 60° con SOLO los dos extremos: el poste en 0° queda en el
    # medio y en ninguna de las dos muestras el brazo está cerca (la brida, en
    # HOME, está a x = 509).
    post = Obstacle("Poste", cylinder((480, 0, 0), (480, 0, 2000), 40), identity())
    a, b = [-60.0] + HOME[1:], [60.0] + HOME[1:]
    report = CollisionChecker(model, links, [post], margin_mm=0).check(jump(a, b))
    hit = next(c for c in report.contacts if c.colliding)
    assert hit.obstacle == "Poste"
    assert report.evaluations > 2          # hubo que mirar entre las dos muestras
    assert hit.fraction == pytest.approx(0.5, abs=0.15)  # el poste está a mitad de camino


def test_pieces_are_solids_not_just_surfaces(model, links):
    # Una caja enorme con el brazo entero adentro: ningún triángulo se toca,
    # pero el robot está metido en la pieza.
    big = Obstacle("Jaula", box((800, 0, 1000), (4000, 4000, 1900)), identity())
    a = HOME
    b = [10.0] + HOME[1:]
    report = CollisionChecker(model, links, [big], margin_mm=0, floor=False).check(moves(a, b))
    inside = [c for c in report.contacts if c.inside]
    assert inside and inside[0].colliding
    assert "queda adentro de «Jaula»" in report.issues[0].message


def test_placement_of_the_piece_counts(model, links):
    post = cylinder((0, 0, 0), (0, 0, 2000), 40)
    a, b = [-60.0] + HOME[1:], [60.0] + HOME[1:]
    turned = [r + [t] for r, t in zip(rot_axis((0, 0, 1), math.radians(90)), (0, 480, 0))]
    turned.append([0, 0, 0, 1])
    # En (0, 480) el poste queda a 90°: fuera del barrido de -60° a 60°.
    away = Obstacle("Poste", post, turned)
    assert CollisionChecker(model, links, [away], margin_mm=0).check(jump(a, b)).contacts == []
    there = Obstacle("Poste", post, [[1, 0, 0, 480], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    assert CollisionChecker(model, links, [there], margin_mm=0).check(jump(a, b)).collisions >= 1


# --- piso y el robot consigo mismo -----------------------------------------------------------


def test_reaching_below_the_base_hits_the_floor(model, links):
    low = [0.0, -80.0, -25.0, 0.0, -90.0, 0.0]   # la brida queda a z ≈ -160
    report = CollisionChecker(model, links, [], margin_mm=20).check(moves(HOME, low))
    floor = [c for c in report.contacts if c.obstacle == FLOOR]
    assert floor and floor[0].colliding
    assert "contra el piso" in report.issues[0].message
    assert CollisionChecker(model, links, [], margin_mm=20, floor=False).check(
        moves(HOME, low)).contacts == []


def test_folding_the_arm_onto_itself_is_reported(model, links):
    folded = [0.0, -95.0, -80.0, 0.0, 0.0, 0.0]  # el antebrazo se mete en la base
    report = CollisionChecker(model, links, [], margin_mm=20, floor=False).check(moves(HOME, folded))
    selfhit = [c for c in report.contacts if c.obstacle in ("base", "columna (J1)")]
    assert selfhit and selfhit[0].colliding
    assert any("consigo mismo" in i.message for i in report.issues)


def test_normal_poses_have_no_self_collision(model, links):
    checker = CollisionChecker(model, links, [], margin_mm=20)
    for q in (HOME, [0.0] * 6, [90.0, 30.0, -30.0, 45.0, 60.0, 90.0]):
        assert all(d > 0 for _a, _b, d in checker.distances_at(q))


# --- herramienta montada ---------------------------------------------------------------------


def test_tool_mesh_is_mounted_on_the_flange(model):
    tip = cylinder((0, 0, 0), (0, 0, 100), 5)
    mounted = collision.tool_in_flange_zero(model, tip, [0, 0, 0, 0, 0, 0])
    (lo, hi) = mounted.bounds()
    flange_x = model.a1 + model.d4 + model.d6
    assert lo[0] == pytest.approx(flange_x, abs=0.1) and hi[0] == pytest.approx(flange_x + 100, abs=0.1)
    shifted = collision.tool_in_flange_zero(model, tip, [0, 0, 50, 0, 0, 0]).bounds()
    assert shifted[1][0] == pytest.approx(flange_x + 150, abs=0.1)


def test_tool_reaches_further_than_the_flange(model, links, weld):
    torch = cylinder((0, 0, 0), (0, 0, 60), 12)   # 60 mm más abajo que la brida
    clear = CollisionChecker(model, links, [table(680)], margin_mm=0).check(weld)
    assert clear.contacts == []
    report = CollisionChecker(model, links, [table(680)], tool_mesh=torch,
                              margin_mm=0).check(weld)
    assert any(c.colliding and TOOL_PART in c.parts for c in report.contacts)


def test_workpiece_only_counts_contact_for_the_tool(model, links, weld):
    torch = cylinder((0, 0, 0), (0, 0, 30), 12)   # punta a ~722-30 = 692 mm
    piece = table(680)                            # ~12 mm debajo de la punta
    near = CollisionChecker(model, links, [piece], tool_mesh=torch, margin_mm=20).check(weld)
    assert any(TOOL_PART in c.parts and not c.colliding for c in near.contacts)
    piece.workpiece = True
    ok = CollisionChecker(model, links, [piece], tool_mesh=torch, margin_mm=20).check(weld)
    assert not any(TOOL_PART in c.parts for c in ok.contacts)
    # El brazo sigue usando el margen contra la pieza que se trabaja.
    piece_high = Obstacle("Mesa", piece.mesh, identity(), workpiece=True)
    piece_high.mesh = table(710).mesh
    arm = CollisionChecker(model, links, [piece_high], tool_mesh=None, margin_mm=20).check(weld)
    assert any("brida (J6)" in c.parts for c in arm.contacts)


# --- varios ---------------------------------------------------------------------------------


def test_jumps_and_waits_are_not_swept(model, links):
    # Un SALTO (recuperación del simulador) no es movimiento: no se barre.
    post = Obstacle("Poste", cylinder((480, 0, 0), (480, 0, 2000), 40), identity())
    a, b = [-60.0] + HOME[1:], [60.0] + HOME[1:]
    result = SimResult(model="BRTIRUS1820A")
    result.segments.append(Segment("SALTO", "MAIN[0]", [a, b], 0.0, times=[0.0, 0.0]))
    result.segments.append(Segment("WAIT", "MAIN[1]", [b], 2.0))
    assert CollisionChecker(model, links, [post], margin_mm=0).check(result).contacts == []


def test_progress_can_cancel(model, links, weld):
    def progress(_n):
        raise Cancelled

    checker = CollisionChecker(model, links, [table(730)], margin_mm=20)
    collision_every = collision.PROGRESS_EVERY
    collision.PROGRESS_EVERY = 1
    try:
        with pytest.raises(Cancelled):
            checker.check(weld, progress=progress)
    finally:
        collision.PROGRESS_EVERY = collision_every


def test_needs_seven_link_meshes(model, links):
    with pytest.raises(ValueError):
        CollisionChecker(model, links[:6], [])


def test_layout_keeps_the_collision_settings(tmp_path):
    from sim.meshes import write_stl

    torch = tmp_path / "antorcha.stl"
    write_stl(cylinder((0, 0, 0), (0, 0, 300), 15), torch)
    layout = Layout(objects=[LayoutObject("mesa", str(torch), workpiece=True)],
                    tool_mesh=str(torch), tool_mount=[0, 0, 10, 0, 0, 45], margin_mm=35,
                    collisions=False)
    path = tmp_path / "celda.layout.json"
    layout.save(path)
    assert '"tool_mesh": "antorcha.stl"' in path.read_text(encoding="utf-8")
    again = Layout.load(path)
    assert again.tool_mesh == str(torch) and again.tool_mount == [0, 0, 10, 0, 0, 45]
    assert again.margin_mm == 35 and again.collisions is False
    assert again.objects[0].workpiece is True


def test_old_layouts_load_with_defaults(tmp_path):
    path = tmp_path / "vieja.layout.json"
    path.write_text('{"model": "BRTIRUS1820A", "objects": []}', encoding="utf-8")
    layout = Layout.load(path)
    assert layout.tool_mesh == "" and layout.tool_mount == [0.0] * 6
    assert layout.margin_mm == 20.0 and layout.collisions is True


@pytest.mark.parametrize("bad", ['"tool_mount": [1, 2, 3]', '"margin_mm": -5',
                                 '"tool_mount": [0, 0, "NaN", 0, 0, 0]'])
def test_bad_collision_settings_are_rejected(tmp_path, bad):
    path = tmp_path / "mala.layout.json"
    path.write_text('{"objects": [], ' + bad + '}', encoding="utf-8")
    with pytest.raises(ValueError):
        Layout.load(path)


def test_check_cli_with_a_cell_reports_collisions(tmp_path, capsys):
    from sim.check import main as check_main
    from sim.meshes import write_stl

    src = tmp_path / "p.krlb"
    src.write_text(WELD, encoding="utf-8")
    mesa = tmp_path / "mesa.stl"
    write_stl(box((0, 0, 365), (600, 400, 730)), mesa)
    cell = tmp_path / "celda.layout.json"
    Layout(model="BRTIRUS1820A",
           objects=[LayoutObject("mesa", str(mesa), x=1100, y=-150)]).save(cell)
    assert check_main([str(src), "--layout", str(cell)]) == 1
    assert "Choque: brida (J6) contra «mesa»" in capsys.readouterr().out

    broken = tmp_path / "rota.layout.json"
    broken.write_text("{", encoding="utf-8")
    assert check_main([str(src), "--layout", str(broken)]) == 2
    assert "no se pudo leer la celda" in capsys.readouterr().err
