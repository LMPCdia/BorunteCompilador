"""
Ensamble STEP del fabricante -> modelo de robot (sim/step_assembly.py y
sim/robot_import.py).

El STEP real del 1510A pesa 55 MB y es del fabricante: no va al repo. Acá se
prueba el método con un "ensamble" sintético armado con el 1820A en una pose
conocida, que el importador tiene que recuperar (cotas y pose).
"""

import math

import pytest

from sim.kinematics import RobotModel, mat_mul
from sim.meshes import box
from sim.robot_import import import_robot
from sim.scene import robot_link_meshes
from sim.step_assembly import Part, StepFile, _apply, _decode, split_assembly

CODES = ["A000", "B000", "C000", "D000", "E000", "F000", None]
# Pose del "ensamble" (grados geométricos, como gira cada eje en el CAD).
CAD_POSE = [-1.25, 3.5, -3.5, 72.0, 10.0, 20.0]
# CAD con Y para arriba (como el del fabricante): robot -> CAD.
TO_CAD = [[1.0, 0, 0, 0], [0, 0, 1.0, 0], [0, -1.0, 0, 0], [0, 0, 0, 1.0]]


@pytest.fixture(scope="module")
def model():
    return RobotModel.load("BRTIRUS1820A")


def synthetic_parts(model, pose_deg, lateral=0.0):
    """Eslabones del 1820A ubicados como los dejaría un CAD en `pose_deg`.
    Cada parte se dibuja en sus coordenadas (las de la posición cero) y su
    matriz la lleva al ensamble."""
    # joint_frames aplica el sentido de giro del modelo: se lo deshace para
    # que la pose sea geométrica.
    q = [a * j.sign for a, j in zip(pose_deg, model.joints)]
    frames = model.joint_frames(q)
    slide = [[1.0, 0, 0, 0], [0, 1.0, 0, lateral], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
    parts = []
    for i, (code, mesh) in enumerate(zip(CODES, robot_link_meshes(model, tool_axis=False))):
        if i == 0:
            m = TO_CAD
        else:
            local = mat_mul(frames[i - 1], slide) if i >= 3 else frames[i - 1]
            m = mat_mul(TO_CAD, local)
        name = f"PBR6US18{code} pieza" if code else "brida"
        world = box((0, 0, 0), (1, 1, 1))
        world.triangles = [tuple(_apply(m, p) for p in tri) for tri in mesh.triangles]
        parts.append(Part(name, world, m))
    return parts


def recipe(model):
    zw, xw = model.d1 + model.a2 + model.a3, model.a1 + model.d4
    return {
        "links": dict(zip(["base", "J1", "J2", "J3", "J4", "J5", "J6"], CODES)),
        "axes": {
            "J1": ("A000", (0, 0, 1), (0, 0, 0)),
            "J2": ("B000", (0, 1, 0), (model.a1, 0, model.d1)),
            "J3": ("D000", (0, 1, 0), (model.a1, 0, model.d1 + model.a2)),
            "J4": ("E000", (1, 0, 0), (model.a1, 0, zw)),
            "J5": ("F000", (0, 1, 0), (xw, 0, zw)),
            "J6": ("F000", (1, 0, 0), (xw, 0, zw)),
        },
    }


def test_recovers_the_dimensions_and_the_cad_pose(model):
    robot = import_robot(synthetic_parts(model, CAD_POSE), recipe(model))
    for key in ("a1", "a2", "a3", "d4"):
        assert robot.geometry[key] == pytest.approx(getattr(model, key), abs=0.2)
    # d1: desde el apoyo de la base (la base sintética arranca en z = 0).
    assert robot.geometry["d1"] == pytest.approx(model.d1, abs=0.2)
    # d6: hasta la punta de la brida (el cilindro de la brida termina ahí).
    assert robot.geometry["d6"] == pytest.approx(model.d6, abs=0.2)
    assert robot.reach_mm == pytest.approx(model.reach_mm, abs=0.5)
    for got, want in zip(robot.cad_pose_deg, CAD_POSE):
        assert abs(((got - want) + 180) % 360 - 180) < 0.05
    assert robot.notes == []


def test_links_end_up_in_the_zero_pose(model):
    robot = import_robot(synthetic_parts(model, CAD_POSE), recipe(model))
    expected = robot_link_meshes(model, tool_axis=False)
    for got, want in zip(robot.links, expected):
        (glo, ghi), (wlo, whi) = got.bounds(), want.bounds()
        assert glo == pytest.approx(wlo, abs=0.3) and ghi == pytest.approx(whi, abs=0.3)


def test_a_forearm_slid_along_the_j3_axis_is_centered(model):
    robot = import_robot(synthetic_parts(model, CAD_POSE, lateral=8.5), recipe(model))
    assert robot.geometry["d4"] == pytest.approx(model.d4, abs=0.2)
    assert robot.notes and "corrido" in robot.notes[0]
    lo, hi = robot.links[4].bounds()
    assert (lo[1] + hi[1]) / 2 == pytest.approx(0, abs=0.3)


def test_missing_parts_are_reported(model):
    parts = synthetic_parts(model, CAD_POSE)[:-2]
    with pytest.raises(ValueError):
        import_robot(parts, recipe(model))


# --- ensamble STEP ----------------------------------------------------------------------


def test_names_in_gbk_are_decoded():
    assert _decode("三轴".encode("gbk").decode("latin-1")) == "三轴"
    assert _decode("plain") == "plain"


def test_a_single_part_step_splits_into_itself(tmp_path):
    gmsh = pytest.importorskip("gmsh")
    step = tmp_path / "caja.step"
    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.addBox(10, 20, 30, 100, 50, 20)
        gmsh.model.occ.synchronize()
        gmsh.write(str(step))
    finally:
        gmsh.finalize()
    [(name, _sdr, matrix)] = StepFile(step).parts()
    assert matrix[0][3] == 0 and matrix[0][0] == 1
    [part] = split_assembly(step)
    lo, hi = part.mesh.bounds()
    assert lo == pytest.approx((10, 20, 30), abs=0.01)
    assert hi == pytest.approx((110, 70, 50), abs=0.01)


def test_placement_reads_axis2_placement(tmp_path):
    step = tmp_path / "p.step"
    step.write_text(
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
        "#1 = CARTESIAN_POINT ( 'NONE',  ( 10.0, 20.0, 30.0 ) ) ;\n"
        "#2 = DIRECTION ( 'NONE',  ( 0.0, 0.0, 1.0 ) ) ;\n"
        "#3 = DIRECTION ( 'NONE',  ( 0.0, 1.0, 0.0 ) ) ;\n"
        "#4 = AXIS2_PLACEMENT_3D ( 'NONE', #1, #2, #3 ) ;\n"
        "ENDSEC;\nEND-ISO-10303-21;\n", encoding="latin-1")
    m = StepFile(step).placement(4)
    assert [m[i][3] for i in range(3)] == [10, 20, 30]
    assert [m[i][0] for i in range(3)] == pytest.approx([0, 1, 0])    # x = ref_direction
    assert [m[i][1] for i in range(3)] == pytest.approx([-1, 0, 0])   # y = z × x
    assert math.isclose(m[2][2], 1.0)


# --- el BRTIRUS1510A, importado del STEP del fabricante --------------------------------------


def test_the_1510a_model_comes_with_the_manufacturer_meshes():
    from sim.scene import has_real_meshes

    m = RobotModel.load("BRTIRUS1510A")
    assert has_real_meshes(m)
    # El alcance medido del CAD coincide con el nombre del modelo (1510 mm).
    assert m.reach_mm == pytest.approx(1510, abs=2)
    assert m.reach_mm == pytest.approx(m.a1 + m.a2 + math.hypot(m.d4, m.a3), abs=0.2)
    links = robot_link_meshes(m)
    assert len(links) == 7 and all(len(x) > 1000 for x in links)
    base_lo, _ = links[0].bounds()
    assert base_lo[2] == pytest.approx(0, abs=0.5)          # apoyada en z = 0
    lo, hi = links[4].bounds()                              # antebrazo centrado
    assert (lo[1] + hi[1]) / 2 == pytest.approx(0, abs=2)
    # La brida termina donde dice la cinemática.
    _, hi = links[6].bounds()
    assert hi[0] == pytest.approx(m.a1 + m.d4 + m.d6, abs=0.5)


def test_the_1510a_does_not_collide_with_itself_at_home():
    pytest.importorskip("fcl")
    from sim.collision import CollisionChecker
    from sim.scene import has_real_meshes

    m = RobotModel.load("BRTIRUS1510A")
    checker = CollisionChecker(m, robot_link_meshes(m, tool_axis=False), [], margin_mm=20,
                               approximate_robot=not has_real_meshes(m))
    home = [0.347, 45.894, -44.865, -0.792, -75.952, -0.859]
    assert all(d > 0 for _a, _b, d in checker.distances_at(home))
    assert all(d > 0 for _a, _b, d in checker.distances_at([0.0] * 6))
    # Doblado sobre sí mismo sí choca (con la forma real).
    assert any(d <= 0 for _a, b, d in checker.distances_at([0, -95, -80, 0, 0, 0]) if b == "base")


def test_import_cli_rejects_a_step_that_is_not_a_robot(tmp_path, capsys):
    gmsh = pytest.importorskip("gmsh")
    from sim.robot_import import main

    step = tmp_path / "caja.step"
    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.addBox(0, 0, 0, 10, 10, 10)
        gmsh.model.occ.synchronize()
        gmsh.write(str(step))
    finally:
        gmsh.finalize()
    assert main([str(step), "--name", "X", "--joints-from", "BRTIRUS1510A",
                 "--out", str(tmp_path)]) == 2
    assert "Error" in capsys.readouterr().err
    assert not (tmp_path / "X.json").exists()
