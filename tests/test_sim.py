"""
Simulador cinemático (`sim/`).

Lo que está fijado contra el plano del fabricante del BRTIRUS1820A: el
alcance máximo (1731.5 mm) y la altura máxima (2056 mm) del "P point" (centro
de la muñeca). El resto (sentidos de giro, convención U/V/W) son hipótesis
documentadas en docs/SIMULATOR.md: estos tests fijan que el código sea
coherente consigo mismo, no que coincida con el robot.
"""

import math
import random

import pytest

from compiler.pad_codegen import compile_to_pad
from sim.check import main as check_main
from sim.kinematics import RobotModel, matrix_to_pose, pose_error, pose_matrix
from sim.pad_sim import simulate

HOME = [0.347, 45.894, -44.865, -0.792, -75.952, -0.859]


@pytest.fixture(scope="module")
def model():
    return RobotModel.load("BRTIRUS1820A")


# --- modelo y cinemática directa ------------------------------------------------


def test_models_are_listed():
    assert "BRTIRUS1820A" in RobotModel.available()


def test_zero_pose_matches_the_drawing(model):
    # Brazo vertical, antebrazo horizontal: muñeca a 170 + 825.5 adelante y a
    # 494.6 + 730 + 100 de altura.
    x, y, z = model.wrist_center([0] * 6)
    assert (round(x, 1), round(y, 1), round(z, 1)) == (995.5, 0.0, 1324.6)


def test_wrist_reach_matches_the_drawing(model):
    # Brazo estirado hacia adelante (J2 = 90 - atan(100/825.5) tiene que
    # entrar en rango solo como chequeo geométrico, sin límites).
    stretch = math.degrees(math.atan2(100, 825.5))
    up = model.wrist_center([0, 0, -(90 - stretch), 0, 0, 0])
    assert up[2] == pytest.approx(2056, abs=0.5)        # cota "2056"
    reach = 170 + 730 + math.hypot(825.5, 100)
    assert reach == pytest.approx(model.reach_mm, abs=0.1)  # cota "1731.5"


def test_home_points_the_tool_down(model):
    # Del HOME de un programa real: herramienta hacia abajo. Es la evidencia
    # del sentido de giro de J5 (ver docs/SIMULATOR.md).
    flange = model.fk(HOME)
    assert flange[2][2] < -0.9


def test_limits(model):
    assert model.out_of_limits([0, 0, 0, 0, 0, 0]) == []
    assert model.out_of_limits([170, 0, 0, 0, 0, 0]) == [0]
    assert model.out_of_limits([0, 71, -90, 0, 0, 0]) == [1, 2]


def test_pose_matrix_roundtrip():
    pose = (100.0, -200.0, 300.0, 162.6, -14.0, 148.1)
    assert matrix_to_pose(pose_matrix(*pose)) == pytest.approx(pose, abs=1e-9)


# --- cinemática inversa --------------------------------------------------------------


def test_ik_recovers_random_poses(model):
    rng = random.Random(1)
    for _ in range(60):
        q = [rng.uniform(j.min_deg * 0.8, j.max_deg * 0.8) for j in model.joints]
        seed = [qi + rng.uniform(-5, 5) for qi in q]
        sol = model.ik(model.fk(q), seed)
        assert sol is not None, q
        err = pose_error(model.fk(sol), model.fk(q))
        assert max(map(abs, err[:3])) < 0.02 and max(map(abs, err[3:])) < 1e-4


def test_ik_converges_at_the_wrist_singularity(model):
    q = [10, 20, -10, 30, 0.0, -30]  # J5 = 0: J4 y J6 alineados
    sol = model.ik(model.fk(q), [qi + 3 for qi in q])
    assert sol is not None
    assert max(map(abs, pose_error(model.fk(sol), model.fk(q))[:3])) < 0.02


def test_ik_fails_out_of_reach(model):
    assert model.ik(pose_matrix(5000, 0, 1000, 0, 90, 0), [0] * 6) is None


# --- simulación de un respaldo del pad -----------------------------------------------


def _world_of(model, q, dx=0.0, dy=0.0, dz=0.0):
    x, y, z, u, v, w = matrix_to_pose(model.fk(q))
    return f"WORLD({x + dx:.3f}, {y + dy:.3f}, {z + dz:.3f}, {u:.3f}, {v:.3f}, {w:.3f})"


def test_simple_program_runs_without_issues(model):
    src = f"""POINT casa = JOINT({', '.join(map(str, HOME))})
MOVEJ casa SPEED 50
MOVEL {_world_of(model, HOME, dy=200, dz=-150)} SPEED 20
SET_OUT(Y010, ON)
WAIT 0.5s
MOVEJ casa SPEED 50
"""
    result = simulate(compile_to_pad(src), model)
    assert result.ok, result.issues
    assert [s.kind for s in result.segments] == ["MOVEJ", "MOVEL", "WAIT", "MOVEJ"]
    assert result.outputs and result.outputs[0][1:] == (0, True)
    assert result.total_time_s > 0.5


def test_movel_follows_a_straight_line(model):
    src = f"""MOVEJ JOINT({', '.join(map(str, HOME))}) SPEED 50
MOVEL {_world_of(model, HOME, dz=-200)} SPEED 20
"""
    result = simulate(compile_to_pad(src), model)
    movel = result.segments[1]
    start = model.fk(movel.samples[0])
    for q in movel.samples:
        p = model.fk(q)
        assert p[0][3] == pytest.approx(start[0][3], abs=0.05)
        assert p[1][3] == pytest.approx(start[1][3], abs=0.05)


def test_joint_limit_is_an_error(model):
    result = simulate(compile_to_pad("MOVEJ JOINT(0, 80, 0, 0, 0, 0) SPEED 10\n"), model)
    assert not result.ok
    assert "J2=80.0°" in result.issues[0].message


def test_unreachable_movel_is_an_error(model):
    src = f"MOVEJ JOINT({', '.join(map(str, HOME))}) SPEED 50\nMOVEL WORLD(4000, 0, 1000, 0, 90, 0) SPEED 20\n"
    result = simulate(compile_to_pad(src), model)
    assert not result.ok
    assert result.issues[0].where == "MAIN[4]"


def test_movel_with_unknown_tool_or_frame_is_skipped_with_one_warning(model):
    src = f"TOOL 2\nCOORD 1\nMOVEL {_world_of(model, HOME)} SPEED 20\nMOVEL {_world_of(model, HOME)} SPEED 20\n"
    result = simulate(compile_to_pad(src), model)
    assert result.ok
    [issue] = result.issues
    assert issue.message.startswith("2 MOVEL sin simular")
    assert "herramienta 2" in issue.message and "coordenadas 1" in issue.message


def test_movel_puts_the_tool_tip_on_the_point_measured_in_the_frame(model):
    from sim.kinematics import mat_mul
    from sim.pad_sim import PadSimulator, invert

    tool = [0, 0, 250, 0, 0, 0]           # torcha de 250 mm en el eje de la brida
    frame = [800, -300, 200, 0, 0, 30]    # mesa girada 30° y desplazada
    sim = PadSimulator(model, tools={2: tool}, frames={1: frame})
    # Punto en el sistema de la mesa, con la herramienta apuntando hacia abajo.
    src = (f"MOVEJ JOINT({', '.join(map(str, HOME))}) SPEED 50\nTOOL 2\nCOORD 1\n"
           "MOVEL WORLD(400, 300, 300, 180, 0, 0) SPEED 20\n")
    result = sim.run(compile_to_pad(src))
    assert result.ok, result.issues
    q = result.segments[-1].samples[-1]
    tip_in_world = mat_mul(model.fk(q), pose_matrix(*tool))
    tip_in_frame = mat_mul(invert(pose_matrix(*frame)), tip_in_world)
    err = pose_error(tip_in_frame, pose_matrix(400, 300, 300, 180, 0, 0))
    assert max(map(abs, err[:3])) < 0.05 and max(map(abs, err[3:])) < 1e-3
    assert sim.tcp(q, 2) == pytest.approx(tuple(tip_in_world[i][3] for i in range(3)))


def test_unknown_actions_are_reported_once_per_code(model):
    backup = compile_to_pad("WAIT 1s\n")
    backup.act.main[1:1] = [{"action": 53000, "insertedIndex": 90},
                            {"action": 53000, "insertedIndex": 91}]
    [issue] = simulate(backup, model).issues
    assert "53000" in issue.message and "2 vez" in issue.message


def test_inputs_used_lists_the_inputs_the_program_reads():
    from sim.pad_sim import inputs_used

    src = "PROC p()\nIF X013 == 1 THEN\nWAIT 1s\nENDIF\nENDPROC\nIF X012 == 1 THEN\np()\nENDIF\n"
    assert inputs_used(compile_to_pad(src)) == [2, 3]


def test_user_models_directory(tmp_path, monkeypatch):
    import json

    import sim.kinematics as kin

    data = json.loads((kin.MODELS_DIR / "BRTIRUS1820A.json").read_text(encoding="utf-8"))
    data["name"] = "MiRobot"
    (tmp_path / "MiRobot.json").write_text(json.dumps(data), encoding="utf-8")
    monkeypatch.setattr(kin, "USER_MODELS_DIR", tmp_path)
    assert "MiRobot" in RobotModel.available()
    assert RobotModel.load("MiRobot").a2 == 730


def test_inputs_drive_the_if(model):
    src = "PROC p()\nWAIT 1s\nENDPROC\nIF X012 == 1 THEN\np()\nENDIF\n"
    backup = compile_to_pad(src)
    assert simulate(backup, model, inputs={2: True}).total_time_s == pytest.approx(1)
    assert simulate(backup, model, inputs={2: False}).total_time_s == 0


def test_check_cli(tmp_path, capsys):
    src = tmp_path / "p.krlb"
    src.write_text("MOVEJ JOINT(0, 80, 0, 0, 0, 0) SPEED 10\n", encoding="utf-8")
    assert check_main([str(src)]) == 1
    out = capsys.readouterr().out
    assert "BRTIRUS1820A" in out and "fuera de rango" in out
