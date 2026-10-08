"""
Simulador cinemático (`sim/`).

Fijado contra el plano del fabricante del BRTIRUS1820A: el alcance máximo
(1731.5 mm) y la altura máxima (2056 mm) del "P point" (centro de la
muñeca). Fijado contra un respaldo real: el HOME queda recogido y con la
herramienta hacia abajo. El resto (convención U/V/W, J4/J6) son hipótesis
documentadas en docs/SIMULATOR.md: los tests fijan coherencia, no verdad.
"""

import math
import random

import pytest

from compiler.pad_codegen import PadOptions, compile_to_pad
from sim.check import main as check_main
from sim.kinematics import (
    RobotModel,
    identity,
    mat_mul,
    matrix_to_pose,
    pose_error,
    pose_matrix,
    rot_axis,
)
from sim.pad_sim import Cancelled, PadSimulator, invert, simulate

HOME = [0.347, 45.894, -44.865, -0.792, -75.952, -0.859]
JHOME = f"JOINT({', '.join(map(str, HOME))})"
UNVERIFIED = PadOptions(allow_unverified=True)


@pytest.fixture(scope="module")
def model():
    return RobotModel.load("BRTIRUS1820A")


def _pad(src, **kw):
    return compile_to_pad(src, PadOptions(allow_unverified=True, **kw))


def _world_of(model, q, dx=0.0, dy=0.0, dz=0.0):
    x, y, z, u, v, w = matrix_to_pose(model.fk(q))
    return f"WORLD({x + dx:.3f}, {y + dy:.3f}, {z + dz:.3f}, {u:.3f}, {v:.3f}, {w:.3f})"


# --- modelo y cinemática directa ------------------------------------------------


def test_models_are_listed():
    assert "BRTIRUS1820A" in RobotModel.available()


def test_zero_pose_matches_the_drawing(model):
    # Brazo vertical, antebrazo horizontal: muñeca a 170 + 825.5 adelante y a
    # 494.6 + 730 + 100 de altura.
    x, y, z = model.wrist_center([0] * 6)
    assert (round(x, 1), round(y, 1), round(z, 1)) == (995.5, 0.0, 1324.6)


def test_wrist_reach_matches_the_drawing(model):
    # Antebrazo girado hasta quedar alineado con el brazo (sin mirar rangos).
    stretch = 90 - math.degrees(math.atan2(100, 825.5))
    up = max(model.wrist_center([0, 0, s, 0, 0, 0])[2] for s in (stretch, -stretch))
    assert up == pytest.approx(2056, abs=0.5)                 # cota "2056"
    reach = 170 + 730 + math.hypot(825.5, 100)
    assert reach == pytest.approx(model.reach_mm, abs=0.1)    # cota "1731.5"


def test_home_is_tucked_with_the_tool_down(model):
    # HOME de un programa real: es la evidencia de los sentidos de J2, J3 y J5
    # (docs/SIMULATOR.md). Con J2/J3 al revés quedaba casi estirado.
    flange = model.fk(HOME)
    assert flange[2][2] < -0.9          # herramienta hacia abajo
    assert flange[0][3] < 800           # brida recogida, no estirada


def test_limits(model):
    assert model.out_of_limits([0, 0, 0, 0, 0, 0]) == []
    assert model.out_of_limits([170, 0, 0, 0, 0, 0]) == [0]
    assert model.out_of_limits([0, 71, -90, 0, 0, 0]) == [1, 2]


def test_wrap_into_limits_picks_the_turn_inside_the_range(model):
    q = model.wrap_into_limits([0, 0, 0, 190, 0, 0], [0, 0, 0, 170, 0, 0])
    assert q[3] == pytest.approx(-170)


def test_pose_matrix_roundtrip():
    pose = (100.0, -200.0, 300.0, 162.6, -14.0, 148.1)
    assert matrix_to_pose(pose_matrix(*pose)) == pytest.approx(pose, abs=1e-9)


def test_rotation_error_is_exact_at_180_degrees():
    rng = random.Random(3)
    for _ in range(500):
        axis = [rng.gauss(0, 1) for _ in range(3)]
        n = math.sqrt(sum(a * a for a in axis))
        axis = tuple(a / n for a in axis)
        angle = rng.choice([math.pi, math.pi - 1e-7, math.pi - 1e-4, rng.uniform(0, math.pi)])
        r = rot_axis(axis, angle)
        m = [r[0] + [0], r[1] + [0], r[2] + [0], [0, 0, 0, 1]]
        v = pose_error(identity(), m)[3:]
        size = math.sqrt(sum(x * x for x in v))
        back = rot_axis(tuple(x / size for x in v), size)
        assert max(abs(back[i][j] - r[i][j]) for i in range(3) for j in range(3)) < 1e-9


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


def test_first_movej_places_the_robot_without_time(model):
    result = simulate(_pad(f"MOVEJ {JHOME} SPEED 50\nWAIT 1s\n"), model)
    assert result.start_deg == HOME
    assert result.total_time_s == pytest.approx(1)  # solo el WAIT


def test_movel_before_any_movej_is_not_evaluated(model):
    result = simulate(_pad(f"MOVEL {_world_of(model, HOME)} SPEED 20\nMOVEJ {JHOME} SPEED 50\n"),
                      model)
    assert result.ok and result.skipped_moves == 1
    assert "no se sabía dónde estaba el robot" in result.issues[0].message


def test_simple_program_runs_without_issues(model):
    src = f"""POINT casa = {JHOME}
MOVEJ casa SPEED 50
MOVEL {_world_of(model, HOME, dy=200, dz=-150)} SPEED 20
SET_OUT(Y010, ON)
WAIT 0.5s
MOVEJ casa SPEED 50
"""
    result = simulate(_pad(src), model)
    assert result.ok, result.issues
    assert [s.kind for s in result.segments] == ["MOVEJ", "MOVEL", "WAIT", "MOVEJ"]
    assert result.outputs and result.outputs[0][1:] == (0, True)
    assert result.total_time_s > 0.5
    assert result.total_moves == 3 and result.skipped_moves == 0


def test_movel_keeps_the_tool_tip_on_a_straight_line(model):
    tool = [0, 0, 400, 0, 0, 0]
    tip = matrix_to_pose(mat_mul(model.fk(HOME), pose_matrix(*tool)))
    target = list(tip)
    target[1] += 250
    target[3] += 15  # además gira: la brida describe un arco, la punta no
    src = (f"MOVEJ {JHOME} SPEED 50\nTOOL 1\nMOVEL WORLD({', '.join(f'{v:.4f}' for v in target)}) "
           f"SPEED 20\n")
    result = PadSimulator(model, tools={1: tool}).run(_pad(src))
    assert result.ok, result.issues
    movel = result.segments[-1]
    p0 = mat_mul(model.fk(movel.samples[0]), pose_matrix(*tool))
    p1 = mat_mul(model.fk(movel.samples[-1]), pose_matrix(*tool))
    a = [p0[i][3] for i in range(3)]
    b = [p1[i][3] for i in range(3)]
    for q in movel.samples:
        m = mat_mul(model.fk(q), pose_matrix(*tool))
        p = [m[i][3] for i in range(3)]
        # distancia del punto a la recta a-b
        ab = [b[i] - a[i] for i in range(3)]
        ap = [p[i] - a[i] for i in range(3)]
        t = sum(x * y for x, y in zip(ap, ab)) / sum(x * x for x in ab)
        off = math.dist(p, [a[i] + ab[i] * t for i in range(3)])
        assert off < 0.1


def test_movel_puts_the_tool_tip_on_the_point_measured_in_the_frame(model):
    tool = [0, 0, 250, 0, 0, 0]
    frame = [800, -300, 200, 0, 0, 30]
    sim = PadSimulator(model, tools={2: tool}, frames={1: frame})
    src = f"MOVEJ {JHOME} SPEED 50\nTOOL 2\nCOORD 1\nMOVEL WORLD(400, 300, 300, 180, 0, 0) SPEED 20\n"
    result = sim.run(_pad(src))
    assert result.ok, result.issues
    q = result.segments[-1].samples[-1]
    tip_in_world = mat_mul(model.fk(q), pose_matrix(*tool))
    tip_in_frame = mat_mul(invert(pose_matrix(*frame)), tip_in_world)
    err = pose_error(tip_in_frame, pose_matrix(400, 300, 300, 180, 0, 0))
    assert max(map(abs, err[:3])) < 0.05 and max(map(abs, err[3:])) < 1e-3
    assert sim.tcp(q, 2) == pytest.approx(tuple(tip_in_world[i][3] for i in range(3)))


def test_joint_limit_is_an_error(model):
    result = simulate(_pad("MOVEJ JOINT(0, 80, 0, 0, 0, 0) SPEED 10\n"), model)
    assert not result.ok
    assert "J2=80.0°" in result.issues[0].message


def test_unreachable_movel_is_an_error_and_the_partial_path_is_kept(model):
    src = f"MOVEJ {JHOME} SPEED 50\nMOVEL WORLD(4000, 0, 1000, 180, 0, 0) SPEED 20\n"
    result = simulate(_pad(src), model)
    assert not result.ok
    assert result.issues[0].where == "MAIN[2]"
    assert result.segments[-1].failed and result.skipped_moves == 1


def test_one_failure_does_not_cascade(model):
    reachable = _world_of(model, HOME, dz=-100)
    src = (f"MOVEJ {JHOME} SPEED 50\nMOVEL WORLD(4000, 0, 1000, 180, 0, 0) SPEED 20\n"
           + f"MOVEL {reachable} SPEED 20\n" * 3 + f"MOVEJ {JHOME} SPEED 50\n"
           + f"MOVEL {reachable} SPEED 20\n")
    result = simulate(_pad(src), model)
    errors = [i for i in result.issues if i.severity == "error"]
    assert len(errors) == 1                       # solo el MOVEL imposible
    assert result.segments[-1].kind == "MOVEL" and not result.segments[-1].failed


def test_unknown_tool_skips_with_one_warning_and_resumes_at_the_next_movej(model):
    src = (f"MOVEJ {JHOME} SPEED 50\nTOOL 2\nCOORD 1\n"
           f"MOVEL {_world_of(model, HOME)} SPEED 20\nMOVEL {_world_of(model, HOME)} SPEED 20\n"
           f"TOOL 0\nCOORD 0\nMOVEL {_world_of(model, HOME, dz=-50)} SPEED 20\n"
           f"MOVEJ {JHOME} SPEED 50\nMOVEL {_world_of(model, HOME, dz=-50)} SPEED 20\n")
    result = simulate(_pad(src), model)
    assert result.ok
    assert result.missing_tools == [2] and result.missing_frames == [1]
    assert result.skipped_moves == 3  # 2 sin marcos + 1 sin pose conocida
    assert result.segments[-1].kind == "MOVEL"


def test_unknown_actions_are_reported_once_per_code(model):
    backup = _pad("WAIT 1s\n")
    backup.act.main[1:1] = [{"action": 53000, "insertedIndex": 90},
                            {"action": 53000, "insertedIndex": 91}]
    [issue] = simulate(backup, model).issues
    assert "53000" in issue.message and "2 vez" in issue.message


def test_an_infinite_loop_is_simulated_once(model):
    backup = _pad(f"MOVEJ {JHOME} SPEED 50\nMOVEJ JOINT(10, 45, -45, 0, -75, 0) SPEED 50\n")
    main = backup.act.main
    # etiqueta al principio y "si X010 OFF volver" al final: bucle con X010 apagada
    main.insert(1, {"action": 59999, "comment": "Inicio", "flag": 0, "insertedIndex": 50})
    main.insert(len(main) - 1, {"action": 10001, "flag": 0, "inout": 0, "insertedIndex": 51,
                                "limit": "0.000", "point": 0, "pointStatus": 0, "type": 0})
    result = simulate(backup, model)
    assert not result.complete
    assert any("bucle" in i.message for i in result.issues)
    assert result.total_time_s < 10


def test_progress_can_cancel(model):
    src = f"MOVEJ {JHOME} SPEED 50\n" + "WAIT 1s\n" * 200

    def cancel(_n):
        raise Cancelled

    with pytest.raises(Cancelled):
        PadSimulator(model, progress=cancel).run(_pad(src))


def test_segment_times_follow_the_joints(model):
    src = f"MOVEJ {JHOME} SPEED 50\nMOVEJ JOINT(40, 45.894, -44.865, -0.792, -75.952, -0.859) SPEED 50\n"
    seg = simulate(_pad(src), model).segments[-1]
    assert seg.times[0] == 0 and seg.times[-1] == pytest.approx(seg.duration_s)
    assert seg.duration_s == pytest.approx((40 - 0.347) / (190 * 0.5), rel=1e-6)


def test_inputs_used_lists_the_inputs_the_program_reads():
    from sim.pad_sim import inputs_used

    src = "PROC p()\nIF X013 == 0 THEN\nWAIT 1s\nENDIF\nENDPROC\nIF X012 == 0 THEN\np()\nENDIF\n"
    assert inputs_used(compile_to_pad(src)) == [2, 3]


def test_inputs_drive_the_if(model):
    src = "PROC p()\nWAIT 1s\nENDPROC\nIF X012 == 1 THEN\np()\nENDIF\n"
    backup = _pad(src)
    assert simulate(backup, model, inputs={2: True}).total_time_s == pytest.approx(1)
    assert simulate(backup, model, inputs={2: False}).total_time_s == 0


def test_user_models_directory(tmp_path, monkeypatch):
    import json

    import sim.kinematics as kin

    data = json.loads((kin.MODELS_DIR / "BRTIRUS1820A.json").read_text(encoding="utf-8"))
    data["name"] = "otro nombre adentro"  # manda el nombre del archivo
    (tmp_path / "MiRobot.json").write_text(json.dumps(data), encoding="utf-8")
    (tmp_path / "Roto.json").write_text("{esto no es json", encoding="utf-8")
    (tmp_path / "Incompleto.json").write_text('{"joints": []}', encoding="utf-8")
    monkeypatch.setattr(kin, "USER_MODELS_DIR", tmp_path)
    assert "MiRobot" in RobotModel.available()
    assert "Roto" not in RobotModel.available() and "Incompleto" not in RobotModel.available()
    assert RobotModel.load("MiRobot").name == "MiRobot"
    assert len(RobotModel.problems()) == 2
    with pytest.raises(ValueError, match="Roto.json"):
        RobotModel.load("Roto")


def test_check_cli(tmp_path, capsys):
    src = tmp_path / "p.krlb"
    src.write_text("MOVEJ JOINT(0, 80, 0, 0, 0, 0) SPEED 10\n", encoding="utf-8")
    assert check_main([str(src)]) == 1
    out = capsys.readouterr().out
    assert "BRTIRUS1820A" in out and "fuera de rango" in out


# --- segunda ronda de revisión ----------------------------------------------------


def test_singularity_is_named_when_the_target_is_reachable(model):
    start = [0, 30, 0, 90, -20, -90]
    target = list(start)
    target[4] = 20
    src = (f"MOVEJ JOINT({', '.join(map(str, start))}) SPEED 50\n"
           f"MOVEL {_world_of(model, target)} SPEED 20\n")
    result = simulate(_pad(src), model)
    errors = [i.message for i in result.issues if i.severity == "error"]
    if errors:  # si el IK logra cruzarla, no hay error; si no, que diga por qué
        assert "singularidad de muñeca" in errors[0] and "sí se alcanza" in errors[0]


def test_unevaluated_moves_say_why(model):
    src = (f"MOVEJ {JHOME} SPEED 50\nTOOL 2\nMOVEL {_world_of(model, HOME)} SPEED 20\n"
           f"TOOL 0\nMOVEL {_world_of(model, HOME, dz=-50)} SPEED 20\n")
    result = simulate(_pad(src), model)
    [unevaluated] = [i for i in result.issues if "sin evaluar" in i.message]
    assert "herramienta/coordenadas sin cargar" in unevaluated.message


def test_recovery_is_a_zero_time_jump_not_a_movement(model):
    reachable = _world_of(model, HOME, dz=-100)
    src = (f"MOVEJ {JHOME} SPEED 50\nMOVEL WORLD(1700, 0, 300, 180, 0, 0) SPEED 20\n"
           f"MOVEL {reachable} SPEED 20\n")
    result = simulate(_pad(src), model)
    jumps = [s for s in result.segments if s.kind == "SALTO"]
    for jump in jumps:
        assert jump.duration_s == 0 and len(jump.samples) == 2


def test_check_cli_accepts_unverified_forms_and_lowercase_inputs(tmp_path, capsys):
    src = tmp_path / "p.krlb"
    src.write_text("PROC p()\nWAIT 1s\nENDPROC\nIF X012 == 1 THEN\np()\nENDIF\n", encoding="utf-8")
    assert check_main([str(src), "--input", "x012=1"]) == 0
    assert "1.0 s" in capsys.readouterr().out


def test_check_cli_reports_compile_errors_without_traceback(tmp_path, capsys):
    src = tmp_path / "p.krlb"
    src.write_text("MOVEJ nada SPEED 10\n", encoding="utf-8")
    assert check_main([str(src)]) == 2
    assert "Línea 1" in capsys.readouterr().err
