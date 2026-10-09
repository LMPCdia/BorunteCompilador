"""
Perfil de velocidad del simulador y curvas de movimiento (sim/motion.py,
gui/motion_charts.py).
"""

import math

import pytest

from compiler.pad_codegen import PadOptions, compile_to_pad
from sim.kinematics import RobotModel
from sim.motion import analyze
from sim.pad_sim import simulate, trapezoid
from sim.scene import Timeline

HOME = "JOINT(0.347, 45.894, -44.865, -0.792, -75.952, -0.859)"


@pytest.fixture(scope="module")
def model():
    return RobotModel.load("BRTIRUS1510A")


def _run(model, src, accel_s=0.0):
    result = simulate(compile_to_pad(src, PadOptions(allow_unverified=True)), model, accel_s=accel_s)
    return result, Timeline(result)


# --- perfil trapezoidal --------------------------------------------------------------


def test_trapezoid_adds_one_acceleration_time():
    times = [i * 0.1 for i in range(11)]           # 1 s a velocidad constante
    out = trapezoid(times, 0.2)
    assert out[0] == 0 and out[-1] == pytest.approx(1.2)
    assert out == sorted(out)
    assert out[5] == pytest.approx(0.6)            # en crucero: corrido accel/2
    assert out[1] == pytest.approx(math.sqrt(2 * 0.1 * 0.2))


def test_short_moves_are_triangular():
    out = trapezoid([0.0, 0.05, 0.1], 0.4)
    assert out[-1] == pytest.approx(2 * math.sqrt(0.1 * 0.4))
    assert out[1] == pytest.approx(out[-1] / 2)


def test_no_acceleration_keeps_the_times():
    assert trapezoid([0.0, 0.3, 0.9], 0.0) == [0.0, 0.3, 0.9]


def test_acceleration_makes_the_cycle_longer(model):
    src = f"MOVEJ {HOME} SPEED 50\nMOVEJ JOINT(60, 30, -20, 0, -60, 0) SPEED 50\n"
    flat, _ = _run(model, src)
    ramped, _ = _run(model, src, accel_s=0.3)
    assert ramped.total_time_s == pytest.approx(flat.total_time_s + 0.3, abs=1e-6)


# --- curvas ------------------------------------------------------------------------------


def test_joint_speed_reaches_the_programmed_percentage(model):
    # J1 sola, 60° al 50 %: la velocidad de crucero es la mitad de la máxima.
    src = (f"MOVEJ JOINT(0, 30, -20, 0, -60, 0) SPEED 50\n"
           f"MOVEJ JOINT(60, 30, -20, 0, -60, 0) SPEED 50\n")
    _r, timeline = _run(model, src, accel_s=0.2)
    curves = analyze(timeline, model, accel_s=0.2)
    cruise = model.joints[0].max_speed_dps * 0.5
    assert max(curves.speed[0]) == pytest.approx(cruise, rel=0.03)
    assert max(abs(v) for v in curves.speed[1]) < 1e-6        # J2 no se mueve
    # Aceleración ≈ velocidad de crucero / tiempo de aceleración.
    assert max(curves.accel[0]) == pytest.approx(cruise / 0.2, rel=0.2)
    assert curves.accelerations_valid
    peak = curves.peaks(model)[0]
    assert peak["v_pct"] == pytest.approx(50, abs=2)
    assert peak["min_deg"] == pytest.approx(0) and peak["max_deg"] == pytest.approx(60)


def test_tcp_speed_in_a_straight_line(model):
    from sim.kinematics import matrix_to_pose

    start = [0.0, 30.0, -20.0, 0.0, -60.0, 0.0]
    x, y, z, u, v, w = matrix_to_pose(model.fk(start))
    src = (f"MOVEJ JOINT({', '.join(map(str, start))}) SPEED 50\n"
           f"MOVEL WORLD({x:.3f}, {y + 300:.3f}, {z:.3f}, {u:.3f}, {v:.3f}, {w:.3f}) SPEED 20\n")
    result, timeline = _run(model, src, accel_s=0.2)
    curves = analyze(timeline, model, accel_s=0.2)
    # 300 mm recorridos = integral de la velocidad de la punta.
    travelled = sum(v * (t1 - t0) for v, t0, t1 in zip(curves.tcp_speed, curves.t, curves.t[1:]))
    assert travelled == pytest.approx(300, rel=0.03)
    # Arranca quieta (lo que queda es la ventana de la derivada).
    assert curves.tcp_speed[0] < 0.05 * max(curves.tcp_speed)


def test_a_simulator_jump_is_not_a_speed_spike(model):
    # Un MOVEL que no llega deja un "salto" de recuperación: sin velocidad infinita.
    src = (f"MOVEJ {HOME} SPEED 50\nMOVEL WORLD(1700, 0, 300, 180, 0, 0) SPEED 20\n"
           f"MOVEJ JOINT(10, 30, -20, 0, -60, 0) SPEED 50\n")
    _r, timeline = _run(model, src, accel_s=0.2)
    if not timeline.jumps():
        pytest.skip("este modelo llegó: no hubo salto")
    curves = analyze(timeline, model, accel_s=0.2)
    limits = [j.max_speed_dps for j in model.joints]
    for j in range(6):
        assert max(abs(v) for v in curves.speed[j]) <= limits[j] * 1.05


def test_without_profile_accelerations_are_flagged(model):
    _r, timeline = _run(model, f"MOVEJ {HOME} SPEED 50\nMOVEJ JOINT(30, 30, -20, 0, -60, 0) SPEED 50\n")
    assert not analyze(timeline, model).accelerations_valid


# --- ventana -------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def app():
    from PySide6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def test_charts_window_shows_curves_and_follows_the_time(app, model):
    from gui.motion_charts import MotionCharts

    src = f"MOVEJ {HOME} SPEED 50\nMOVEJ JOINT(60, 30, -20, 0, -60, 0) SPEED 50\n"
    _r, timeline = _run(model, src, accel_s=0.25)
    clicked = []
    charts = MotionCharts(on_time=clicked.append)
    charts.set_data(analyze(timeline, model, accel_s=0.25), model, 0.25)
    assert charts.tabs.count() == 5
    chart = charts.views["speed"].chart()
    names = [s.name() for s in chart.series() if s.name()]
    assert names == [f"J{i}" for i in range(1, 7)]
    assert charts.table.rowCount() == 7 and "°/s" in charts.table.item(0, 2).text()
    assert "supuesto" in charts.note.text()
    charts.set_time(1.0)
    cursor = charts._cursors["speed"]
    assert cursor.at(0).x() == pytest.approx(1.0)
    charts._clicked(99.0)
    assert clicked == [pytest.approx(timeline.duration)]


def test_sim_view_opens_the_charts_and_marks_collisions(app, tmp_path):
    pytest.importorskip("fcl")
    from gui.sim_view import SimView
    from sim.meshes import box, write_stl

    src = (f"POINT p = WORLD(1097.1, -150.0, 721.9, 180.0, -10.0, 180.0)\n"
           f"MOVEJ {HOME} SPEED 80\nMOVEL p + OFFSET(0, 0, 100, 0, 0, 0) SPEED 50\n"
           f"MOVEL p SPEED 10\nMOVEJ {HOME} SPEED 80\n")
    view = SimView(lambda: src, enable_3d=False)
    view.model_combo.setCurrentText("BRTIRUS1820A")
    piece = tmp_path / "mesa.stl"
    write_stl(box((0, 0, 365), (600, 400, 730)), piece)
    obj = view.import_object(piece)
    obj.x, obj.y, obj.z = 1100.0, -150.0, 0.0
    view._refresh_objects(rebuild=False)
    view.accel_spin.setValue(0.3)
    view.simulate()
    assert "arranque/frenado supuesto de 0.3 s" in view.summary.text()
    view.show_charts()
    assert view.charts is not None and view.charts.isVisible()
    assert view.collisions.collisions >= 1
    bands = [s for s in view.charts.views["angle"].chart().series() if not s.name()]
    assert len(bands) >= 2                       # al menos una franja + el cursor
    view.charts._clicked(1.0)
    assert view._t == pytest.approx(1.0)
    view.cancel()
    assert not view.charts.isVisible()


def test_changing_the_acceleration_marks_the_result_stale(app):
    from gui.sim_view import SimView

    view = SimView(lambda: f"MOVEJ {HOME} SPEED 50\n", enable_3d=False)
    view.simulate()
    assert not view.stale
    view.accel_spin.setValue(view.accel_spin.value() + 0.1)
    assert view.stale
