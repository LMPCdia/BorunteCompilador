"""Planilla de parámetros del robot (sim/robot_params.py)."""

import json

import pytest

from sim import robot_params
from sim.kinematics import MODELS_DIR, RobotModel
from sim.robot_params import ParamsError, apply_params, parse_params, template_csv, verify_ik

SHEET = """Parámetros del robot,BRTIRUS1510A,,,,,,
,,,,,,,
Eje,Mínimo (°),Máximo (°),Velocidad máx (°/s),Aceleración máx (°/s²),Sentido,Confirmado,Notas
J1,±165,,190,450,1,sí,
J2,-95,70,173,500,-1,parcial,velocidad confirmada
J3,-85,75,223,600,-1,no,
J4,"-180","180",254,"900,5",1,sí,
J5,-115,115,270,900,-1,sí,
J6,±360,,342,1200,1,sí,
,,,,,,,
Dato,Valor,Unidad,Confirmado,Notas
Alcance,1500,mm,sí,
Carga,10,kg,,
Velocidad lineal máx,1800,mm/s,no,
a2,686,mm,,plano
d4,700,mm,,plano (mal a propósito)
"""


def base_data():
    return json.loads((MODELS_DIR / "BRTIRUS1510A.json").read_text(encoding="utf-8"))


def test_parses_ranges_speeds_accelerations_and_general_data():
    p = parse_params(SHEET)
    assert p.model == "BRTIRUS1510A"
    assert (p.joints[0].min_deg, p.joints[0].max_deg) == (-165, 165)        # ±165
    assert (p.joints[5].min_deg, p.joints[5].max_deg) == (-360, 360)
    assert p.joints[3].max_accel_dps2 == 900.5                               # coma decimal
    assert [j.sign for j in p.joints] == [1, -1, -1, 1, -1, 1]
    assert [j.confirmed for j in p.joints[:3]] == ["sí", "parcial", "no"]
    assert p.general == {"reach_mm": 1500, "payload_kg": 10, "max_linear_speed_mms": 1800}
    assert p.dims == {"a2": 686, "d4": 700}


def test_english_headers_and_a_range_column():
    sheet = ("Axis,Range,Max speed,Max acceleration,Direction\n" +
             "".join(f"J{i},-100/100,150,500,1\n" for i in range(1, 7)))
    p = parse_params(sheet)
    assert (p.joints[2].min_deg, p.joints[2].max_deg) == (-100, 100)
    assert p.joints[2].max_speed_dps == 150 and p.joints[2].max_accel_dps2 == 500


def test_all_problems_are_reported_with_their_row():
    bad = SHEET.replace("J3,-85,75,223", "J3,75,-85,223").replace("J5,-115,115,270", "J5,-115,115,")
    bad = bad.replace("J6,±360,,342,1200,1", "J6,±360,,342,1200,2")
    with pytest.raises(ParamsError) as e:
        parse_params(bad)
    message = str(e.value)
    assert "fila 6 (J3): el mínimo (75) no es menor que el máximo (-85)" in message
    assert "fila 8 (J5): falta la velocidad máxima" in message
    assert "fila 9 (J6): el sentido tiene que ser 1 o -1" in message
    with pytest.raises(ParamsError, match="faltan los ejes J4, J5, J6"):
        parse_params("\n".join(SHEET.splitlines()[:6]))


def test_apply_keeps_cad_geometry_and_warns_about_differences():
    data = base_data()
    new, warnings = apply_params(data, parse_params(SHEET), "Parámetros BRTIRUS1510A")
    assert new["geometry_mm"] == data["geometry_mm"]                         # manda el CAD
    assert new["joints"][0]["max_accel_dps2"] == 450
    assert new["max_linear_speed_mms"] == 1800
    assert any(w.startswith("d4: el plano dice 700 mm y el CAD mide 634.6 mm") for w in warnings)
    assert not any(w.startswith("a2") for w in warnings)                      # 686 vs 686.2: ok
    assert not any(w.startswith("alcance") for w in warnings)                 # 1500 vs 1511: < 2 %
    assert any("sin confirmar en la planilla: J2, J3" in w for w in warnings)
    assert "J3: SIN CONFIRMAR" in new["notes"] and "J2: confirmado en parte" in new["notes"]
    assert new["params"]["confirmed"]["J1"] == "sí"


def test_a_wrong_reach_is_flagged():
    data = base_data()
    _new, warnings = apply_params(data, parse_params(SHEET.replace("Alcance,1500", "Alcance,1800")))
    assert any(w.startswith("alcance: el datasheet dice 1800 mm") for w in warnings)


def test_template_round_trips():
    data = base_data()
    p = parse_params(template_csv(data))
    assert [j.max_speed_dps for j in p.joints] == [j["max_speed_dps"] for j in data["joints"]]
    assert all(j.max_accel_dps2 is None for j in p.joints)


def test_written_user_model_uses_the_app_meshes_and_closes_ik(tmp_path, monkeypatch):
    import sim.kinematics as kinematics
    from sim.scene import has_real_meshes

    monkeypatch.setattr(kinematics, "USER_MODELS_DIR", tmp_path)
    new, _w = apply_params(base_data(), parse_params(SHEET))
    path = robot_params.write_user_model("BRTIRUS1510A", new)
    assert path.parent == tmp_path
    model = RobotModel.load("BRTIRUS1510A")                                  # el del usuario manda
    assert model.joints[0].max_accel_dps2 == 450 and model.has_accelerations
    assert model.max_linear_speed_mms == 1800
    assert has_real_meshes(model)                                            # mallas de la app
    ok, n, worst = verify_ik(model)
    assert ok == n and worst < 0.1


def test_command_line(tmp_path, monkeypatch, capsys):
    import sim.kinematics as kinematics

    monkeypatch.setattr(kinematics, "USER_MODELS_DIR", tmp_path / "modelos")
    sheet = tmp_path / "planilla.csv"
    sheet.write_text(SHEET, encoding="utf-8")
    assert robot_params.main([str(sheet), "--model", "BRTIRUS1510A", "--write"]) == 0
    out = capsys.readouterr().out
    assert "J1: -165..165°  190 °/s  450.0 °/s²  sentido 1" in out
    assert "Cinemática inversa: 60/60 poses" in out
    assert robot_params.main(["-", "--model", "BRTIRUS1510A"]) == 0
    assert capsys.readouterr().out.startswith("Parámetros del robot,BRTIRUS1510A")
