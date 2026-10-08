"""
Formato de respaldo del pad (`pad/`).

El respaldo real que se usó para deducir el formato NO está en el repo (es de
un cliente). Estos tests usan un respaldo sintético con la misma forma: mismas
claves, mismo orden, mismas líneas del `.act`. Si llega un respaldo de otra
versión del pad que no encaja, el lugar para agregarlo es acá.
"""

import io
import json
import zipfile
from datetime import datetime

import pytest

from pad.backup import ActFile, Module, PadBackup
from pad.listing import describe, io_name, list_backup, split_tool_coord

MOVE = {
    "action": 4, "bindIOInfo": 0, "ckStatus": "63", "customName": "Casa", "delay": "0.000",
    "distance": "0.000", "insertedIndex": 1, "passTrans": 0,
    "points": [{"pointName": "", "pos": {"m0": "0.347", "m1": "45.894", "m2": "-44.865",
                                         "m3": "-0.792", "m4": "-75.952", "m5": "-0.859",
                                         "m6": "0.000", "m7": "0.000"}}],
    "quotePoint": [0, 0, 0], "relativeType": 0, "smooth": 0, "speed": "25.0", "toolCoord": 131073,
}
OUT = {"action": 200, "delay": "0.000", "insertedIndex": 2, "isWaitInput": False, "point": 20,
       "pointStatus": False, "type": 0, "valveID": 20}


def _act_text() -> str:
    main = [
        {"action": 50000, "comment": "Programa de prueba", "insertedIndex": 0},
        {"action": 20000, "flag": -1, "insertedIndex": 1, "module": "0"},
        {"action": 10001, "flag": 0, "inout": 0, "insertedIndex": 2, "limit": "0.000",
         "point": 2, "pointStatus": 1, "type": 0},
        {"action": 59999, "comment": "Salida", "flag": 0, "insertedIndex": 3},
        {"action": 60000, "insertedIndex": 4},
    ]
    empty = [{"action": 60000, "insertedIndex": 0}]
    module = [MOVE, OUT, {"action": 20001, "insertedIndex": 0}]
    modules = [{"id": 0, "name": "HOME",
                "program": json.dumps(module, separators=(",", ":"), ensure_ascii=False)}]
    lines = [main] + [empty] * 8 + [{}] + [modules] + [empty] * 8
    return "\n".join(json.dumps(x, separators=(",", ":"), ensure_ascii=False) for x in lines)


def _zip_bytes(name="Prueba", act=None, extra=None) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr(f"{name}.act", act if act is not None else _act_text())
        zf.writestr(f"{name}.fnc", "514, 0\n357, 131071\n")
        for ext in ("counters", "palletStyle", "timers", "variables"):
            zf.writestr(f"{name}.{ext}", b"")
        for entry, data in (extra or {}).items():
            zf.writestr(entry, data)
    return buf.getvalue()


# --- .act ---------------------------------------------------------------------


def test_act_roundtrip_is_byte_exact():
    text = _act_text()
    assert ActFile.loads(text).dumps() == text


def test_act_keeps_non_ascii_unescaped():
    text = _act_text().replace("Programa de prueba", "Calibración ñandú")
    out = ActFile.loads(text).dumps()
    assert "Calibración ñandú" in out
    assert out == text


def test_modules_are_parsed_and_rewritten():
    act = ActFile.loads(_act_text())
    [home] = act.modules
    assert (home.id, home.name) == (0, "HOME")
    assert home.actions[0]["action"] == 4

    home.actions.append({"action": 100, "insertedIndex": 3, "limit": "0.500"})
    act.modules = [home, Module(id=1, name="Otro", actions=[{"action": 20001, "insertedIndex": 0}])]
    again = ActFile.loads(act.dumps())
    assert [m.name for m in again.modules] == ["HOME", "Otro"]
    assert again.module_by_id(0).actions[-1]["limit"] == "0.500"
    # `program` se guarda como string con JSON adentro, como lo hace el pad.
    assert isinstance(again.lines[10][0]["program"], str)


def test_act_without_modules_line_is_rejected():
    with pytest.raises(ValueError, match="módulos"):
        ActFile.loads('[{"action":60000,"insertedIndex":0}]')


# --- zip ----------------------------------------------------------------------


def test_backup_roundtrip_keeps_every_file():
    original = _zip_bytes()
    backup = PadBackup.from_bytes(original, timestamp=datetime(2026, 8, 14, 21, 38, 43))
    assert backup.name == "Prueba"
    assert sorted(backup.others) == ["counters", "fnc", "palletStyle", "timers", "variables"]

    with zipfile.ZipFile(io.BytesIO(original)) as a, zipfile.ZipFile(io.BytesIO(backup.to_bytes())) as b:
        assert sorted(a.namelist()) == b.namelist()
        for entry in a.namelist():
            assert a.read(entry) == b.read(entry), entry


def test_backup_file_name_is_what_the_pad_accepts(tmp_path):
    backup = PadBackup.from_bytes(_zip_bytes(), timestamp=datetime(2026, 8, 14, 21, 38, 43))
    path = backup.write(tmp_path)
    assert path.name == "HCBackupRobot_20260814213843.zip"
    assert PadBackup.read(path).timestamp == datetime(2026, 8, 14, 21, 38, 43)


def test_read_tolerates_a_prefix_before_the_backup_name(tmp_path):
    path = tmp_path / "copia-HCBackupRobot_20260101000000.zip"
    path.write_bytes(_zip_bytes())
    assert PadBackup.read(path).timestamp == datetime(2026, 1, 1)


def test_backup_with_two_programs_is_rejected():
    data = _zip_bytes(extra={"Otro.act": _act_text()})
    with pytest.raises(ValueError, match="exactamente un .act"):
        PadBackup.from_bytes(data)


def test_backup_with_a_foreign_file_is_rejected():
    with pytest.raises(ValueError, match="inesperado"):
        PadBackup.from_bytes(_zip_bytes(extra={"leeme.txt": "hola"}))


# --- interpretación (hipótesis documentadas en docs/PAD_FORMAT.md) -------------


def test_io_names_match_what_the_pad_shows():
    # Los tres casos observados en el respaldo real.
    assert io_name("X", 2) == "X012"
    assert io_name("X", 3) == "X013"
    assert io_name("Y", 20) == "Y034"


def test_tool_coord_is_tool_high_coord_low():
    assert split_tool_coord(131072) == (2, 0)
    assert split_tool_coord(131073) == (2, 1)
    assert split_tool_coord(0) == (0, 0)


def test_listing_names_the_known_actions():
    text = "\n".join(list_backup(PadBackup.from_bytes(_zip_bytes())))
    assert "CALL HOME()" in text
    assert "IF X012 ON GOTO Salida" in text
    assert "Salida:" in text
    assert "MOVEJ (0.347, 45.894, -44.865, -0.792, -75.952, -0.859) SPEED 25.0 TOOL 2 COORD 1" in text
    assert "SET_OUT(Y034, OFF)" in text
    assert "ENDPROC" in text and "END" in text
    assert "??" not in text


def test_listing_shows_non_default_move_fields():
    move = dict(MOVE, smooth=4, bindIOInfo=20545, distance="2.000")
    line = describe(move, {}, {})
    assert "smooth=4" in line and "bindIOInfo=20545" in line and "distance=2.000" in line


def test_listing_does_not_hide_unknown_actions():
    line = describe({"action": 53000, "addr": 98304, "data": 2561, "insertedIndex": 2}, {}, {})
    assert line.startswith("?? acción 53000")
    assert '"addr": 98304' in line


def test_disabled_action_is_shown_as_such():
    comment = {"action": 50000, "comment": "&nbsp;&nbsp;Output:Y034OFF DLY0.000",
               "commentAction": OUT, "insertedIndex": 9}
    assert describe(comment, {}, {}) == "; [desactivada] SET_OUT(Y034, OFF) delay=0.000"
