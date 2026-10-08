"""
Generador del respaldo del pad (`compiler/pad_codegen.py`).

Lo que fijan estos tests es que lo generado tenga la MISMA forma que el
respaldo real analizado (claves, tipos, formato de los números), y que todo lo
que no sabemos traducir sea un error claro y no una adivinanza.
"""

import pytest

from compiler.codegen import CompileError
from compiler.pad_codegen import PadOptions, compile_to_pad, io_point
from pad.backup import PadBackup
from pad.listing import io_name, list_backup

HOME = "POINT casa = JOINT(0.347, 45.894, -44.865, -0.792, -75.952, -0.859)\n"
PIEZA = "POINT pieza = WORLD(1307.0, 1155.72, -309.813, 162.642, -14.032, 148.128)\n"

# Claves exactas de cada acción en el respaldo real.
REAL_KEYS = {
    4: {"action", "bindIOInfo", "ckStatus", "customName", "delay", "distance", "insertedIndex",
        "passTrans", "points", "quotePoint", "relativeType", "smooth", "speed", "toolCoord"},
    100: {"action", "customName", "insertedIndex", "isUnlimit", "limit", "point", "pointStatus",
          "type"},
    200: {"action", "delay", "insertedIndex", "isWaitInput", "point", "pointStatus", "type",
          "valveID"},
    800: {"action", "coordID", "insertedIndex"},
    801: {"action", "insertedIndex", "toolID"},
    10001: {"action", "flag", "inout", "insertedIndex", "limit", "point", "pointStatus", "type"},
    20000: {"action", "flag", "insertedIndex", "module"},
    20001: {"action", "insertedIndex"},
    50000: {"action", "comment", "insertedIndex"},
    59999: {"action", "comment", "flag", "insertedIndex"},
    60000: {"action", "insertedIndex"},
}
REAL_KEYS[10] = REAL_KEYS[4]


def _main(source, **options):
    return compile_to_pad(source, PadOptions(**options)).act.main


def _all_actions(backup):
    yield from backup.act.main
    for module in backup.act.modules:
        yield from module.actions


FULL = HOME + PIEZA + """
PROC soldar()
MOVEL pieza SPEED 50
SET_OUT(Y034, ON)
WAIT 0.3s
SET_OUT(Y034, OFF)
ENDPROC
MOVEJ casa SPEED 25
IF X012 == 1 THEN
soldar()
ENDIF
"""


# --- forma del archivo ------------------------------------------------------------


def test_every_action_has_the_same_keys_as_the_real_backup():
    backup = compile_to_pad(FULL)
    codes = set()
    for action in _all_actions(backup):
        codes.add(action["action"])
        assert set(action) == REAL_KEYS[action["action"]], action
        assert list(action) == sorted(action), "el pad escribe las claves en orden alfabético"
    assert codes >= {4, 10, 100, 200, 10001, 20000, 20001, 60000}


def test_inserted_index_is_unique_and_sequential():
    backup = compile_to_pad(FULL)
    for actions in [backup.act.main] + [m.actions for m in backup.act.modules]:
        assert [a["insertedIndex"] for a in actions] == list(range(len(actions)))


def test_main_ends_with_end_and_modules_with_endproc():
    backup = compile_to_pad(FULL)
    assert backup.act.main[-1]["action"] == 60000
    assert all(m.actions[-1]["action"] == 20001 for m in backup.act.modules)


def test_backup_survives_a_zip_roundtrip_and_has_every_file():
    backup = compile_to_pad(FULL, PadOptions(program_name="Prueba"))
    again = PadBackup.from_bytes(backup.to_bytes())
    assert again.name == "Prueba"
    assert sorted(again.others) == ["counters", "fnc", "palletStyle", "timers", "variables"]
    assert again.others["fnc"].startswith(b"514, 0\n")
    assert len(again.act.lines) == 19
    assert again.act.lines[9] == {}
    assert again.act.dumps() == backup.act.dumps()


def test_template_fnc_keeps_the_pad_line_endings():
    # Si falla en Windows: falta `pad/template.fnc -text` en .gitattributes y
    # git convirtió el archivo a "\r\n" al hacer checkout.
    from pad.backup import TEMPLATE_FNC

    data = TEMPLATE_FNC.read_bytes()
    assert b"\r" not in data
    assert data.endswith(b"\n")


def test_template_lines_and_files_are_copied():
    template = compile_to_pad(HOME + "MOVEJ casa SPEED 10\n")
    template.act.lines[9] = {"algo": 1}
    template.others["fnc"] = b"1, 2\n"
    backup = compile_to_pad(FULL, PadOptions(template=template))
    assert backup.act.lines[9] == {"algo": 1}
    assert backup.others["fnc"] == b"1, 2\n"
    # El programa es el nuevo, no el de la plantilla.
    assert [m.name for m in backup.act.modules] == ["soldar"]


def test_invalid_program_name_is_rejected():
    with pytest.raises(CompileError, match="Nombre de programa"):
        compile_to_pad(HOME, PadOptions(program_name="Soldadura ñ"))


# --- movimientos --------------------------------------------------------------------


def test_movej_writes_joint_angles_with_three_decimals():
    [move] = [a for a in _main(HOME + "MOVEJ casa SPEED 25\n") if a["action"] == 4]
    assert move["points"][0]["pos"] == {
        "m0": "0.347", "m1": "45.894", "m2": "-44.865", "m3": "-0.792",
        "m4": "-75.952", "m5": "-0.859", "m6": "0.000", "m7": "0.000",
    }
    assert move["speed"] == "25.0"
    assert move["customName"] == "casa"


def test_movel_applies_offsets_at_compile_time():
    src = PIEZA + "MOVEL pieza + OFFSET(0, 0, 10, 0, 0, 0) SPEED 2\n"
    [move] = [a for a in _main(src) if a["action"] == 10]
    assert move["points"][0]["pos"]["m2"] == "-299.813"
    assert move["customName"] == ""  # no es un punto declarado tal cual


def test_negative_zero_is_written_as_zero():
    src = "MOVEL WORLD(0.0001, -0.0001, 0, 0, 0, 0) SPEED 10\n"
    [move] = [a for a in _main(src) if a["action"] == 10]
    assert move["points"][0]["pos"]["m1"] == "0.000"


def test_tool_and_coord_go_in_every_move_and_in_the_preamble():
    main = _main(HOME + "MOVEJ casa SPEED 25\n", tool=2, coord=1)
    assert {"action": 800, "coordID": 1, "insertedIndex": 1} in main
    assert {"action": 801, "insertedIndex": 2, "toolID": 2} in main
    [move] = [a for a in main if a["action"] == 4]
    assert move["toolCoord"] == 131073  # 0x20001, como en el respaldo real


def test_movej_to_a_world_point_is_rejected():
    with pytest.raises(CompileError, match="MOVEJ necesita un punto JOINT"):
        compile_to_pad(PIEZA + "MOVEJ pieza SPEED 10\n")


def test_movel_to_a_joint_point_is_rejected():
    with pytest.raises(CompileError, match="MOVEL necesita un punto WORLD"):
        compile_to_pad(HOME + "MOVEL casa SPEED 10\n")


def test_offset_keeps_the_frame_of_its_base():
    with pytest.raises(CompileError, match="MOVEL necesita un punto WORLD"):
        compile_to_pad(HOME + "MOVEL casa + OFFSET(0, 0, 1, 0, 0, 0) SPEED 10\n")


@pytest.mark.parametrize("speed", ["0", "150"])
def test_speed_out_of_range_is_rejected(speed):
    with pytest.raises(CompileError, match="SPEED"):
        compile_to_pad(HOME + f"MOVEJ casa SPEED {speed}\n")


# --- E/S, esperas, IF ------------------------------------------------------------------


def test_io_point_is_the_inverse_of_the_listing_name():
    for point in (0, 2, 3, 20, 63):
        assert io_point(io_name("Y", point), "Y") == point
        assert io_point(io_name("X", point), "X") == point
    assert io_point("Y034", "Y") == 20
    assert io_point("x012", "X") == 2


@pytest.mark.parametrize("name", ["Y08", "Y7", "X010", "Y-1", "M10"])
def test_bad_output_names_are_rejected(name):
    with pytest.raises(CompileError):
        io_point(name, "Y")


def test_set_out_and_wait():
    main = _main("SET_OUT(Y034, ON)\nWAIT 1.5s\nWAIT UNTIL MOVE_DONE\n")
    out = next(a for a in main if a["action"] == 200)
    assert (out["point"], out["valveID"], out["pointStatus"]) == (20, 20, True)
    waits = [a for a in main if a["action"] == 100]
    assert len(waits) == 1, "WAIT UNTIL MOVE_DONE no emite nada"
    assert waits[0]["limit"] == "1.500" and waits[0]["type"] == 100


def test_if_input_on_skips_the_block_when_the_input_is_off():
    main = _main("IF X012 == 1 THEN\nSET_OUT(Y010, ON)\nENDIF\n")
    codes = [a["action"] for a in main]
    i = codes.index(10001)
    jump, body, label = main[i], main[i + 1], main[i + 2]
    assert (jump["point"], jump["pointStatus"]) == (2, 0)
    assert body["action"] == 200
    assert label["action"] == 59999 and label["flag"] == jump["flag"]


def test_if_input_off_uses_the_form_seen_in_the_real_backup():
    main = _main("IF 0 == X012 THEN\nSET_OUT(Y010, ON)\nENDIF\n")
    jump = next(a for a in main if a["action"] == 10001)
    assert jump["pointStatus"] == 1  # "si X012 ON, saltar el bloque"


def test_labels_are_numbered_per_program():
    src = """PROC p()
IF X010 == 1 THEN
SET_OUT(Y010, ON)
ENDIF
ENDPROC
IF X010 == 1 THEN
p()
ENDIF
IF X011 == 1 THEN
p()
ENDIF
"""
    backup = compile_to_pad(src)
    assert [a["flag"] for a in backup.act.main if a["action"] == 59999] == [0, 1]
    [module] = backup.act.modules
    assert [a["flag"] for a in module.actions if a["action"] == 59999] == [0]


@pytest.mark.parametrize("src, match", [
    ("VAR n : INT = 1\n", "variables"),
    ("VAR n : INT = 1\nn = 2\n", "variables"),
    ("WAIT_IN(X010, 5)\n", "WAIT_IN"),
    ("IF X010 == 1 THEN\nSET_OUT(Y010, ON)\nELSE\nSET_OUT(Y011, ON)\nENDIF\n", "ELSE"),
    ("IF X010 == 2 THEN\nSET_OUT(Y010, ON)\nENDIF\n", "una entrada"),
    ("IF Y010 == 1 THEN\nSET_OUT(Y010, ON)\nENDIF\n", "entrada"),
    ("PROC p(a)\nENDPROC\n", "parámetros"),
])
def test_what_the_pad_cannot_express_is_a_clear_error(src, match):
    with pytest.raises(CompileError, match=match):
        compile_to_pad(src)


# --- módulos ---------------------------------------------------------------------------


def test_forward_calls_resolve_to_the_module_id():
    src = "segundo()\nPROC primero()\nENDPROC\nPROC segundo()\nENDPROC\n"
    backup = compile_to_pad(src)
    assert [(m.id, m.name) for m in backup.act.modules] == [(0, "primero"), (1, "segundo")]
    call = next(a for a in backup.act.main if a["action"] == 20000)
    assert call["module"] == "1" and call["flag"] == -1


def test_undefined_call_is_rejected():
    with pytest.raises(CompileError, match="no definido"):
        compile_to_pad("nada()\n")


def test_listing_of_the_generated_backup_has_no_unknown_actions():
    text = "\n".join(list_backup(compile_to_pad(FULL)))
    assert "??" not in text
    assert "IF X012 OFF GOTO FinIf0" in text
    assert "CALL soldar()" in text


# --- TOOL / COORD ------------------------------------------------------------------


def test_tool_and_coord_change_the_following_moves():
    src = HOME + PIEZA + "MOVEJ casa SPEED 10\nTOOL 2\nCOORD 1\nMOVEL pieza SPEED 10\n"
    main = compile_to_pad(src).act.main
    moves = [a for a in main if a["action"] in (4, 10)]
    assert [m["toolCoord"] for m in moves] == [0, 131073]
    assert {"action": 801, "insertedIndex": 4, "toolID": 2} in main


def test_each_proc_starts_with_the_export_defaults():
    src = PIEZA + "TOOL 2\nPROC p()\nMOVEL pieza SPEED 10\nENDPROC\np()\n"
    backup = compile_to_pad(src, PadOptions(tool=1, coord=3))
    [module] = backup.act.modules
    [move] = [a for a in module.actions if a["action"] == 10]
    assert move["toolCoord"] == (1 << 16) | 3


def test_tool_inside_if_is_rejected():
    with pytest.raises(CompileError, match="TOOL dentro de un IF"):
        compile_to_pad("IF X010 == 1 THEN\nTOOL 2\nENDIF\n")


@pytest.mark.parametrize("src", ["TOOL 2.5\n", "COORD 70000\n"])
def test_tool_and_coord_must_be_whole_numbers(src):
    with pytest.raises(CompileError, match="número entero"):
        compile_to_pad(src)


def test_tool_and_coord_are_ignored_by_the_reference_vm():
    from compiler.codegen import compile_source

    program = compile_source("TOOL 2\nCOORD 1\nSET_OUT(Y10, ON)\n")
    assert [i.opcode for i in program.instructions] == ["SET_OUT", "END"]
