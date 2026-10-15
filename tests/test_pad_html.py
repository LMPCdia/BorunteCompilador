"""
`bindIOInfo` decodificado (pad/bindio.py) y el cruce del respaldo con el
listado HTML del pad (pad/pad_html.py).

Los datos son sintéticos con la forma de los reales: el programa de un cliente
no va al repo. Si en esta máquina están los archivos reales (variables de
entorno BORUNTE_REAL_BACKUP y BORUNTE_REAL_LISTING), también se cruzan.
"""

import os
from pathlib import Path

import pytest

from pad import bindio, pad_html
from pad.backup import ActFile, Module, PadBackup

# --- bindIOInfo -------------------------------------------------------------------------------

SEEN = {
    0x5041: bindio.BindIO(point=20, end=False, on=False),     # lejos del inicial -> OFF
    0x5045: bindio.BindIO(point=20, end=True, on=False),
    0x105041: bindio.BindIO(point=20, end=False, on=True),
    0x105045: bindio.BindIO(point=20, end=True, on=True),     # cerca del final -> ON
    0x105044: bindio.BindIO(point=20, end=True, on=True, enabled=False),
}


@pytest.mark.parametrize("value, bind", SEEN.items(), ids=[hex(v) for v in SEEN])
def test_the_values_seen_in_the_real_backup_decode_and_encode_back(value, bind):
    assert bindio.decode(value) == bind
    assert bind.encode() == value


def test_the_pad_decimal_values():
    # En el .act van como números decimales.
    assert bindio.decode(20545) == SEEN[0x5041] and bindio.decode(1069125) == SEEN[0x105045]


@pytest.mark.parametrize("value", [0, -1, 0x5043, 0x5081, 0x5041 | (1 << 30), 0x41])
def test_unknown_values_are_left_alone(value):
    # 0x41 = salida 0: es válido; los demás tienen bits sueltos o falta el 0x40.
    decoded = bindio.decode(value)
    if value == 0x41:
        assert decoded == bindio.BindIO(point=0, end=False, on=False)
    else:
        assert decoded is None


def test_the_text_the_pad_shows():
    assert bindio.pad_text(SEEN[0x105045], 1.0, "Y034") == (
        "whiledistanceend pointthe distancelimit then 1.000mm then output Y034 on")
    assert bindio.pad_text(SEEN[0x5041], 2.0, "Y034") == (
        "whiledistancestart pointthe distancemore then 2.000mm then output Y034 off")


# --- listado del pad ------------------------------------------------------------------------


def move(code, name, values, speed, bind=0, distance="0.000", smooth=0, delay="0.000", tc=131074):
    return {"action": code, "bindIOInfo": bind, "ckStatus": "63", "customName": name,
            "delay": delay, "distance": distance, "insertedIndex": 0, "passTrans": 0,
            "points": [{"pointName": "", "pos": {**{f"m{i}": f"{v:.3f}" for i, v in enumerate(values)},
                                                  "m6": "0.000", "m7": "0.000"}}],
            "quotePoint": [0, 0, 0], "relativeType": 0, "smooth": smooth,
            "speed": f"{speed:.1f}", "toolCoord": tc}


def synthetic():
    main = [
        {"action": 50000, "comment": "PROGRAMA DE PRUEBA", "insertedIndex": 0},
        {"action": 53000, "addr": 98304, "data": 2561, "insertedIndex": 1, "op": 0,
         "specialType": 1, "type": 0, "typeSel": 1},
        {"action": 20000, "flag": -1, "insertedIndex": 2, "module": "0"},
        {"action": 10001, "flag": 1, "inout": 0, "insertedIndex": 3, "limit": "0.000",
         "point": 16, "pointStatus": 1, "type": 0},
        {"action": 20000, "flag": "0", "insertedIndex": 4, "module": "0"},
        {"action": 59999, "comment": "Otro", "flag": 1, "insertedIndex": 5},
        {"action": 59999, "comment": "Salida", "flag": 0, "insertedIndex": 6},
        {"action": 60000, "insertedIndex": 7},
    ]
    module = [
        {"action": 800, "coordID": 1, "insertedIndex": 0},
        {"action": 801, "toolID": 2, "insertedIndex": 1},
        move(4, "Safty", [1, 2, 3, 4, 5, 6], 100.0, smooth=4),
        move(10, "P1-1", [100.5, -20, 300, 10, 20, 30], 25.0, bind=0x105045, distance="1.000"),
        move(10, "", [100.5, -20, 300.25, 10, 20, 30], 0.3, delay="1.000", tc=131073),
        {"action": 100, "customName": "", "insertedIndex": 5, "isUnlimit": False, "limit": "0.300",
         "point": 0, "pointStatus": 0, "type": 100},
        {"action": 200, "delay": "0.000", "insertedIndex": 6, "isWaitInput": False, "point": 20,
         "pointStatus": False, "type": 0, "valveID": 20},
        {"action": 50000, "insertedIndex": 7,
         "comment": "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Output:Y034OFF DLY0.000",
         "commentAction": {"action": 200, "delay": "0.000", "insertedIndex": 7, "isWaitInput": False,
                           "point": 20, "pointStatus": False, "type": 0, "valveID": 20}},
        {"action": 10001, "flag": 0, "inout": 0, "insertedIndex": 8, "limit": "0.000", "point": 2,
         "pointStatus": 1, "type": 0},
        {"action": 59999, "comment": "Fin", "flag": 0, "insertedIndex": 9},
        {"action": 20001, "insertedIndex": 10},
    ]
    lines = [[{"action": 60000, "insertedIndex": 0}] for _ in range(19)]
    lines[10] = [Module(0, "Soldar", module).to_json()]
    lines[0] = main
    return PadBackup(name="Prueba", act=ActFile(lines), others={})


def row(i, *texts):
    first = f"&nbsp;{i:>2}:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;{texts[0]}<br>"
    return first + "".join(f"&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;{t}<br>" for t in texts[1:])


LISTING = (
    "<html><body>mainProgram:<br>"
    + row(0, "PROGRAMA DE PRUEBA") + row(1, "Cmd Set:Stop")
    + row(2, "Call Fun[0]:Soldar And then return to next line")
    + row(3, "IF:X030ON Lim:0.000 Go to Flag[1]:")
    + row(4, "Call Fun[0]:Soldar And then return to Flag[0]:")
    + row(5, "Flag[1]:Otro") + row(6, "Flag[0]:Salida") + row(7, "Program End") + "<br>"
    + "subProgram1:<br>" + row(0, "Program End") + "<br>"
    + "fuction[0]:Soldar<br>"
    + row(0, "Switch Coord:[CoordID1]Mesa") + row(1, "Switch Tool:[2:Torcha]")
    + row(2, "*Safty Free Path:", "Next:J1:1.000,J2:2.000,J3:3.000,J4:4.000,J5:5.000,J6:6.000",
          "V100.0 DLY0.000 Smooth:4")
    + row(3, "*P1-1 Line3D-Pose:", "Next:X:100.500,Y:-20.000,Z:300.000,U:10.000,V:20.000,W:30.000",
          "V25.0 DLY0.000 Smooth:None tool:2:Torcha coord:2:Mesa",
          "whiledistanceend pointthe distancelimit then 1.000mm then output Y034 on")
    + row(4, "Line3D-Pose:", "Next:X:100.500,Y:-20.000,Z:300.250,U:10.000,V:20.000,W:30.000",
          "V0.3 DLY1.000 Smooth:None tool:2:Torcha coord:1:Mesa")
    + row(5, "DLY:0.300s") + row(6, "Output:Y034OFF DLY0.000")
    + row(7, "Output:Y034OFF DLY0.000") + row(8, "IF:X012ON Lim:0.000 Go to Flag[0]:")
    + row(9, "Flag[0]:Fin") + row(10, "Module End")
    + "</body></html>"
)


def test_parse_listing_groups_the_lines_of_each_instruction():
    units = pad_html.parse_listing(LISTING)
    assert set(units) == {"main", 0}
    assert len(units["main"]) == 8 and len(units[0]) == 11
    assert units[0][3][0] == "*P1-1 Line3D-Pose:" and len(units[0][3]) == 4   # el movimiento ocupa 4 líneas


def test_a_consistent_backup_and_listing_pass():
    rep = pad_html.check(synthetic(), pad_html.parse_listing(LISTING))
    assert rep.ok, rep.problems
    assert rep.entries == 19
    for kind in ("valores del punto", "suavizado", "salida por distancia", "desactivada",
                 "CALL", "IF", "etiqueta", "WAIT", "SET_OUT", "COORD", "TOOL", "comentario"):
        assert rep.checked[kind] >= 1, kind


@pytest.mark.parametrize("old, new, what", [
    ("V25.0", "V26.0", "velocidad"),
    ("Smooth:4", "Smooth:3", "suavizado"),
    ("X:100.500", "X:100.600", "valores del punto"),
    ("limit then 1.000mm then output Y034 on", "limit then 1.000mm then output Y034 off", "salida por distancia"),
    ("DLY:0.300s", "DLY:0.400s", "WAIT"),
    ("Output:Y034OFF DLY0.000<br>&nbsp;" , "Output:Y035OFF DLY0.000<br>&nbsp;", "SET_OUT"),
    ("Go to Flag[1]:", "Go to Flag[2]:", "IF"),
    ("coord:2:Mesa", "coord:3:Mesa", "herramienta y coordenada"),
])
def test_a_difference_is_reported(old, new, what):
    assert old in LISTING
    rep = pad_html.check(synthetic(), pad_html.parse_listing(LISTING.replace(old, new, 1)))
    assert not rep.ok and any(what in p for p in rep.problems), rep.problems


def test_a_different_number_of_entries_is_reported():
    broken = LISTING.replace(row(5, "DLY:0.300s"), "")
    rep = pad_html.check(synthetic(), pad_html.parse_listing(broken))
    assert any("acciones y" in p for p in rep.problems)


def test_a_disabled_if_carries_the_label_name():
    backup = synthetic()
    module = backup.act.modules[0]
    module.actions.insert(9, {
        "action": 50000, "insertedIndex": 99,
        "comment": "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;IF:X013ON Lim:0.000 Go to Flag[0]:Fin",
        "commentAction": {"action": 10001, "flag": 0, "inout": 0, "insertedIndex": 99,
                          "limit": "0.000", "point": 3, "pointStatus": 1, "type": 0}})
    backup.act.modules = [module]
    listing = LISTING.replace(row(9, "Flag[0]:Fin") + row(10, "Module End"),
                              row(9, "IF:X013ON Lim:0.000 Go to Flag[0]:") + row(10, "Flag[0]:Fin")
                              + row(11, "Module End"))
    rep = pad_html.check(backup, pad_html.parse_listing(listing))
    assert rep.ok, rep.problems
    # Con otro nombre de etiqueta en el texto guardado, no coincide.
    module.actions[9]["comment"] = module.actions[9]["comment"].replace("Fin", "Otra")
    backup.act.modules = [module]
    assert not pad_html.check(backup, pad_html.parse_listing(listing)).ok


# --- los archivos reales, si están ----------------------------------------------------------------

REAL = (os.environ.get("BORUNTE_REAL_BACKUP"), os.environ.get("BORUNTE_REAL_LISTING"))


@pytest.mark.skipif(not all(REAL) or not all(Path(p).exists() for p in REAL if p),
                    reason="sin el respaldo y el listado reales (BORUNTE_REAL_BACKUP / _LISTING)")
def test_real_backup_matches_the_pads_own_listing():
    backup = PadBackup.read(REAL[0])
    rep = pad_html.check(backup, pad_html.parse_listing(Path(REAL[1]).read_text(encoding="utf-8")))
    assert rep.ok, rep.problems[:5]
    # Todo bindIOInfo del programa se entiende (si no, hay un valor nuevo para decodificar).
    values = {int(a.get("bindIOInfo") or 0) for acts in [backup.act.main, *[m.actions for m in backup.act.modules]]
              for a in acts if a["action"] in (4, 10)}
    assert all(v == 0 or bindio.decode(v) for v in values), values
