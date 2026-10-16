"""
Lo que el lenguaje aprendió del respaldo real: opciones de movimiento,
etiquetas y saltos, acciones desactivadas, comentarios, Cmd Set:Stop y los
nombres del pad (`AS "..."`). Cada acción generada tiene la forma que tenía en
el respaldo real (ver docs/PAD_FORMAT.md).
"""

import pytest

from compiler.codegen import CompileError, PadOnlyError, compile_source
from compiler.pad_codegen import PadOptions, compile_to_pad_report

NBSP8 = "&nbsp;" * 8
J = "POINT j = JOINT(0, 0, 0, 0, -90, 0)\n"
W = "POINT w = WORLD(500, 0, 400, 180, 0, 180)\n"


def build(src: str, **options):
    backup, warnings = compile_to_pad_report(src, PadOptions(**options))
    return backup, warnings


def unit(src: str, name: str | None = None, **options) -> list[dict]:
    """Las acciones del principal (sin el comentario inicial) o de un módulo."""
    backup, _ = build(src, **options)
    if name is None:
        return backup.act.main
    return next(m for m in backup.act.modules if m.name == name).actions


def proc(body: str, head: str = J + W) -> list[dict]:
    return unit(f"{head}PROC p()\n{body}ENDPROC\np()\n", "p")


# --- opciones de un movimiento ---------------------------------------------------------------


def test_move_options_become_the_fields_the_pad_stores():
    move = proc("MOVEL w SPEED 25 TOOL 2 COORD 1 SMOOTH 4 DELAY 1.5 NEAR_END Y034 ON 1.0\n")[0]
    assert move == {
        "action": 10, "bindIOInfo": 1069125, "ckStatus": "63", "customName": "w",
        "delay": "1.500", "distance": "1.000", "insertedIndex": 0, "passTrans": 0,
        "points": [{"pointName": "", "pos": {"m0": "500.000", "m1": "0.000", "m2": "400.000",
                                              "m3": "180.000", "m4": "0.000", "m5": "180.000",
                                              "m6": "0.000", "m7": "0.000"}}],
        "quotePoint": [0, 0, 0], "relativeType": 0, "smooth": 4, "speed": "25.0",
        "toolCoord": (2 << 16) | 1}
    # Las claves, en orden alfabético como las escribe el pad.
    assert list(move) == sorted(move)


@pytest.mark.parametrize("option, info, distance", [
    ("NEAR_END Y034 ON 1.0", 0x105045, "1.000"),
    ("NEAR_END Y034 OFF 2", 0x5045, "2.000"),
    ("AWAY_START Y034 ON 2.5", 0x105041, "2.500"),
    ("AWAY_START Y034 OFF 2", 0x5041, "2.000"),
    ("NEAR_END Y034 ON 0 DISABLED", 0x105044, "0.000"),
])
def test_distance_outputs_encode_like_the_pad(option, info, distance):
    move = proc(f"MOVEL w SPEED 25 {option}\n")[0]
    assert (move["bindIOInfo"], move["distance"]) == (info, distance)


def test_a_move_without_options_keeps_the_state_and_zero_fields():
    move = proc("TOOL 2\nCOORD 1\nMOVEJ j SPEED 100\n")[2]
    assert (move["smooth"], move["delay"], move["bindIOInfo"], move["distance"]) == (
        0, "0.000", 0, "0.000")
    assert move["toolCoord"] == (2 << 16) | 1


def test_tool_and_coord_of_one_move_do_not_change_the_state():
    actions = proc("TOOL 2\nCOORD 1\nMOVEL w SPEED 25 COORD 0\nMOVEL w SPEED 25\n")
    assert [a["toolCoord"] for a in actions if a["action"] == 10] == [2 << 16, (2 << 16) | 1]


@pytest.mark.parametrize("option, message", [
    ("SMOOTH 10", "SMOOTH 10: tiene que ser un número entero entre 0 y 9"),
    ("SMOOTH 1.5", "entero"),
    ("DELAY -1", "DELAY -1: tiene que estar entre 0 y 3600"),
    ("TOOL 16", "tiene que ser un número entero entre 0 y 15"),
    ("NEAR_END Y034 ON -1", "distancia"),
    ("NEAR_END X010 ON 1", r"no es una salida \(Y\)"),
    ("SMOOTH 1 SMOOTH 2", "SMOOTH repetido"),
    ("NEAR_END Y034 ON 1 AWAY_START Y034 OFF 2", "repetido"),
])
def test_bad_move_options_are_errors(option, message):
    with pytest.raises(CompileError, match=message):
        proc(f"MOVEL w SPEED 25 {option}\n")


# --- etiquetas y saltos ---------------------------------------------------------------------------


def test_labels_gotos_and_calls_with_a_jump():
    src = (J + "PROC a()\nSET_OUT(Y010, ON)\nENDPROC\n"
           "a()\nIF X030 == 1 GOTO Salto\na() THEN GOTO Salida\nLABEL Salto 1\nLABEL Salida 0\n")
    main = unit(src)[1:]            # sin el comentario «Generado por…»
    assert [a["action"] for a in main] == [20000, 10001, 20000, 59999, 59999, 60000]
    assert main[0]["flag"] == -1 and main[2]["flag"] == "0"          # texto, como el pad
    assert main[1] == {"action": 10001, "flag": 1, "inout": 0, "insertedIndex": 2, "limit": "0.000",
                       "point": 16, "pointStatus": 1, "type": 0}
    assert (main[3]["comment"], main[3]["flag"]) == ("Salto", 1)
    assert (main[4]["comment"], main[4]["flag"]) == ("Salida", 0)


def test_labels_without_a_number_take_a_free_one_and_leave_room_for_the_if_blocks():
    src = (J + "PROC a()\nLABEL x\nLABEL y 0\nLABEL z\nIF X010 == 0 THEN\nSET_OUT(Y010, ON)\nENDIF\nENDPROC\na()\n")
    labels = {a["comment"]: a["flag"] for a in unit(src, "a") if a["action"] == 59999}
    assert labels["y"] == 0 and labels["x"] == 1 and labels["z"] == 2
    assert labels["FinIf3"] == 3                       # el IF estructurado no pisa a ninguna


def test_labels_are_per_unit():
    src = J + "PROC a()\nLABEL x 0\nENDPROC\nPROC b()\nIF X010 == 1 GOTO x\nENDPROC\na()\nb()\n"
    with pytest.raises(CompileError, match="La etiqueta 'x' no está definida en el PROC b"):
        build(src)


@pytest.mark.parametrize("src, message", [
    ("LABEL a\nLABEL a\n", "declarada más de una vez"),
    ("LABEL a 1\nLABEL b 1\n", "el número 1 de la etiqueta 'b' ya lo usa otra"),
    ("IF X010 == 1 GOTO nada\n", "La etiqueta 'nada' no está definida en el programa principal"),
    ("LABEL a\nPROC p()\nENDPROC\np() THEN GOTO b\n", "La etiqueta 'b' no está definida"),
])
def test_label_errors(src, message):
    with pytest.raises(CompileError, match=message):
        build(src)


def test_goto_when_the_input_is_off_was_never_seen():
    src = "LABEL fin\nIF X010 == 0 GOTO fin\n"
    with pytest.raises(CompileError, match="Nunca lo vimos en un respaldo real"):
        build(src)
    backup, _ = build(src, allow_unverified=True)
    assert next(a for a in backup.act.main if a["action"] == 10001)["pointStatus"] == 0


# --- acciones desactivadas -----------------------------------------------------------------------


def test_a_disabled_set_out_is_stored_like_the_pad_does():
    actions = proc("SET_OUT(Y010, ON)\nDISABLED SET_OUT(Y034, OFF)\n")
    assert actions[1] == {
        "action": 50000, "comment": NBSP8 + "Output:Y034OFF DLY0.000",
        "commentAction": {"action": 200, "delay": "0.000", "insertedIndex": 1, "isWaitInput": False,
                          "point": 20, "pointStatus": False, "type": 0, "valveID": 20},
        "insertedIndex": 1}


def test_a_disabled_if_carries_the_flag_and_the_label_name():
    src = J + "PROC p()\nDISABLED IF X012 == 1 GOTO Reja\nLABEL Reja 3 AS \"Reja de 6\"\nENDPROC\np()\n"
    action = unit(src, "p")[0]
    assert action["comment"] == NBSP8 + "IF:X012ON Lim:0.000 Go to Flag[3]:Reja de 6"
    assert action["commentAction"] == {
        "action": 10001, "flag": 3, "inout": 0, "insertedIndex": 0, "limit": "0.000", "point": 2,
        "pointStatus": 1, "type": 0}


@pytest.mark.parametrize("src", ["DISABLED MOVEJ j SPEED 10\n", "DISABLED WAIT 1s\n",
                                 "DISABLED COMMENT \"x\"\n"])
def test_only_what_was_seen_disabled_can_be_disabled(src):
    with pytest.raises(CompileError):
        build(J + src)


# --- comentarios, Cmd Set:Stop y banner ----------------------------------------------------------


def test_comment_and_cmd_stop_shapes():
    main = unit('COMMENT "PROGRAMA DE PRUEBA"\nCMD STOP\n')
    assert main[0] == {"action": 50000, "comment": "PROGRAMA DE PRUEBA", "insertedIndex": 0}
    assert main[1] == {"action": 53000, "addr": 98304, "data": 2561, "insertedIndex": 1, "op": 0,
                       "specialType": 1, "type": 0, "typeSel": 1}


def test_our_banner_is_only_added_when_the_program_has_no_comment_of_its_own():
    assert unit("PROC a()\nSET_OUT(Y010, ON)\nENDPROC\na()\n")[0]["comment"] == "Generado por Borunte DSL"
    own = unit('PROC a()\nSET_OUT(Y010, ON)\nENDPROC\nCOMMENT "mío"\na()\n')
    assert own[0]["comment"] == "mío" and "Generado por Borunte DSL" not in str(own)


def test_comment_text_with_quotes_and_semicolons():
    assert unit('COMMENT "a; \\"b\\" \\\\ ñ"\n')[0]["comment"] == 'a; "b" \\ ñ'


# --- los nombres del pad ---------------------------------------------------------------------------


def test_pad_names_are_kept_with_as():
    src = ('POINT p = JOINT(0, 0, 0, 0, -90, 0) AS "Entrada/SalidaP1-1"\n'
           'POINT q = JOINT(1, 0, 0, 0, -90, 0) AS ""\nPOINT r = JOINT(2, 0, 0, 0, -90, 0)\n'
           'PROC Mesa_90M() AS "Mesa.90M"\nMOVEJ p SPEED 10\nMOVEJ q SPEED 10\nMOVEJ r SPEED 10\n'
           'MOVEJ JOINT(3, 0, 0, 0, -90, 0) SPEED 10\nENDPROC\nMesa_90M()\n')
    backup, _ = build(src)
    module = backup.act.modules[0]
    assert module.name == "Mesa.90M"
    assert [a["customName"] for a in module.actions if a["action"] == 4] == [
        "Entrada/SalidaP1-1", "", "r", ""]


# --- la VM de la PC -------------------------------------------------------------------------------


def test_the_vm_ignores_what_does_not_change_the_logic():
    src = (J + 'COMMENT "x"\nCMD STOP\nDISABLED SET_OUT(Y010, ON)\n'
           'PROC a() AS "A.b"\nMOVEJ j SPEED 10 SMOOTH 3 DELAY 1 TOOL 1 NEAR_END Y034 ON 1\nENDPROC\na()\n')
    program = compile_source(src)
    assert program is not None


@pytest.mark.parametrize("src", ["LABEL a\n", "IF X010 == 1 GOTO a\nLABEL a\n",
                                 "PROC p()\nENDPROC\nLABEL a\np() THEN GOTO a\n"])
def test_the_vm_says_that_jumps_are_for_the_pad(src):
    with pytest.raises(PadOnlyError, match="los ejecuta el pad"):
        compile_source(src)
    build(src)                                  # y para el pad compila


# --- sintaxis ---------------------------------------------------------------------------------------


@pytest.mark.parametrize("src", [
    "IF X010 == 1 GOTO\n", "LABEL\n", 'COMMENT hola\n', "CMD\n", "DISABLED\n",
    'POINT p = JOINT(0,0,0,0,0,0) AS\n', "MOVEJ x SPEED 10 SMOOTH\n", "p() THEN\n",
])
def test_incomplete_new_statements_are_syntax_errors(src):
    with pytest.raises(CompileError):
        build(J + src)
