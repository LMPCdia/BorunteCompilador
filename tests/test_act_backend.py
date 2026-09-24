"""
Tests del backend `.act` (compiler/act_backend.py) y de su lector
(compiler/act_reader.py).

Sin hardware y sin Modbus: el `.act` es un archivo, así que todo esto es
comparar estructuras de datos.

Lo que estos tests NO prueban: que el robot acepte lo que generamos. El formato
salió de ingeniería inversa sobre un export real, no de documentación de
Borunte. Un test verde acá significa "generamos exactamente la forma que tiene
el export real", no "el robot lo va a correr".
"""

import json
from pathlib import Path

import pytest

from compiler.act_backend import (
    ACTION_BASE,
    ACTION_CALL,
    ACTION_END_MAIN,
    ACTION_END_SUB,
    ACTION_IF_INPUT,
    ACTION_LABEL,
    ACTION_MOVEJ,
    ACTION_MOVEL,
    ACTION_SET_OUT,
    ACTION_TOOL,
    ACTION_WAIT,
    ActCompileError,
    compile_to_act,
    compile_to_act_text,
    pack_tool_coord,
)
from compiler.act_reader import ActFile, parse_act, read_act

#: Export real del robot. Vive fuera del repo (es de la máquina del usuario), así
#: que los tests que lo usan se saltean si no está. Sirve como la referencia más
#: fuerte que tenemos del formato.
EXPORT_REAL = Path(
    r"D:\Trabajo\HCBackupRobot_20260814213843\AberturaSanLorenzo2026.act"
)

PROGRAMA_COMPLETO = """POINT p_home = WORLD(0.0, 500.0, 300.0, 0.0, 0.0, 0.0)
POINT p_pieza = WORLD(100.0, 600.0, 200.0, 10.0, 20.0, 30.0)

BASE 1
TOOL 2

PROC HOME(id=1)
MOVEJ p_home SPEED 100
ENDPROC

HOME()
IF INPUT(X003) == ON THEN
MOVEL p_pieza SPEED 25
SET_OUT(Y020, ON)
WAIT 1.5s
SET_OUT(Y020, OFF)
ENDIF
HOME()
"""


def codigos(acciones) -> list[int]:
    return [a["action"] for a in acciones]


# --- estructura del archivo ------------------------------------------------


def test_compiles_to_an_act_file():
    act = compile_to_act(PROGRAMA_COMPLETO)
    assert isinstance(act, ActFile)
    assert act.main_line == 0
    assert act.library_line == 10


def test_main_program_ends_with_end_main():
    act = compile_to_act(PROGRAMA_COMPLETO)
    assert act.main[-1]["action"] == ACTION_END_MAIN
    assert codigos(act.main).count(ACTION_END_MAIN) == 1


def test_every_subprogram_ends_with_end_sub():
    act = compile_to_act(PROGRAMA_COMPLETO)
    for sub in act.library:
        assert sub.actions[-1]["action"] == ACTION_END_SUB
        assert codigos(sub.actions).count(ACTION_END_SUB) == 1


def test_inserted_index_matches_the_position():
    act = compile_to_act(PROGRAMA_COMPLETO)
    assert [a["insertedIndex"] for a in act.main] == list(range(len(act.main)))


def test_library_program_is_a_json_string_not_a_list():
    """En el export real `program` está codificado DOS veces: es un string que
    contiene JSON. Si lo generáramos como lista, el robot no lo leería."""
    act = compile_to_act(PROGRAMA_COMPLETO)
    entrada = act.documents[act.library_line][0]
    assert isinstance(entrada["program"], str)
    assert isinstance(json.loads(entrada["program"]), list)


def test_the_file_has_the_same_line_layout_as_a_real_export():
    act = compile_to_act(PROGRAMA_COMPLETO)
    assert len(act.documents) == 19
    assert act.documents[9] == {}
    for i in list(range(1, 9)) + list(range(11, 19)):
        assert act.documents[i] == [{"action": ACTION_END_MAIN, "insertedIndex": 0}]


# --- BASE y TOOL --------------------------------------------------------------


def test_base_and_tool_emit_their_native_actions():
    act = compile_to_act("BASE 1\nTOOL 2\n")
    base = [a for a in act.main if a["action"] == ACTION_BASE]
    tool = [a for a in act.main if a["action"] == ACTION_TOOL]
    assert base[0]["coordID"] == 1
    assert tool[0]["toolID"] == 2
    assert set(base[0]) == {"action", "coordID", "insertedIndex"}
    assert set(tool[0]) == {"action", "insertedIndex", "toolID"}


def test_moves_carry_the_current_base_and_tool():
    """Cada movimiento lleva su propio `toolCoord`: BASE/TOOL cambian el estado
    que se estampa en los movimientos siguientes."""
    src = (
        "POINT p = WORLD(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)\n"
        "BASE 1\nTOOL 2\nMOVEJ p SPEED 50\n"
        "BASE 2\nMOVEJ p SPEED 50\n"
    )
    act = compile_to_act(src)
    movs = [a for a in act.main if a["action"] in (ACTION_MOVEJ, ACTION_MOVEL)]
    assert movs[0]["toolCoord"] == pack_tool_coord(2, 1)
    assert movs[1]["toolCoord"] == pack_tool_coord(2, 2)


def test_tool_coord_packs_tool_in_the_high_word():
    assert pack_tool_coord(2, 1) == 131073
    assert pack_tool_coord(2, 2) == 131074
    assert pack_tool_coord(2, 0) == 131072


# --- movimientos ----------------------------------------------------------------


def test_movej_and_movel_use_different_actions():
    src = (
        "POINT p = WORLD(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)\n"
        "MOVEJ p SPEED 100\nMOVEL p SPEED 25\n"
    )
    act = compile_to_act(src)
    movs = [a for a in act.main if a["action"] in (ACTION_MOVEJ, ACTION_MOVEL)]
    assert movs[0]["action"] == ACTION_MOVEJ
    assert movs[1]["action"] == ACTION_MOVEL


def test_move_has_exactly_the_fields_of_a_real_move():
    act = compile_to_act(
        "POINT p = WORLD(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)\nMOVEJ p SPEED 50\n"
    )
    mov = [a for a in act.main if a["action"] == ACTION_MOVEJ][0]
    assert set(mov) == {
        "action", "bindIOInfo", "ckStatus", "customName", "delay", "distance",
        "insertedIndex", "passTrans", "points", "quotePoint", "relativeType",
        "smooth", "speed", "toolCoord",
    }


def test_move_pose_uses_three_decimal_strings():
    act = compile_to_act(
        "POINT p = WORLD(1.5, -2.25, 300.0, 0.0, 0.0, 180.0)\nMOVEJ p SPEED 50\n"
    )
    pos = [a for a in act.main if a["action"] == ACTION_MOVEJ][0]["points"][0]["pos"]
    assert pos["m0"] == "1.500"
    assert pos["m1"] == "-2.250"
    assert pos["m2"] == "300.000"
    assert pos["m5"] == "180.000"
    assert all(isinstance(v, str) for v in pos.values())


def test_m6_and_m7_are_always_zero():
    """En las 556 acciones de movimiento del export real no hay una excepción:
    son los ejes externos, que este robot no tiene."""
    act = compile_to_act(
        "POINT p = WORLD(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)\nMOVEJ p SPEED 50\n"
    )
    pos = [a for a in act.main if a["action"] == ACTION_MOVEJ][0]["points"][0]["pos"]
    assert pos["m6"] == "0.000"
    assert pos["m7"] == "0.000"


def test_speed_uses_one_decimal():
    act = compile_to_act(
        "POINT p = WORLD(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)\nMOVEJ p SPEED 100\n"
    )
    assert [a for a in act.main if a["action"] == ACTION_MOVEJ][0]["speed"] == "100.0"


def test_point_offset_is_resolved_at_compile_time():
    src = (
        "POINT base = WORLD(10.0, 20.0, 30.0, 0.0, 0.0, 0.0)\n"
        "POINT alto = base + OFFSET(0.0, 0.0, 100.0, 0.0, 0.0, 0.0)\n"
        "MOVEJ alto SPEED 50\n"
    )
    act = compile_to_act(src)
    pos = [a for a in act.main if a["action"] == ACTION_MOVEJ][0]["points"][0]["pos"]
    assert pos["m2"] == "130.000"


# --- SET_OUT y WAIT ---------------------------------------------------------------


def test_set_out_maps_to_action_200():
    act = compile_to_act("SET_OUT(Y020, ON)\nSET_OUT(Y020, OFF)\n")
    salidas = [a for a in act.main if a["action"] == ACTION_SET_OUT]
    assert salidas[0]["pointStatus"] is True
    assert salidas[1]["pointStatus"] is False
    # En el export real `point` y `valveID` son siempre el mismo número.
    assert salidas[0]["point"] == salidas[0]["valveID"] == 20


def test_wait_maps_to_action_100_with_the_limit_in_seconds():
    act = compile_to_act("WAIT 1.5s\n")
    espera = [a for a in act.main if a["action"] == ACTION_WAIT][0]
    assert espera["limit"] == "1.500"
    assert espera["isUnlimit"] is False
    assert espera["type"] == ACTION_WAIT


# --- IF sobre entradas ---------------------------------------------------------------


def test_if_over_input_emits_a_conditional_jump_and_a_label():
    act = compile_to_act(
        "IF INPUT(X003) == ON THEN\nSET_OUT(Y020, ON)\nENDIF\n"
    )
    assert codigos(act.main) == [
        ACTION_IF_INPUT, ACTION_SET_OUT, ACTION_LABEL, ACTION_END_MAIN
    ]


def test_the_condition_is_inverted_because_the_native_if_skips_forward():
    """El IF nativo salta cuando la condición se CUMPLE, y lo que queremos es
    saltear el cuerpo cuando NO se cumple. Por eso se invierte."""
    act = compile_to_act("IF INPUT(X003) == ON THEN\nSET_OUT(Y020, ON)\nENDIF\n")
    salto = act.main[0]
    assert salto["action"] == ACTION_IF_INPUT
    assert salto["point"] == 3
    assert salto["pointStatus"] == 0  # invertido respecto de ON

    act_off = compile_to_act("IF INPUT(X003) == OFF THEN\nSET_OUT(Y020, ON)\nENDIF\n")
    assert act_off.main[0]["pointStatus"] == 1  # invertido respecto de OFF


def test_the_jump_and_its_label_share_the_flag():
    act = compile_to_act("IF INPUT(X003) == ON THEN\nSET_OUT(Y020, ON)\nENDIF\n")
    salto = act.main[0]
    etiqueta = act.main[2]
    assert salto["flag"] == etiqueta["flag"]


def test_nested_ifs_get_distinct_labels():
    src = (
        "IF INPUT(X003) == ON THEN\n"
        "IF INPUT(X004) == ON THEN\n"
        "SET_OUT(Y020, ON)\n"
        "ENDIF\n"
        "ENDIF\n"
    )
    act = compile_to_act(src)
    flags = [a["flag"] for a in act.main if a["action"] == ACTION_LABEL]
    assert len(flags) == len(set(flags)) == 2


def test_input_condition_accepts_the_reversed_order():
    act = compile_to_act("IF ON == INPUT(X003) THEN\nSET_OUT(Y020, ON)\nENDIF\n")
    assert act.main[0]["point"] == 3


# --- PROC y llamadas ------------------------------------------------------------------


def test_proc_becomes_a_library_entry():
    act = compile_to_act(PROGRAMA_COMPLETO)
    assert [(s.id, s.name) for s in act.library] == [(1, "HOME")]


def test_call_references_the_library_id_as_a_string():
    act = compile_to_act(PROGRAMA_COMPLETO)
    llamadas = [a for a in act.main if a["action"] == ACTION_CALL]
    assert len(llamadas) == 2
    assert llamadas[0]["module"] == "1"
    assert isinstance(llamadas[0]["module"], str)
    assert llamadas[0]["flag"] == -1


def test_proc_body_does_not_appear_in_the_main_program():
    """El cuerpo del PROC vive en la biblioteca, no en línea: no hace falta
    saltearlo como en el bytecode del PLC."""
    act = compile_to_act(PROGRAMA_COMPLETO)
    assert ACTION_MOVEJ not in codigos(act.main)
    assert ACTION_MOVEJ in codigos(act.library[0].actions)


def test_call_before_declaration_resolves():
    src = "HOME()\nPROC HOME(id=1)\nSET_OUT(Y020, ON)\nENDPROC\n"
    act = compile_to_act(src)
    assert [a for a in act.main if a["action"] == ACTION_CALL][0]["module"] == "1"


# --- lo que NO se puede generar (y por qué) ----------------------------------------------


def test_else_is_rejected_with_an_explanation():
    src = (
        "IF INPUT(X003) == ON THEN\nSET_OUT(Y020, ON)\n"
        "ELSE\nSET_OUT(Y021, ON)\nENDIF\n"
    )
    with pytest.raises(ActCompileError, match="salto incondicional"):
        compile_to_act(src)


def test_if_over_an_internal_variable_is_rejected():
    """La gramática lo acepta (lo soporta el backend del PLC), pero acá no hay
    opcode nativo confirmado que compare variables."""
    src = "VAR a : INT = 1\nIF a == 1 THEN\nSET_OUT(Y020, ON)\nENDIF\n"
    with pytest.raises(ActCompileError):
        compile_to_act(src)


def test_variables_are_rejected():
    with pytest.raises(ActCompileError, match="variables internas"):
        compile_to_act("VAR a : INT = 1\n")


def test_wait_in_is_rejected():
    with pytest.raises(ActCompileError, match="WAIT_IN"):
        compile_to_act("WAIT_IN(X003, 5)\n")


def test_timer_is_rejected():
    with pytest.raises(ActCompileError, match="TIMER"):
        compile_to_act("TIMER t = 2s\n")


def test_proc_without_an_explicit_id_is_rejected():
    src = "PROC HOME()\nSET_OUT(Y020, ON)\nENDPROC\nHOME()\n"
    with pytest.raises(ActCompileError, match="id explícito"):
        compile_to_act(src)


def test_proc_with_parameters_is_rejected():
    src = "PROC HOME(altura)\nSET_OUT(Y020, ON)\nENDPROC\n"
    with pytest.raises(ActCompileError, match="no los soporta"):
        compile_to_act(src)


def test_two_procs_with_the_same_id_are_rejected():
    src = (
        "PROC A(id=1)\nSET_OUT(Y020, ON)\nENDPROC\n"
        "PROC B(id=1)\nSET_OUT(Y021, ON)\nENDPROC\n"
    )
    with pytest.raises(ActCompileError, match="ya lo usa"):
        compile_to_act(src)


def test_undefined_point_is_rejected():
    with pytest.raises(ActCompileError, match="Punto no definido"):
        compile_to_act("MOVEJ p_fantasma SPEED 50\n")


def test_undefined_proc_call_is_rejected():
    with pytest.raises(ActCompileError, match="no definido"):
        compile_to_act("NoExiste()\n")


# --- ida y vuelta: backend -> lector ----------------------------------------------------


def test_round_trip_through_the_reader():
    texto = compile_to_act_text(PROGRAMA_COMPLETO)
    releido = parse_act(texto)
    original = compile_to_act(PROGRAMA_COMPLETO)

    assert codigos(releido.main) == codigos(original.main)
    assert [(s.id, s.name) for s in releido.library] == [
        (s.id, s.name) for s in original.library
    ]
    assert releido.main == original.main


def test_round_trip_recovers_the_subprogram_actions():
    texto = compile_to_act_text(PROGRAMA_COMPLETO)
    releido = parse_act(texto)
    home = releido.subprogram("HOME")
    assert home.id == 1
    assert codigos(home.actions) == [ACTION_MOVEJ, ACTION_END_SUB]


def test_round_trip_is_stable_a_second_time():
    """Leer y reescribir no debe cambiar nada: si cambiara, alguna conversión
    no es simétrica."""
    texto = compile_to_act_text(PROGRAMA_COMPLETO)
    assert parse_act(texto).to_text() == texto


def test_generated_text_has_no_trailing_newline():
    """Igual que el export del robot."""
    assert not compile_to_act_text(PROGRAMA_COMPLETO).endswith("\n")


# --- contra el export real del robot -------------------------------------------------------


@pytest.mark.skipif(not EXPORT_REAL.is_file(), reason="no está el export real del robot")
def test_real_export_round_trips_byte_for_byte():
    """La prueba más fuerte que tenemos de que entendimos el formato: leer un
    export real de 338 KB y reescribirlo idéntico, incluida la biblioteca
    doblemente codificada."""
    original = EXPORT_REAL.read_text(encoding="utf-8-sig")
    assert read_act(EXPORT_REAL).to_text() == original


@pytest.mark.skipif(not EXPORT_REAL.is_file(), reason="no está el export real del robot")
def test_real_export_uses_only_actions_we_know_about():
    """Si aparece una acción que no está en el catálogo, la conocemos de menos."""
    from compiler.act_reader import ACTION_NAMES

    act = read_act(EXPORT_REAL)
    vistas = set(codigos(act.main))
    for sub in act.library:
        vistas |= set(codigos(sub.actions))
    assert vistas <= set(ACTION_NAMES), f"acciones desconocidas: {vistas - set(ACTION_NAMES)}"


@pytest.mark.skipif(not EXPORT_REAL.is_file(), reason="no está el export real del robot")
def test_generated_moves_have_the_same_shape_as_real_ones():
    """Los campos que generamos tienen que estar dentro de los que usa el robot,
    y cubrir todos los que el robot pone SIEMPRE.

    No se compara contra una muestra suelta a propósito: en el export real la
    acción 4 a veces no trae `customName`, así que una muestra cualquiera no
    define el esquema. Lo definen la unión (lo permitido) y la intersección (lo
    obligatorio)."""
    act_real = read_act(EXPORT_REAL)
    reales = [
        a
        for sub in act_real.library
        for a in sub.actions
        if a["action"] in (ACTION_MOVEJ, ACTION_MOVEL)
    ]
    permitidos = set().union(*(set(a) for a in reales))
    obligatorios = set(reales[0]).intersection(*(set(a) for a in reales))
    generado = [
        a
        for a in compile_to_act(
            "POINT p = WORLD(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)\nMOVEJ p SPEED 50\n"
        ).main
        if a["action"] == ACTION_MOVEJ
    ][0]
    assert set(generado) <= permitidos, (
        f"campos que el robot no usa: {set(generado) - permitidos}"
    )
    assert obligatorios <= set(generado), (
        f"faltan campos obligatorios: {obligatorios - set(generado)}"
    )


@pytest.mark.skipif(not EXPORT_REAL.is_file(), reason="no está el export real del robot")
def test_generated_set_out_has_the_same_shape_as_real_ones():
    act_real = read_act(EXPORT_REAL)
    reales = [
        a
        for sub in act_real.library
        for a in sub.actions
        if a["action"] == ACTION_SET_OUT
    ]
    generado = [
        a for a in compile_to_act("SET_OUT(Y020, ON)\n").main
        if a["action"] == ACTION_SET_OUT
    ][0]
    assert set(generado) == set(reales[0])
