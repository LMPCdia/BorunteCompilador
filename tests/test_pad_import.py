"""
Respaldo del pad -> .src + .dat (compiler/pad_import.py) y ida y vuelta: lo
que se importa, al compilarse de nuevo, da el mismo programa.

Los respaldos son sintéticos: el programa de un cliente no va al repo. Con las
variables de entorno BORUNTE_REAL_BACKUP (y BORUNTE_REAL_LISTING) también se
prueba contra el respaldo real.
"""

import copy
import os
import zipfile
from pathlib import Path

import pytest

from compiler import pad_import
from compiler import program_files as pf
from compiler.codegen import CompileError
from compiler.pad_codegen import PadOptions, compile_to_pad, compile_to_pad_report
from pad import pad_html
from pad.backup import PadBackup

FULL = '''POINT home = JOINT(0, 0, 0, 0, -90, 0) AS "HOME pose"
POINT a = WORLD(500, 0, 400, 180, 0, 180) AS "Entrada/SalidaP1-1"
POINT b = WORLD(500, 100, 400, 180, 0, 180)

PROC Mesa_90M() AS "Mesa.90M"
    COORD 1
    TOOL 2
    MOVEJ home SPEED 100 SMOOTH 3
    MOVEL a SPEED 50 COORD 0 DELAY 1 NEAR_END Y034 ON 1
    MOVEL b SPEED 25 AWAY_START Y034 OFF 2.5 DISABLED
    MOVEL WORLD(1, 2, 3, 0, 0, 0) SPEED 0.3 TOOL 0
    MOVEL a SPEED 50
    WAIT 0.3s
    DISABLED SET_OUT(Y034, OFF)
    SET_OUT(Y034, ON)
    IF X012 == 1 GOTO fin
    DISABLED IF X013 == 1 GOTO fin
    LABEL fin 4 AS "Fin de fila"
ENDPROC

PROC otro()
    SET_OUT(Y035, OFF)
ENDPROC

COMMENT "Programa de prueba"
CMD STOP
Mesa_90M()
IF X030 == 1 GOTO salto
otro() THEN GOTO salida
LABEL salto 1
LABEL salida 0
otro()
'''


def backup_of(src: str, name: str = "Prueba") -> PadBackup:
    return compile_to_pad(src, PadOptions(program_name=name))


def edit_modules(backup: PadBackup, fn) -> PadBackup:
    """`act.modules` devuelve copias: se edita y se vuelve a asignar."""
    modules = backup.act.modules
    fn(modules)
    backup.act.modules = modules
    return backup


def roundtrip(backup: PadBackup) -> tuple[pad_import.Imported, PadBackup]:
    imported = pad_import.import_backup(backup, "x.zip")
    return imported, pad_import.rebuild(imported, backup.name)


# --- ida y vuelta ------------------------------------------------------------------------------------


def test_every_construct_survives_import_and_export():
    original = backup_of(FULL)
    imported, rebuilt = roundtrip(original)
    assert pad_import.compare(original, rebuilt) == []
    assert imported.warnings == []
    assert imported.actions == sum(len(a) for a in [original.act.main, *[m.actions for m in original.act.modules]])


def test_the_imported_text_is_readable_and_has_points_in_the_dat():
    imported, _ = roundtrip(backup_of(FULL))
    src, dat = imported.sources.src.text, imported.sources.dat.text
    assert "POINT" not in src and "POINT" in dat
    assert 'PROC Mesa_90M() AS "Mesa.90M"' in src
    assert "MOVEL Entrada_SalidaP1_1 SPEED 50 COORD 0 DELAY 1 NEAR_END Y034 ON 1" in src
    assert "MOVEL HOME_pose SPEED" not in src and "MOVEJ HOME_pose SPEED 100 SMOOTH 3" in src
    assert 'POINT HOME_pose = JOINT(0, 0, 0, 0, -90, 0) AS "HOME pose"' in dat
    assert 'POINT Entrada_SalidaP1_1 = WORLD(500, 0, 400, 180, 0, 180) AS "Entrada/SalidaP1-1"' in dat
    # El nombre del identificador sale del nombre que muestra el pad.
    assert "DISABLED SET_OUT(Y034, OFF)" in src and "DISABLED IF X013 == 1 GOTO Fin_de_fila" in src
    assert 'LABEL Fin_de_fila 4 AS "Fin de fila"' in src
    assert 'COMMENT "Programa de prueba"' in src and "CMD STOP" in src
    assert "otro() THEN GOTO salida" in src and "AWAY_START Y034 OFF 2.5 DISABLED" in src
    # La herramienta y el sistema vigentes no se repiten en cada movimiento.
    assert src.count("TOOL 2") == 1 and src.count("COORD 1") == 1


def test_the_same_point_is_declared_once():
    imported, _ = roundtrip(backup_of(FULL))
    assert imported.sources.dat.text.count("Entrada_SalidaP1_1") == 1


def test_the_imported_files_pass_the_program_file_rules():
    imported, _ = roundtrip(backup_of(FULL))
    assert pf.check(imported.sources) == []           # .dat solo con POINT, sin repetidos ni sin usar


def test_the_regenerated_backup_lists_the_same_as_the_original():
    from pad.listing import list_backup

    original = backup_of(FULL)
    _, rebuilt = roundtrip(original)
    assert list_backup(original) == list_backup(rebuilt)


# --- nombres ---------------------------------------------------------------------------------------


def _with_names(names: list[str], module_names: list[str], labels: list[str]) -> PadBackup:
    """Un respaldo con nombres del pad que no son identificadores."""
    original = backup_of(
        "POINT p = JOINT(0, 0, 0, 0, -90, 0)\nPROC m()\nMOVEJ p SPEED 10\nENDPROC\nm()\n")
    module = original.act.modules[0]
    template = copy.deepcopy(module.actions[0])
    actions = []
    for i, name in enumerate(names):
        a = copy.deepcopy(template)
        a["customName"] = name
        a["points"][0]["pos"]["m0"] = f"{i}.000"          # puntos distintos
        actions.append(a)
    for i, text in enumerate(labels):
        actions.append({"action": 59999, "comment": text, "flag": i})
    actions.append({"action": 20001})
    for i, a in enumerate(actions):
        a["insertedIndex"] = i
    modules = []
    from pad.backup import Module

    for i, name in enumerate(module_names):
        modules.append(Module(i, name, copy.deepcopy(actions)))
    original.act.modules = modules
    original.act.main = (
        [{"action": 50000, "comment": "Programa de prueba", "insertedIndex": 0}]
        + [{"action": 20000, "flag": -1, "insertedIndex": i + 1, "module": str(i)}
           for i in range(len(modules))]
        + [{"action": 60000, "insertedIndex": len(modules) + 1}])
    return original


def test_names_that_are_not_identifiers_are_sanitized_and_kept_with_as():
    names = ["Entrada/SalidaP1-1", "A B", "", "1abc", "STOP", "ñandú", "x", "x", "IF", "_"]
    original = _with_names(names, ["Mesa.90M", "Mesa 90M", "HOME", "9mesa"],
                           ["Reja.1", "Reja 1", "fin", ""])
    imported, rebuilt = roundtrip(original)
    assert pad_import.compare(original, rebuilt) == []
    src = imported.sources.src.text
    assert 'PROC Mesa_90M() AS "Mesa.90M"' in src and 'PROC Mesa_90M_2() AS "Mesa 90M"' in src
    assert "PROC HOME()" in src and 'AS "9mesa"' in src
    assert 'AS "Entrada/SalidaP1-1"' in imported.sources.dat.text
    assert imported.warnings == []


def test_points_with_the_same_name_and_different_values_are_different_points():
    original = _with_names(["P1", "P1"], ["m"], [])
    imported, rebuilt = roundtrip(original)
    assert imported.points == 2 and pad_import.compare(original, rebuilt) == []


# --- lo que no se puede expresar ------------------------------------------------------------------------


def test_what_cannot_be_imported_is_reported_and_left_as_a_note():
    def edit(modules):
        actions = modules[0].actions
        next(a for a in actions if a["action"] == 10)["bindIOInfo"] = 0x5043   # no lo entendemos
        actions.insert(1, {"action": 12345, "foo": 1, "insertedIndex": 99})
        actions.insert(2, {"action": 50000, "comment": "x", "insertedIndex": 98,
                           "commentAction": {"action": 100, "limit": "1.000", "insertedIndex": 98}})

    original = edit_modules(backup_of(FULL), edit)
    imported = pad_import.import_backup(original, "x.zip")
    text = " | ".join(imported.warnings)
    assert "bindIOInfo 20547" in text and "acción 12345 desconocida" in text
    assert "acción 100 desactivada" in text
    src = imported.sources.src.text
    assert "; NO IMPORTADO: acción 12345" in src and "; NO IMPORTADO: bindIOInfo=20547" in src
    assert "; NO IMPORTADO (desactivada)" in src
    # Y lo importado se puede compilar igual (las notas son comentarios).
    pf.compile_pad(imported.sources)


def test_a_zero_status_if_is_imported_but_needs_the_unverified_switch_to_export():
    def edit(modules):
        for a in modules[0].actions:
            if a["action"] == 10001:
                a["pointStatus"] = 0

    original = edit_modules(backup_of("PROC p()\nLABEL x\nIF X010 == 1 GOTO x\nENDPROC\np()\n"), edit)
    imported = pad_import.import_backup(original, "x.zip")
    assert any("pointStatus 0" in w for w in imported.warnings)
    assert "IF X010 == 0 GOTO" in imported.sources.src.text
    with pytest.raises(CompileError, match="Nunca lo vimos"):
        pf.compile_pad(imported.sources)
    rebuilt = pad_import.rebuild(imported, original.name, allow_unverified=True)
    assert pad_import.compare(original, rebuilt) == []


def test_extra_move_fields_and_odd_values_are_warned():
    def edit(modules):
        move = next(a for a in modules[0].actions if a["action"] == 4)
        move["points"][0]["pos"]["m6"] = "5.000"
        move["speed"] = "12.34"
        move["passTrans"] = 1

    original = edit_modules(backup_of(FULL), edit)
    text = " | ".join(pad_import.import_backup(original, "x.zip").warnings)
    assert "ejes extra" in text and "velocidad '12.34'" in text and "passTrans=1" in text


def test_a_move_without_customname_is_equivalent_to_an_empty_one():
    def edit(modules):
        for a in modules[0].actions:
            if a["action"] == 10 and a["customName"] == "":
                del a["customName"]

    original = edit_modules(backup_of(FULL), edit)
    imported, rebuilt = roundtrip(original)
    assert any("no trae customName" in w for w in imported.warnings)
    assert pad_import.compare(original, rebuilt) == []


def test_compare_finds_real_differences():
    original = backup_of(FULL)
    _, rebuilt = roundtrip(original)
    rebuilt.act.main[0]["comment"] = "otro"

    def edit(modules):
        next(a for a in modules[0].actions if a["action"] == 10)["speed"] = "99.0"

    edit_modules(rebuilt, edit)
    diffs = pad_import.compare(original, rebuilt)
    assert len(diffs) == 2 and "speed" in diffs[1]


# --- .fnc y nombres del listado ------------------------------------------------------------------------


def test_the_fnc_travels_with_the_program(tmp_path):
    original = backup_of(FULL)
    original.others["fnc"] = b"1, 2\n3, 4\n"
    imported, rebuilt = roundtrip(original)
    assert imported.sources.fnc == b"1, 2\n3, 4\n" and rebuilt.others["fnc"] == b"1, 2\n3, 4\n"
    pf.save(imported.sources, tmp_path / "Prueba.src")
    assert (tmp_path / "Prueba.fnc").read_bytes() == b"1, 2\n3, 4\n"
    loaded = pf.load(tmp_path / "Prueba.src")
    assert loaded.fnc == b"1, 2\n3, 4\n"
    backup, _ = pf.compile_pad(loaded, PadOptions(program_name="Prueba"))
    assert backup.others["fnc"] == b"1, 2\n3, 4\n"


def test_without_a_fnc_the_example_one_is_used():
    backup, _ = pf.compile_pad(pf.ProgramSources(pf.SourceFile("a.src", "")), PadOptions())
    from pad.backup import TEMPLATE_FNC

    assert backup.others["fnc"] == TEMPLATE_FNC.read_bytes()


def test_tool_and_coord_names_from_the_pads_listing_become_comments():
    original = backup_of(FULL)
    listing = ("<html><body>fuction[0]:Mesa<br>&nbsp;0:&nbsp;&nbsp;Switch Coord:[CoordID1]MesaDer<br>"
               "&nbsp;1:&nbsp;&nbsp;Switch Tool:[2:MiTCP]<br></body></html>")
    imported = pad_import.import_backup(original, "x.zip", listing)
    src = imported.sources.src.text
    assert "; COORD 1 = MesaDer" in src and "; TOOL 2 = MiTCP" in src
    assert "COORD 1    ; MesaDer" in src


# --- línea de comandos ---------------------------------------------------------------------------------------


def test_command_line_writes_the_files_and_checks_the_round_trip(tmp_path, capsys):
    zip_path = backup_of(FULL).write(tmp_path)
    out = tmp_path / "salida"
    assert pad_import.main([str(zip_path), "--out", str(out), "--check"]) == 0
    text = capsys.readouterr().out
    assert "Ida y vuelta: idéntico" in text
    assert {p.name for p in out.iterdir()} == {"Prueba.src", "Prueba.dat", "Prueba.fnc"}


# --- ventana principal -------------------------------------------------------------------------------------


@pytest.fixture
def window():
    from PySide6.QtWidgets import QApplication

    QApplication.instance() or QApplication([])
    from gui.main_window import MainWindow

    w = MainWindow()
    yield w
    w.close()


def test_import_from_the_window_then_edit_save_and_export(window, tmp_path):
    original = backup_of(FULL)
    original.others["fnc"] = b"7, 8\n"
    zip_path = original.write(tmp_path)
    assert window.import_pad_backup(zip_path)
    assert window.program_tabs.isTabVisible(1) and "MOVEL" in window.editor.toPlainText()
    assert "POINT" in window.dat_editor.toPlainText() and window.pad_program_name() == "Prueba"
    assert window.program_tabs.tabText(0) == "Prueba.src"

    # F7: tiene saltos, la VM de la PC no los ejecuta: se revisa para el pad, sin error.
    window._on_compile()
    assert window._program is None
    errors = [m for m in window.messages.messages() if m[0] == "Error"]
    assert errors == []
    assert any("Compilado para el pad" in m[2] for m in window.messages.messages())
    assert "los ejecuta el pad" in window.bytecode_view.toPlainText()

    # Se edita (un punto) y se guarda: .src, .dat y .fnc.
    window.dat_editor.setPlainText(window.dat_editor.toPlainText().replace(
        "WORLD(500, 100, 400", "WORLD(500, 150, 400"))
    assert window.save_program(tmp_path / "Prueba.src")
    assert {p.name for p in tmp_path.glob("Prueba.*")} == {"Prueba.src", "Prueba.dat", "Prueba.fnc"}
    assert (tmp_path / "Prueba.fnc").read_bytes() == b"7, 8\n"

    # Se exporta: el .fnc del programa y el cambio hecho.
    (tmp_path / "salida").mkdir()
    exported = window.export_to_pad(tmp_path / "salida")
    assert exported is not None
    backup = PadBackup.read(exported)
    assert backup.others["fnc"] == b"7, 8\n"
    moves = [a for m in backup.act.modules for a in m.actions if a["action"] == 10]
    assert any(a["points"][0]["pos"]["m1"] == "150.000" for a in moves)


def test_import_uses_the_html_next_to_the_zip(window, tmp_path):
    zip_path = backup_of(FULL).write(tmp_path)
    (tmp_path / "Prueba.html").write_text(
        "<html><body>fuction[0]:M<br>&nbsp;0:&nbsp;&nbsp;Switch Tool:[2:MiTCP]<br></body></html>",
        encoding="utf-8")
    assert window.import_pad_backup(zip_path)
    assert "; TOOL 2 = MiTCP" in window.editor.toPlainText()


def test_a_broken_backup_is_explained(window, tmp_path):
    bad = tmp_path / "HCBackupRobot_20260101000000.zip"
    with zipfile.ZipFile(bad, "w") as z:
        z.writestr("algo.txt", "no es un respaldo")
    assert not window.import_pad_backup(bad)
    errors = [m for m in window.messages.messages() if m[0] == "Error"]
    assert errors and "No se pudo importar" in errors[-1][2]


def test_import_warnings_reach_the_messages(window, tmp_path):
    original = edit_modules(backup_of(FULL), lambda ms: ms[0].actions.insert(
        1, {"action": 12345, "insertedIndex": 99}))
    assert window.import_pad_backup(original.write(tmp_path))
    warnings = [m for m in window.messages.messages() if m[0] == "Advertencia"]
    assert any("acción 12345 desconocida" in m[2] for m in warnings)


# --- el respaldo real, si está -------------------------------------------------------------------------------

REAL = (os.environ.get("BORUNTE_REAL_BACKUP"), os.environ.get("BORUNTE_REAL_LISTING"))


@pytest.mark.skipif(not REAL[0] or not Path(REAL[0]).exists(), reason="sin BORUNTE_REAL_BACKUP")
def test_the_real_backup_round_trips():
    original = PadBackup.read(REAL[0])
    listing = Path(REAL[1]).read_text(encoding="utf-8") if REAL[1] and Path(REAL[1]).exists() else None
    imported = pad_import.import_backup(original, Path(REAL[0]).name, listing)
    rebuilt = pad_import.rebuild(imported, original.name)
    assert pad_import.compare(original, rebuilt) == []
    assert rebuilt.others == original.others
    if listing:
        report = pad_html.check(rebuilt, pad_html.parse_listing(listing))
        assert report.ok, report.problems[:5]
