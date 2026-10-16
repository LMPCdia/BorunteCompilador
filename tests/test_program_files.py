"""
Programa en .src (lógica) + .dat (sus puntos) + config.dat (puntos comunes de
la carpeta): compiler/program_files.py y la ventana principal.
"""

import pytest

from compiler import program_files as pf
from compiler.codegen import CompileError
from compiler.pad_codegen import PadOptions, compile_to_pad_report
from pad.listing import list_backup

SRC = """; Paletizado
MOVEJ home SPEED 20
MOVEL p_mesa + OFFSET(0, 0, 100, 0, 0, 0) SPEED 50
MOVEL p_mesa SPEED 10
SET_OUT(Y010, ON)
MOVEJ home SPEED 20
"""
DAT = """; Puntos del paletizado
POINT p_mesa = WORLD(1097.1, -150.0, 721.9, 180.0, -10.0, 180.0)
"""
CONFIG = """; Puntos de toda la celda
POINT home = JOINT(0.347, 45.894, -44.865, -0.792, -75.952, -0.859)
"""


def sources(src=SRC, dat=DAT, config=CONFIG) -> pf.ProgramSources:
    return pf.ProgramSources(pf.SourceFile("pal.src", src), pf.SourceFile("pal.dat", dat),
                             pf.SourceFile("config.dat", config))


def test_the_three_files_compile_like_one():
    backup, warnings = pf.compile_pad(sources())
    single, _ = compile_to_pad_report(CONFIG + DAT + SRC, PadOptions())
    assert list_backup(backup) == list_backup(single)
    assert not any("no se usa" in w for w in warnings)


def test_errors_say_which_file_and_line():
    with pytest.raises(CompileError) as e:
        pf.compile_pad(sources(src=SRC + "MOVEL p_caja SPEED 10\n"))
    assert str(e.value).startswith("pal.src, línea 7:") and "p_caja" in str(e.value)
    assert (e.value.file, e.value.line) == ("pal.src", 7)


def test_syntax_error_in_the_dat_is_located_there():
    with pytest.raises(CompileError) as e:
        pf.compile_pad(sources(dat=DAT + "POINT mal = WORLD(1, 2, 3)\n"))
    assert str(e.value).startswith("pal.dat, línea 3")
    assert e.value.file == "pal.dat"


def test_the_dat_only_has_points():
    with pytest.raises(CompileError, match=r"pal\.dat, línea 3: un \.dat solo tiene puntos"):
        pf.compile_pad(sources(dat=DAT + "MOVEJ home SPEED 10\n"))
    with pytest.raises(CompileError, match=r"config\.dat, línea 1: un \.dat solo tiene puntos"):
        pf.compile_pad(sources(config="SET_OUT(Y010, ON)\n" + CONFIG))


@pytest.mark.parametrize("extra", ["POINT p2 = WORLD(1, 2, 3, 0, 0, 0)\n",
                                   "PROC subir()\nPOINT p2 = WORLD(1, 2, 3, 0, 0, 0)\nENDPROC\n"])
def test_the_src_does_not_declare_points(extra):
    with pytest.raises(CompileError) as e:
        pf.compile_pad(sources(src=extra + SRC))
    assert "pal.src, línea" in str(e.value) and "se declara en pal.dat" in str(e.value)


def test_a_point_name_is_not_repeated_between_config_and_dat():
    dat = DAT + "POINT home = JOINT(0, 0, 0, 0, 0, 0)\n"
    with pytest.raises(CompileError) as e:
        pf.compile_pad(sources(dat=dat))
    assert str(e.value) == ("pal.dat, línea 3: el punto home ya está en config.dat, línea 2")


def test_unused_points_of_the_dat_are_a_warning_but_not_those_of_config():
    dat = DAT + "POINT sobra = JOINT(0, 0, 0, 0, 0, 0)\n"
    config = CONFIG + "POINT otro_programa = JOINT(0, 0, 0, 0, 0, 0)\n"
    _backup, warnings = pf.compile_pad(sources(dat=dat, config=config))
    assert "pal.dat, línea 3: el punto sobra no se usa en pal.src" in warnings
    assert not any("otro_programa" in w for w in warnings)


def test_a_point_used_only_by_another_point_counts_as_used():
    dat = DAT + "POINT p_arriba = p_mesa + OFFSET(0, 0, 100, 0, 0, 0)\n"
    src = SRC.replace("p_mesa + OFFSET(0, 0, 100, 0, 0, 0)", "p_arriba")
    _backup, warnings = pf.compile_pad(sources(src=src, dat=dat))
    assert not any("no se usa" in w for w in warnings)


def test_compiler_warnings_are_located_too():
    # El principal con movimientos propios da un aviso con "Línea N".
    _backup, warnings = pf.compile_pad(sources())
    located = [w for w in warnings if "línea" in w]
    assert located and all(w.startswith("pal.src, línea") for w in located)


def test_the_vm_compiles_the_three_files_too():
    program, warnings = pf.compile_vm(sources())
    assert set(program.point_names) == {"home", "p_mesa"}
    assert warnings == []


def test_a_single_file_program_works_as_before():
    single = pf.ProgramSources(pf.SourceFile("viejo.krlb", CONFIG + DAT + SRC))
    assert not single.split
    backup, _ = pf.compile_pad(single)
    assert list_backup(backup) == list_backup(pf.compile_pad(sources())[0])


def test_split_a_single_file_program():
    split = pf.split_single(CONFIG + DAT + SRC, "pal.src")
    assert split.dat.text.splitlines() == [CONFIG.splitlines()[1], DAT.splitlines()[1]]
    assert "POINT" not in split.src.text and "MOVEJ home" in split.src.text
    assert (split.src.name, split.dat.name, split.config.text) == ("pal.src", "pal.dat", "")
    pf.compile_pad(split)


# --- disco ---------------------------------------------------------------------------------


def test_save_and_load_the_three_files(tmp_path):
    written = pf.save(sources(), tmp_path / "pal.src")
    assert sorted(p.name for p in written) == ["config.dat", "pal.dat", "pal.src"]
    for path in (tmp_path / "pal.src", tmp_path / "pal.dat"):   # se abre por cualquiera
        loaded = pf.load(path)
        assert (loaded.src.text, loaded.dat.text, loaded.config.text) == (SRC, DAT, CONFIG)
        assert loaded.src.name == "pal.src"


def test_empty_config_is_not_created_and_a_missing_one_reads_empty(tmp_path):
    written = pf.save(sources(config=""), tmp_path / "solo.src")
    assert not (tmp_path / "config.dat").exists() and len(written) == 2
    assert pf.load(tmp_path / "solo.src").config.text == ""


def test_a_dat_without_its_src_is_explained(tmp_path):
    (tmp_path / "huerfano.dat").write_text(DAT, encoding="utf-8")
    with pytest.raises(FileNotFoundError, match="huerfano.src"):
        pf.load(tmp_path / "huerfano.dat")


def test_other_extensions_are_one_file(tmp_path):
    (tmp_path / "viejo.krlb").write_text(SRC, encoding="utf-8")
    assert not pf.load(tmp_path / "viejo.krlb").split


# --- ventana principal ------------------------------------------------------------------------


@pytest.fixture
def window(tmp_path):
    from PySide6.QtWidgets import QApplication

    QApplication.instance() or QApplication([])
    from gui.main_window import MainWindow

    w = MainWindow()
    yield w
    w.close()


def test_new_program_starts_split_and_compiles(window):
    assert window.program_tabs.count() == 3
    assert "POINT" not in window.editor.toPlainText()
    assert "POINT p_home" in window.dat_editor.toPlainText()
    window._on_compile()
    assert window._program is not None


def test_save_and_open_from_the_window(window, tmp_path):
    window.dat_editor.setPlainText(DAT)
    window.config_editor.setPlainText(CONFIG)
    window.editor.setPlainText(SRC)
    assert window.save_program(tmp_path / "pal.src")
    assert {p.name for p in tmp_path.iterdir()} == {"pal.src", "pal.dat", "config.dat"}
    assert window.program_tabs.tabText(0) == "pal.src" and window.pad_program_name() == "pal"

    other = type(window)()
    assert other.open_program(tmp_path / "pal.dat")
    assert other.editor.toPlainText() == SRC and other.dat_editor.toPlainText() == DAT
    assert other.config_editor.toPlainText() == CONFIG
    other._on_compile()
    assert other._program is not None
    other.close()


def test_config_dat_of_the_folder_is_not_overwritten_unless_edited(window, tmp_path):
    (tmp_path / "config.dat").write_text("POINT home = JOINT(1, 2, 3, 4, 5, 6)\n", encoding="utf-8")
    window.config_editor.setPlainText(CONFIG)
    window.config_editor.document().setModified(False)    # no lo tocó el usuario
    window.dat_editor.setPlainText(DAT)
    window.editor.setPlainText(SRC)
    assert window.save_program(tmp_path / "pal.src")
    assert "JOINT(1, 2, 3" in (tmp_path / "config.dat").read_text(encoding="utf-8")
    assert "JOINT(1, 2, 3" in window.config_editor.toPlainText()
    warnings = [m for m in window.messages.messages() if m[0] == "Advertencia"]
    assert any("config.dat" in m[2] for m in warnings)
    window.config_editor.selectAll()                         # ahora sí lo edita (tipeando)
    window.config_editor.insertPlainText(CONFIG)
    assert window.config_editor.document().isModified()
    assert window.save_program(tmp_path / "pal.src")
    assert (tmp_path / "config.dat").read_text(encoding="utf-8") == CONFIG


def test_old_single_file_programs_open_and_can_be_split(window, tmp_path):
    (tmp_path / "viejo.krlb").write_text(CONFIG + DAT + SRC, encoding="utf-8")
    assert window.open_program(tmp_path / "viejo.krlb")
    assert not window.program_tabs.isTabVisible(1) and window.act_split.isEnabled()
    window._on_compile()
    assert window._program is not None
    window.split_program()
    assert window.program_tabs.isTabVisible(1) and not window.act_split.isEnabled()
    assert window.program_tabs.tabText(0) == "viejo.src"
    assert "POINT" not in window.editor.toPlainText()
    assert window.save_program(tmp_path / "viejo.src")
    assert (tmp_path / "viejo.dat").exists()


def test_save_as_a_single_file_joins_the_three(window, tmp_path):
    window.editor.setPlainText(SRC)
    window.dat_editor.setPlainText(DAT)
    window.config_editor.setPlainText(CONFIG)
    assert window.save_program(tmp_path / "todo.krlb")
    text = (tmp_path / "todo.krlb").read_text(encoding="utf-8")
    assert "POINT home" in text and "POINT p_mesa" in text and "MOVEJ home" in text
    assert not window.program_tabs.isTabVisible(1)


def test_the_simulator_reports_the_file_and_line(window):
    window.editor.setPlainText(SRC + "MOVEL p_caja SPEED 10\n")
    window.dat_editor.setPlainText(DAT)
    window.config_editor.setPlainText(CONFIG)
    assert window.sim_view.simulate() is None
    errors = [m for m in window.messages.messages() if m[0] == "Error"]
    assert any("pal" not in m[2] and "programa.src, línea 7" in m[2] for m in errors)


def test_check_cli_simulates_a_src_with_its_dats(tmp_path, capsys):
    from sim.check import main as check_main

    pf.save(sources(), tmp_path / "pal.src")
    assert check_main([str(tmp_path / "pal.src"), "--model", "BRTIRUS1820A"]) in (0, 1)
    out = capsys.readouterr()
    assert "Error:" not in out.err
