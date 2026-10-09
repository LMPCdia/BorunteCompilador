"""
Test de humo de la GUI, en modo headless (sin display real, usando el plugin
"offscreen" de Qt). No valida diseño visual — solo que la ventana se construye,
compila el ejemplo, se conecta al simulador, corre sin explotar, y que el
resaltado es legible.

Requiere: pip install PySide6
Corre con QT_QPA_PLATFORM=offscreen si no hay display.
"""

import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from comms.robot_client import Pose
from gui.main_window import MainWindow
from gui.syntax_highlighter import (
    ALL_PALETTES,
    DARK_PALETTE,
    KEYWORDS,
    LIGHT_PALETTE,
    MOVE_KINDS,
    POINT_FUNCTIONS,
    STATES,
    TYPES,
    choose_palette,
    contrast_ratio,
)

MIN_CONTRAST = 3.0  # WCAG AA para texto grande / no-texto


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance() or QApplication([])
    yield application


@pytest.fixture
def window(app):
    return MainWindow()


def _connect_to_simulator(window) -> None:
    window.connection_panel.mode_sim.setChecked(True)
    # .click() y no _on_connect_clicked(): hace falta que se emita la señal
    # para que la ventana refresque el estado de las acciones.
    window.connection_panel.connect_btn.click()


# --- construcción -------------------------------------------------------------


def test_main_window_constructs(window):
    assert "soldar_pieza" in window.editor.toPlainText()


def test_all_docks_exist(window):
    esperados = {"estructura", "campos", "propiedades", "conexion", "mensajes", "log"}
    assert esperados <= set(window._docks)


def test_status_bar_starts_disconnected(window):
    assert "Sin conectar" in window.status_connection.text()
    assert "detenida" in window.status_vm.text()


# --- compilar -----------------------------------------------------------------


def test_compile_populates_bytecode_and_enables_run(window):
    window._on_compile()
    assert window._program is not None
    assert "MOVEJ" in window.bytecode_view.toPlainText()
    assert window.act_run.isEnabled()


def test_compile_fills_the_points_table(window):
    window._on_compile()
    assert window.points_table.rowCount() == len(window._program.points)
    assert "Puntos:" in window.status_points.text()


def test_compile_error_goes_to_the_message_window_not_the_bytecode_tab(window):
    """Antes el error se escribía DENTRO del tab de bytecode, que era un lugar
    raro para buscarlo."""
    window.editor.setPlainText("ESTO NO ES UN PROGRAMA VALIDO !!!\n")
    window._on_compile()
    assert window._program is None
    assert not window.act_run.isEnabled()
    assert window.bytecode_view.toPlainText() == ""
    errores = [m for m in window.messages.messages() if m[0] == "Error"]
    assert errores, "el error de compilación no llegó a la ventana de mensajes"
    assert errores[-1][1] == "compilador"


def test_compile_warns_about_the_undeduplicated_points_table(window):
    """El ejemplo mueve 5 veces a 2 puntos declarados. Antes había que contar
    filas para darse cuenta."""
    window._on_compile()
    advertencias = [m for m in window.messages.messages() if m[0] == "Advertencia"]
    assert any("no se deduplica" in m[2] for m in advertencias)


def test_program_without_trailing_newline_compiles_from_the_editor(window):
    """Lo que pasa escribiendo en el editor sin apretar Enter al final."""
    window.editor.setPlainText("SET_OUT(Y10, ON)")
    window._on_compile()
    assert window._program is not None


# --- ejecutar -----------------------------------------------------------------


def test_connect_to_simulator_and_run(window, app):
    _connect_to_simulator(window)
    assert window.connection_panel.is_connected()
    assert "Conectado" in window.status_connection.text()

    window._on_compile()
    assert window._program is not None

    window._on_run()

    deadline = time.monotonic() + 15
    while window.is_running() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.02)

    assert not window.is_running(), "la ejecución no terminó a tiempo"
    assert "✔" in window.log_view.toPlainText() or "✘" in window.log_view.toPlainText()
    assert "detenida" in window.status_vm.text()


def test_run_without_connection_warns_instead_of_crashing(window):
    window._on_compile()
    window._on_run()
    assert not window.is_running()
    assert any("Conectate" in m[2] for m in window.messages.messages())


# --- resaltado de sintaxis -----------------------------------------------------


def test_highlighter_is_installed_on_the_editor(window):
    assert window.highlighter.document() is window.editor.document()


def test_every_palette_color_meets_minimum_contrast():
    """El bug original: la paleta estaba pensada para fondo blanco y sobre el
    tema oscuro de Windows las keywords en azul marino quedaban ilegibles."""
    fallas = []
    for palette in ALL_PALETTES:
        for nombre, color in palette.colors().items():
            ratio = contrast_ratio(color, palette.reference_background)
            if ratio < MIN_CONTRAST:
                fallas.append(f"{palette.name}/{nombre} ({color}): {ratio:.2f}:1")
    assert not fallas, "colores por debajo del contraste mínimo: " + ", ".join(fallas)


def test_palette_is_chosen_from_the_actual_background():
    from PySide6.QtGui import QColor

    assert choose_palette(QColor("#ffffff")) is LIGHT_PALETTE
    assert choose_palette(QColor("#1e1e1e")) is DARK_PALETTE


def test_highlighted_words_exist_in_the_grammar():
    """Las listas se derivan a mano de la gramática; al menos verificamos que
    no resalten palabras que la gramática no conoce."""
    from pathlib import Path

    grammar = (
        Path(__file__).resolve().parents[1] / "compiler" / "grammar.lark"
    ).read_text(encoding="utf-8")
    for word in KEYWORDS + MOVE_KINDS + POINT_FUNCTIONS + TYPES + STATES:
        assert f'"{word}"' in grammar, f"{word} se resalta pero no está en la gramática"


# --- digitalización de puntos ---------------------------------------------------


def test_digitize_is_disabled_until_connected(window):
    assert not window.act_digitize.isEnabled()
    _connect_to_simulator(window)
    assert window.act_digitize.isEnabled()


def test_digitize_adds_a_row_with_an_editable_name(window):
    _connect_to_simulator(window)
    window._on_digitize()
    assert window.digitized_table.rowCount() == 1
    name_item = window.digitized_table.item(0, 0)
    assert name_item.text() == "p_digit_1"
    assert bool(name_item.flags() & Qt.ItemFlag.ItemIsEditable)


def test_digitized_coordinates_are_not_editable(window):
    _connect_to_simulator(window)
    window._on_digitize()
    coord_item = window.digitized_table.item(0, 1)
    assert not bool(coord_item.flags() & Qt.ItemFlag.ItemIsEditable)


def test_digitize_without_connection_warns(window):
    window._on_digitize()
    assert window.digitized_table.rowCount() == 0
    assert any("Conectate al robot" in m[2] for m in window.messages.messages())


def test_digitized_points_survive_a_recompile(window):
    """Se guardan aparte de program.points: recompilar reconstruye esa lista y
    los borraría."""
    _connect_to_simulator(window)
    window._on_digitize()
    assert window.digitized_table.rowCount() == 1
    window._on_compile()
    assert window.digitized_table.rowCount() == 1


def test_delete_digitized_removes_the_selected_row(window):
    _connect_to_simulator(window)
    window._on_digitize()
    window._on_digitize()
    window.digitized_table.setCurrentCell(0, 0)
    window._on_delete_digitized()
    assert window.digitized_table.rowCount() == 1


def test_insert_as_point_writes_declarations_into_the_editor(window):
    """Lo que cierra el circuito: sin esto la tabla no tenía salida hacia el
    programa."""
    _connect_to_simulator(window)
    window.dat_editor.setPlainText("POINT p_viejo = JOINT(0, 0, 0, 0, 0, 0)")
    window._on_digitize()
    window._on_insert_points()
    # El programa está separado: los puntos van al final del .dat, no al .src.
    texto = window.dat_editor.toPlainText()
    assert texto.splitlines()[0] == "POINT p_viejo = JOINT(0, 0, 0, 0, 0, 0)"
    assert texto.splitlines()[1].startswith("POINT p_digit_1 = WORLD(")
    assert "p_digit_1" not in window.editor.toPlainText()
    assert window.program_tabs.currentWidget() is window.dat_editor


def test_insert_as_point_in_a_single_file_program_goes_to_the_editor(window):
    from compiler import program_files as pf

    _connect_to_simulator(window)
    window.set_sources(pf.ProgramSources(pf.SourceFile("viejo.krlb", "")))
    window._on_digitize()
    window._on_insert_points()
    assert "POINT p_digit_1 = WORLD(" in window.editor.toPlainText()


def test_insert_as_point_uses_the_name_edited_by_the_user(window):
    _connect_to_simulator(window)
    window._on_digitize()
    window.digitized_table.item(0, 0).setText("p_apoyo")
    window._on_insert_points()
    assert "POINT p_apoyo = WORLD(" in window.dat_editor.toPlainText()


def test_generated_declaration_recompiles():
    """Un punto digitalizado tiene que producir una declaración que el
    compilador acepte — si no, la digitalización no sirve para nada."""
    from compiler.codegen import compile_source

    decl = MainWindow.point_declaration("p_test", Pose(1.5, -2.25, 3.0, 0.0, 0.0, 180.0))
    program = compile_source(decl + "\nMOVEJ p_test SPEED 50\n")
    assert program.point_names["p_test"] == 0


# --- exportar al pad ------------------------------------------------------------


def test_example_program_exports_to_the_pad(window, tmp_path):
    from pad.backup import PadBackup

    path = window.export_to_pad(tmp_path)
    assert path is not None and path.parent == tmp_path
    assert path.name.startswith("HCBackupRobot_") and path.suffix == ".zip"
    backup = PadBackup.read(path)
    assert [m.name for m in backup.act.modules] == ["soldar_pieza"]
    assert "CALL soldar_pieza()" in window.pad_view.toPlainText()
    assert window.tabs.currentWidget() is window.pad_view


def test_export_error_goes_to_the_messages_and_writes_nothing(window, tmp_path):
    window.editor.setPlainText("WAIT_IN(X010, 5)\n")
    assert window.export_to_pad(tmp_path) is None
    assert list(tmp_path.iterdir()) == []
    assert any("WAIT_IN" in text for _level, _source, text in window.messages.messages())


def test_pad_program_name_comes_from_the_file(window, tmp_path):
    from pathlib import Path

    assert window.pad_program_name() == "BorunteDSL"
    window._current_path = Path(tmp_path) / "Reja grande ñ.krlb"
    assert window.pad_program_name() == "Reja_grande__"
