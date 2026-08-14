"""
Test de humo de la GUI, en modo headless (sin display real, usando el
plugin "offscreen" de Qt). No valida diseño visual — solo que la ventana se
construye, compila el ejemplo, se conecta al simulador y corre sin explotar.

Requiere: pip install PySide6
Corre con QT_QPA_PLATFORM=offscreen seteado en conftest.py de este archivo.
"""

import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtWidgets import QApplication

from gui.main_window import MainWindow


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance() or QApplication([])
    yield application


def test_main_window_constructs(app):
    window = MainWindow()
    assert window is not None
    assert "soldar_pieza" in window.editor.toPlainText()


def test_compile_button_populates_bytecode(app):
    window = MainWindow()
    window._on_compile()
    assert window._program is not None
    assert "MOVEJ" in window.bytecode_view.toPlainText()
    assert window.run_btn.isEnabled()


def test_compile_error_shows_message_not_crash(app):
    window = MainWindow()
    window.editor.setPlainText("ESTO NO ES UN PROGRAMA VALIDO !!!\n")
    window._on_compile()
    assert window._program is None
    assert not window.run_btn.isEnabled()
    assert window.bytecode_view.toPlainText() != ""


def test_connect_to_simulator_and_run(app):
    window = MainWindow()
    window.connection_panel.mode_sim.setChecked(True)
    window.connection_panel._on_connect_clicked()
    assert window.connection_panel.is_connected()

    window._on_compile()
    assert window._program is not None

    window._on_run()

    deadline = time.monotonic() + 10
    while window.run_btn.isEnabled() is False and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.02)

    assert window.run_btn.isEnabled(), "la ejecución no terminó a tiempo"
    assert "✔" in window.log_view.toPlainText() or "✘" in window.log_view.toPlainText()
