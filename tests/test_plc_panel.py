"""
Tests del panel de la VM del PLC (gui/plc_panel.py) y del worker de poll.

Headless igual que el resto de los tests de GUI. Acá hay hilos de verdad
corriendo, así que la fixture se asegura de bajarlos: un QThread que sobrevive al
test se lleva puesto al proceso entero cuando Python recolecta el objeto, y el
síntoma es un pytest que corta sin imprimir el resumen.
"""

import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtWidgets import QApplication

from comms.plc_client import CoolmayPlcClient, PlcVmState
from comms.plc_simulator import SimulatedCoolmayPlc
from compiler.codegen import compile_source
from gui.main_window import MainWindow
from gui.plc_panel import VARIABLES_MOSTRADAS, PlcPanel
from gui.plc_status_worker import PlcStatusWorker

PROGRAMA = """VAR contador : INT = 0
VAR objetivo : INT = 3
PROC sumar(cuanto)
contador = contador + 1
ENDPROC
sumar(1)
sumar(1)
sumar(1)
IF contador == objetivo THEN
SET_OUT(Y10, ON)
ENDIF
"""


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def panel(app):
    """Panel con desconexión garantizada.

    No alcanza con shutdown(): hay que soltar el cliente y el simulador para que
    no queden vivos hasta que termine el proceso. Con decenas de tests
    acumulándolos, el intérprete los destruye al salir en un orden que Qt no
    tolera y el proceso crashea DESPUÉS de que pytest imprimió el resumen — o
    sea, todo verde y exit code distinto de 0.
    """
    p = PlcPanel()
    yield p
    p.shutdown()
    if p.is_connected():
        p._on_disconnect()


@pytest.fixture
def window(app):
    w = MainWindow()
    yield w
    w.plc_panel.shutdown()
    if w.plc_panel.is_connected():
        w.plc_panel._on_disconnect()
    w.close()


def _wait(app, condition, timeout_s: float = 10.0) -> bool:
    """Bombea el bucle de eventos hasta que se cumpla la condición."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app.processEvents()
        if condition():
            return True
        time.sleep(0.01)
    return False


def _connect(panel) -> None:
    panel.mode_sim.setChecked(True)
    panel.connect_btn.click()


# --- construcción y estado inicial ---------------------------------------------


def test_panel_starts_disconnected(panel):
    assert not panel.is_connected()
    assert "Sin conectar" in panel.connection_label.text()


def test_everything_is_disabled_before_connecting(panel):
    assert not panel.upload_btn.isEnabled()
    assert not panel.start_btn.isEnabled()
    assert not panel.stop_btn.isEnabled()
    assert not panel.disconnect_btn.isEnabled()


def test_set_program_fills_the_capacity_bars(panel):
    program = compile_source(PROGRAMA)
    panel.set_program(program)
    assert panel.capacity_instr.value() == len(program.instructions)
    assert panel.capacity_points.value() == len(program.points)


def test_set_program_none_clears_the_bars(panel):
    panel.set_program(compile_source(PROGRAMA))
    panel.set_program(None)
    assert panel.capacity_instr.value() == 0
    assert panel.program is None


# --- conexión --------------------------------------------------------------------


def test_connect_to_the_simulator(panel, app):
    _connect(panel)
    assert panel.is_connected()
    assert "simulador" in panel.connection_label.text()
    assert panel.disconnect_btn.isEnabled()
    assert not panel.connect_btn.isEnabled()


def test_connecting_starts_the_status_poll(panel, app):
    _connect(panel)
    assert _wait(app, lambda: panel._last_status is not None), "el poll no reportó nada"
    assert panel.pc_label.text() == "0"
    assert "Parada" in panel.state_label.text()


def test_connect_emits_an_info_message(panel, app):
    recibidos = []
    panel.message.connect(lambda sev, texto, origen: recibidos.append((sev, texto)))
    _connect(panel)
    assert any(sev == "info" and "Conectado" in t for sev, t in recibidos)


def test_real_mode_warns_that_it_was_never_tested(panel, app):
    """El panel no puede dejar creer que hablar con un CX3G real está probado."""
    recibidos = []
    panel.message.connect(lambda sev, texto, origen: recibidos.append((sev, texto)))
    panel.mode_real.setChecked(True)
    panel.ip_edit.setText("127.0.0.1")
    panel.connect_btn.click()
    # Conecte o no (no hay nada escuchando), lo que importa es que avise.
    if panel.is_connected():
        assert any(sev == "warning" and "ladder" in t for sev, t in recibidos)
    else:
        assert any(sev == "error" for sev, _ in recibidos)


def test_disconnect_cleans_up(panel, app):
    _connect(panel)
    assert _wait(app, lambda: panel._last_status is not None)
    panel.disconnect_btn.click()
    assert not panel.is_connected()
    assert panel._thread is None
    assert panel.pc_label.text() == "—"
    assert panel.variables_table.rowCount() == 0


def test_mode_cannot_be_changed_while_connected(panel, app):
    _connect(panel)
    assert not panel.mode_sim.isEnabled()
    assert not panel.mode_real.isEnabled()


# --- carga del programa ------------------------------------------------------------


def test_upload_needs_a_program(panel, app):
    _connect(panel)
    assert not panel.upload_btn.isEnabled()  # sin programa compilado


def test_upload_puts_the_program_in_the_plc(panel, app):
    panel.set_program(compile_source(PROGRAMA))
    _connect(panel)
    assert panel.upload_btn.isEnabled()
    panel.upload_btn.click()
    assert _wait(
        app, lambda: panel._last_status and panel._last_status.instruction_count > 0
    ), "el PLC no reportó instrucciones cargadas"
    assert panel._last_status.instruction_count == len(panel.program.instructions)


def test_upload_verifies_and_reports(panel, app):
    panel.set_program(compile_source(PROGRAMA))
    _connect(panel)
    recibidos = []
    panel.message.connect(lambda sev, texto, origen: recibidos.append((sev, texto)))
    panel.upload_btn.click()
    assert any("Cargadas y verificadas" in t for _, t in recibidos)


def test_verify_after_upload_passes(panel, app):
    panel.set_program(compile_source(PROGRAMA))
    _connect(panel)
    panel.upload_btn.click()
    recibidos = []
    panel.message.connect(lambda sev, texto, origen: recibidos.append((sev, texto)))
    panel.verify_btn.click()
    assert any(sev == "info" and "coincide" in t for sev, t in recibidos)


def test_verify_detects_a_corrupted_plc(panel, app):
    panel.set_program(compile_source(PROGRAMA))
    _connect(panel)
    panel.upload_btn.click()
    panel._simulator.write_register(1001, 0x4242)  # ensucia el operando A

    recibidos = []
    panel.message.connect(lambda sev, texto, origen: recibidos.append((sev, texto)))
    panel.verify_btn.click()
    assert any(sev == "error" and "verificación falló" in t for sev, t in recibidos)


def test_start_is_disabled_until_something_is_uploaded(panel, app):
    panel.set_program(compile_source(PROGRAMA))
    _connect(panel)
    assert not panel.start_btn.isEnabled()
    panel.upload_btn.click()
    assert panel.start_btn.isEnabled()


# --- ejecución en el PLC -------------------------------------------------------------


def test_start_runs_the_program_and_the_poll_sees_it_finish(panel, app):
    panel.set_program(compile_source(PROGRAMA))
    _connect(panel)
    panel.upload_btn.click()
    panel.start_btn.click()

    assert _wait(
        app,
        lambda: panel._last_status and panel._last_status.state == PlcVmState.FINISHED,
    ), "el panel nunca vio el programa terminado"
    assert panel._simulator.read_output(10) is True


def test_the_program_counter_advances_in_the_panel(panel, app):
    panel.set_program(compile_source(PROGRAMA))
    _connect(panel)
    panel.upload_btn.click()
    # Esperar a que el poll reporte al menos una vez: antes de eso el label
    # todavía dice "—", no "0".
    assert _wait(app, lambda: panel.pc_label.text() == "0"), "el poll no reportó PC=0"
    panel.start_btn.click()
    assert _wait(app, lambda: panel.pc_label.text() not in ("0", "—"))


def test_variables_show_their_names_from_the_compiled_program(panel, app):
    """El PLC solo tiene números; los nombres los sabe el compilador."""
    program = compile_source(PROGRAMA)
    panel.set_program(program)
    _connect(panel)
    panel.upload_btn.click()
    panel.start_btn.click()
    assert _wait(
        app,
        lambda: panel._last_status and panel._last_status.state == PlcVmState.FINISHED,
    )

    fila = program.var_names["contador"]
    assert panel.variables_table.item(fila, 1).text() == "contador"
    assert panel.variables_table.item(fila, 2).text() == "3"


def test_variables_table_shows_the_register_number(panel, app):
    _connect(panel)
    assert _wait(app, lambda: panel.variables_table.rowCount() > 0)
    assert panel.variables_table.item(0, 0).text() == "D10"
    assert panel.variables_table.rowCount() == VARIABLES_MOSTRADAS


def test_stop_stops_a_looping_program(panel, app):
    from runtime.bytecode import Instruction, Program

    panel.set_program(Program(instructions=[Instruction("JUMP", b=0), Instruction("END")]))
    _connect(panel)
    panel.upload_btn.click()
    panel.start_btn.click()
    assert _wait(
        app,
        lambda: panel._last_status and panel._last_status.state == PlcVmState.RUNNING,
    ), "nunca se vio corriendo"
    assert panel.stop_btn.isEnabled()

    panel.stop_btn.click()
    assert _wait(
        app,
        lambda: panel._last_status and panel._last_status.state == PlcVmState.STOPPED,
    ), "el STOP no llegó"


def test_upload_is_blocked_while_running(panel, app):
    from runtime.bytecode import Instruction, Program

    panel.set_program(Program(instructions=[Instruction("JUMP", b=0), Instruction("END")]))
    _connect(panel)
    panel.upload_btn.click()
    panel.start_btn.click()
    assert _wait(
        app,
        lambda: panel._last_status and panel._last_status.state == PlcVmState.RUNNING,
    )
    # Cargar con la VM corriendo reescribiría el bytecode bajo los pies del PC.
    assert not panel.upload_btn.isEnabled()
    panel.stop_btn.click()


def test_error_state_is_shown_with_its_name(panel, app):
    from runtime.bytecode import Instruction, Program

    panel.set_program(Program(instructions=[Instruction("RET"), Instruction("END")]))
    _connect(panel)
    panel.upload_btn.click()
    panel.start_btn.click()
    assert _wait(
        app,
        lambda: panel._last_status and panel._last_status.state == PlcVmState.ERROR,
    ), "no se vio el estado de error"
    assert "RET_WITHOUT_CALL" in panel.error_label.text()


# --- worker de poll ------------------------------------------------------------------


def test_worker_pause_leaves_it_idle(app):
    sim = SimulatedCoolmayPlc(tick_s=0.0)
    client = CoolmayPlcClient(host="sim", client=sim)
    client.connect()
    worker = PlcStatusWorker(client, interval_ms=10)

    import threading

    hilo = threading.Thread(target=worker.run, daemon=True)
    hilo.start()
    try:
        assert _wait(app, worker.is_running, 2.0)
        worker.set_paused(True)
        # is_idle() es la condición que el panel espera antes de tocar el socket.
        deadline = time.monotonic() + 2
        while not worker.is_idle() and time.monotonic() < deadline:
            time.sleep(0.005)
        assert worker.is_idle()
    finally:
        worker.request_stop()
        hilo.join(2)
    assert not hilo.is_alive()


def test_worker_gives_up_after_repeated_failures(app):
    """Tres fallos seguidos y corta, en vez de seguir golpeando la red."""

    class PlcRoto:
        def connect(self):
            return True

        def close(self):
            pass

        def read_holding_registers(self, *a, **kw):
            raise OSError("sin ruta al host")

    client = CoolmayPlcClient(host="roto", client=PlcRoto())
    worker = PlcStatusWorker(client, interval_ms=1)
    fallos = []
    worker.failed.connect(fallos.append)

    worker.run()  # corre en este hilo: corta solo
    assert not worker.is_running()
    assert len(fallos) == 1
    assert "comunicación" in fallos[0]


# --- integración con la ventana ----------------------------------------------------------


def test_the_window_has_a_plc_dock(window):
    assert "plc" in window._docks
    assert window._docks["plc"].windowTitle() == "VM del PLC"


def test_compiling_hands_the_program_to_the_plc_panel(window):
    window._on_compile()
    assert window.plc_panel.program is not None
    assert window.plc_panel.capacity_instr.value() == len(window._program.instructions)


def test_a_compile_error_clears_the_program_in_the_plc_panel(window):
    window._on_compile()
    assert window.plc_panel.program is not None
    window.editor.setPlainText("ESTO NO COMPILA !!!\n")
    window._on_compile()
    assert window.plc_panel.program is None


def test_plc_messages_reach_the_message_window(window, app):
    _connect(window.plc_panel)
    assert any(
        origen == "plc" for _sev, origen, _texto in window.messages.messages()
    ), "los mensajes del panel del PLC no llegaron a la ventana de mensajes"


def test_plc_dock_is_in_the_window_menu(window):
    titulos = [a.text() for a in window.menu_window.actions()]
    assert "VM del PLC" in titulos


def test_closing_the_window_shuts_down_the_poll_thread(app):
    """Si la ventana se destruye con el QThread vivo, el proceso crashea al
    salir."""
    w = MainWindow()
    _connect(w.plc_panel)
    assert _wait(app, lambda: w.plc_panel._thread is not None, 2.0)
    w.close()
    assert w.plc_panel._thread is None
