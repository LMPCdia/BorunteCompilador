"""
Tests de los paneles de la GUI v0.2: árbol del proyecto, navegación al código,
propiedades, ventana de mensajes, campos de trabajo y la parada de la VM.

Headless igual que tests/test_gui_smoke.py.
"""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtWidgets import QApplication

from comms.robot_client import BorunteRobotClient
from comms.robot_simulator import SimulatedBorunteRobot
from compiler.codegen import compile_source
from gui.main_window import MainWindow
from gui.message_window import MessageWindow, Severity
from gui.project_tree import ROLE_LINE, ROLE_PROPERTIES, ProjectTree, find_declarations
from gui.properties_panel import PropertiesPanel
from gui.work_fields import WORK_FIELDS, WorkFieldsPanel, field_by_key
from runtime.plc_io_simulator import PlcIoSimulator
from runtime.vm import ReferenceVM

PROGRAMA = """POINT p_home = WORLD(0.0, 500.0, 300.0, 0.0, 0.0, 0.0)
POINT p_pieza = WORLD(100.0, 600.0, 200.0, 10.0, 20.0, 30.0)

VAR pieza : INT = 1
VAR contador : INT = 0

PROC apilar(altura)
MOVEJ p_home SPEED 80
ENDPROC

apilar(3)
"""


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def program():
    return compile_source(PROGRAMA)


# --- ubicación de declaraciones en el texto -----------------------------------


def test_find_declarations_locates_every_kind():
    decls = find_declarations(PROGRAMA)
    assert decls[("point", "p_home")] == 1
    assert decls[("point", "p_pieza")] == 2
    assert decls[("var", "pieza")] == 4
    assert decls[("var", "contador")] == 5
    assert decls[("proc", "apilar")] == 7


def test_find_declarations_ignores_commented_out_lines():
    decls = find_declarations("; POINT p_falso = WORLD(0,0,0,0,0,0)\nVAR a : INT = 1\n")
    assert ("point", "p_falso") not in decls
    assert decls[("var", "a")] == 2


def test_find_declarations_works_on_a_half_written_file():
    """El árbol tiene que funcionar mientras se escribe — que es cuando más se
    necesita ver la estructura. Un reparseo con Lark fallaría acá."""
    decls = find_declarations("POINT p_ok = WORLD(0,0,0,0,0,0)\nMOVEJ p_ok SPE")
    assert decls[("point", "p_ok")] == 1


# --- árbol del proyecto --------------------------------------------------------


def test_tree_shows_placeholder_before_compiling(app):
    tree = ProjectTree()
    tree.rebuild(None)
    assert tree.topLevelItemCount() == 1
    assert "sin compilar" in tree.topLevelItem(0).text(0)


def test_tree_groups_points_procs_and_variables(app, program):
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    grupos = [tree.topLevelItem(i).text(0) for i in range(tree.topLevelItemCount())]
    assert grupos == ["Puntos", "Procedimientos", "Variables"]


def test_tree_lists_declared_points(app, program):
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    puntos = tree.topLevelItem(0)
    nombres = [puntos.child(i).text(0) for i in range(puntos.childCount())]
    assert nombres == ["p_home", "p_pieza"]


def test_tree_shows_param_slots_under_their_proc(app, program):
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    procs = tree.topLevelItem(1)
    apilar = procs.child(0)
    assert "apilar" in apilar.text(0)
    assert apilar.childCount() == 1
    assert apilar.child(0).text(0) == "altura"
    assert "apilar.altura" in apilar.child(0).text(1)


def test_tree_does_not_list_param_slots_as_user_variables(app, program):
    """Los slots están en var_names pero no son variables del usuario."""
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    variables = tree.topLevelItem(2)
    nombres = [variables.child(i).text(0) for i in range(variables.childCount())]
    assert nombres == ["pieza", "contador"]
    assert not any("." in n for n in nombres)


def test_tree_reports_the_undeduplicated_point_count(app, program):
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    detalle = tree.topLevelItem(0).text(1)
    assert "2 declarado(s)" in detalle
    assert "entrada(s) en la tabla" in detalle


# --- navegación al código -------------------------------------------------------


def test_clicking_a_point_emits_its_declaration_line(app, program):
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    recibidas = []
    tree.line_requested.connect(recibidas.append)

    puntos = tree.topLevelItem(0)
    tree.itemClicked.emit(puntos.child(1), 0)  # p_pieza
    assert recibidas == [2]


def test_group_nodes_have_no_line_and_do_not_navigate(app, program):
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    recibidas = []
    tree.line_requested.connect(recibidas.append)
    tree.itemClicked.emit(tree.topLevelItem(0), 0)  # el grupo "Puntos"
    assert recibidas == []


def test_main_window_moves_the_cursor_to_the_declaration(app):
    """Programa de un solo archivo (.krlb de antes): la línea es la del editor."""
    from compiler import program_files as pf

    window = MainWindow()
    window.set_sources(pf.ProgramSources(pf.SourceFile("viejo.krlb", PROGRAMA)))
    window._on_compile()

    puntos = window.project_tree.topLevelItem(0)
    p_pieza = puntos.child(1)
    linea = p_pieza.data(0, ROLE_LINE)
    assert linea == 2

    window.project_tree.itemClicked.emit(p_pieza, 0)
    assert window.editor.textCursor().blockNumber() + 1 == 2
    assert window.tabs.currentWidget() is window.program_tabs
    assert window.program_tabs.currentWidget() is window.editor


def test_main_window_goes_to_the_point_in_the_dat(app):
    """Programa separado: el punto se declara en el .dat y el clic va ahí."""
    from compiler import program_files as pf

    window = MainWindow()
    sources = pf.split_single(PROGRAMA, "pieza.src")
    window.set_sources(sources)
    window._on_compile()
    assert window._program is not None

    p_pieza = window.project_tree.topLevelItem(0).child(1)
    window.project_tree.itemClicked.emit(p_pieza, 0)
    assert window.program_tabs.currentWidget() is window.dat_editor
    line = window.dat_editor.textCursor().block().text()
    assert line.startswith("POINT p_pieza")


# --- propiedades ------------------------------------------------------------------


def test_point_properties_keep_the_pose_axis_order(app, program):
    """El bug: guardar las propiedades como dict hacía que Qt las convirtiera a
    QVariantMap, que está ordenado por clave, y las coordenadas salían
    U, V, W, X, Y, Z."""
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    panel = PropertiesPanel()
    panel.show_item(tree.topLevelItem(0).child(1))  # p_pieza

    claves = panel.keys_in_order()
    ejes = [k for k in claves if k in ("X", "Y", "Z", "U", "V", "W")]
    assert ejes == ["X", "Y", "Z", "U", "V", "W"]


def test_point_properties_show_the_right_values(app, program):
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    panel = PropertiesPanel()
    panel.show_item(tree.topLevelItem(0).child(1))  # p_pieza
    assert panel.value_of("X") == "100"
    assert panel.value_of("U") == "10"
    assert panel.value_of("W") == "30"


def test_properties_panel_clears_with_none(app, program):
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    panel = PropertiesPanel()
    panel.show_item(tree.topLevelItem(0).child(0))
    assert panel.rowCount() > 0
    panel.show_item(None)
    assert panel.rowCount() == 0


def test_properties_are_stored_as_pairs_not_a_dict(app, program):
    """Fija la causa raíz, no solo el síntoma."""
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    props = tree.topLevelItem(0).child(0).data(0, ROLE_PROPERTIES)
    assert isinstance(props, list)
    assert all(isinstance(p, tuple) and len(p) == 2 for p in props)


def test_proc_properties_mention_the_reentrancy_limitation(app, program):
    tree = ProjectTree()
    tree.rebuild(program, PROGRAMA)
    panel = PropertiesPanel()
    slot = tree.topLevelItem(1).child(0).child(0)  # apilar -> altura
    panel.show_item(slot)
    assert "reentrante" in (panel.value_of("Nota") or "")


# --- ventana de mensajes ----------------------------------------------------------


def test_message_window_records_severity_source_and_text(app):
    mw = MessageWindow()
    mw.error("algo se rompió", "compilador")
    assert mw.messages() == [("Error", "compilador", "algo se rompió")]


def test_message_window_counts_by_severity(app):
    mw = MessageWindow()
    mw.info("a")
    mw.warning("b")
    mw.warning("c")
    mw.error("d")
    assert mw.count(Severity.INFO) == 1
    assert mw.count(Severity.WARNING) == 2
    assert mw.count(Severity.ERROR) == 1
    assert "1 errores" in mw.summary_label.text()
    assert "2 advertencias" in mw.summary_label.text()


def test_message_window_clear_resets_rows_and_counters(app):
    mw = MessageWindow()
    mw.error("x")
    mw.clear()
    assert mw.messages() == []
    assert mw.count(Severity.ERROR) == 0


def test_message_window_keeps_chronological_order(app):
    mw = MessageWindow()
    for i in range(5):
        mw.info(f"mensaje {i}")
    assert [m[2] for m in mw.messages()] == [f"mensaje {i}" for i in range(5)]


# --- campos de trabajo ------------------------------------------------------------


def test_work_fields_panel_lists_every_field(app):
    panel = WorkFieldsPanel()
    assert panel.count() == len(WORK_FIELDS)
    assert panel.current_field().key == "programacion"


def test_selecting_a_work_field_emits_its_key(app):
    panel = WorkFieldsPanel()
    recibidas = []
    panel.field_changed.connect(recibidas.append)
    panel.select("puesta_en_servicio")
    assert recibidas == ["puesta_en_servicio"]


def test_changing_work_field_hides_the_docks_it_does_not_declare(app):
    window = MainWindow()
    window.work_fields.select("puesta_en_servicio")
    campo = field_by_key("puesta_en_servicio")
    assert "estructura" not in campo.visible_docks
    assert not window._docks["estructura"].isVisibleTo(window)
    assert window._docks["mensajes"].isVisibleTo(window)


def test_going_back_to_programming_shows_the_structure_again(app):
    window = MainWindow()
    window.work_fields.select("puesta_en_servicio")
    window.work_fields.select("programacion")
    assert window._docks["estructura"].isVisibleTo(window)


def test_unknown_work_field_is_an_error(app):
    with pytest.raises(KeyError):
        field_by_key("no_existe")


# --- menú Ventana -------------------------------------------------------------------


def test_window_menu_has_a_toggle_per_dock(app):
    window = MainWindow()
    titulos = [a.text() for a in window.menu_window.actions()]
    assert len(titulos) == len(window._docks)
    assert "Estructura del proyecto" in titulos
    assert "Ventana de mensajes" in titulos


def test_dock_toggle_action_hides_and_shows_it(app):
    # Acá la ventana SÍ se muestra (offscreen): toggleViewAction refleja la
    # visibilidad real del dock, y en una ventana que nunca se mostró esa
    # visibilidad no está definida todavía.
    window = MainWindow()
    window.show()
    app.processEvents()
    try:
        dock = window._docks["propiedades"]
        action = dock.toggleViewAction()
        assert action.isChecked() and dock.isVisible()

        action.trigger()
        app.processEvents()
        assert not action.isChecked() and not dock.isVisible()

        action.trigger()
        app.processEvents()
        assert action.isChecked() and dock.isVisible()
    finally:
        window.close()


# --- parada de la VM -------------------------------------------------------------------


def _vm(program):
    sim = SimulatedBorunteRobot(move_duration_s=0.01)
    robot = BorunteRobotClient(host="fake", client=sim)
    robot.connect()
    return ReferenceVM(program, robot, PlcIoSimulator())


def test_request_stop_before_running_stops_immediately():
    program = compile_source("SET_OUT(Y10, ON)\nSET_OUT(Y11, ON)\n")
    vm = _vm(program)
    vm.request_stop()
    vm.run_from(0)
    assert vm.stopped_by_request is True


def test_stopped_program_does_not_finish_its_outputs():
    program = compile_source("SET_OUT(Y10, ON)\nSET_OUT(Y11, ON)\n")
    vm = _vm(program)
    vm.request_stop()
    vm.run_from(0)
    assert vm.plc_io.read_output(10) is False
    assert vm.plc_io.read_output(11) is False


def test_a_program_that_is_not_stopped_reports_no_stop():
    program = compile_source("SET_OUT(Y10, ON)\n")
    vm = _vm(program)
    vm.run_from(0)
    assert vm.stopped_by_request is False
    assert vm.plc_io.read_output(10) is True


def test_stop_action_is_disabled_when_nothing_is_running(app):
    window = MainWindow()
    assert not window.act_stop.isEnabled()
    window._on_compile()
    assert not window.act_stop.isEnabled()  # compilar no arranca nada


def test_stop_without_a_worker_does_nothing(app):
    """El botón está deshabilitado, pero el atajo Shift+F5 podría llegar igual."""
    window = MainWindow()
    window._on_stop()  # no debe levantar
    assert not window.is_running()
