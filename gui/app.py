"""
Punto de entrada: `python -m gui.app`

Con `--self-test` no abre ventana: construye todo, compila el programa de
ejemplo, se conecta al simulador, corre un programa chico y devuelve un código
de salida. Existe por una razón concreta del empaquetado.

`compiler/grammar.lark` **se lee como archivo en tiempo de ejecución**.
PyInstaller solo empaqueta lo que ve por imports, así que si no está declarado
en `datas`, el ejecutable abre bien y **falla al compilar el primer programa** —
y con `console=False` ese error no se ve en ninguna parte. Es el error más fácil
de no notar al empaquetar, y el único modo de detectarlo es ejercitar la
compilación sobre el binario ya construido:

    python -m gui.app --self-test      # en desarrollo
    BorunteDSL.exe --self-test         # sobre el ejecutable

`packaging/build_exe.ps1` lo usa como criterio de aceptación antes de dar el
.exe por bueno.
"""

from __future__ import annotations

import os
import sys
import traceback


def _emit(line: str = "") -> None:
    """Imprime sin asumir que hay stdout.

    En un ejecutable construido con `console=False`, PyInstaller deja
    `sys.stdout` en `None` cuando no hay un handle válido, y un `print()` pelado
    revienta con AttributeError — justo en el código cuyo trabajo es reportar si
    algo anda mal. El código de salida es el contrato de verdad del self-test;
    esta salida es de cortesía para cuando sí hay dónde escribirla.
    """
    try:
        stream = sys.stdout
        if stream is not None:
            stream.write(line + "\n")
            stream.flush()
    except Exception:  # noqa: BLE001
        pass


def _self_test() -> int:
    """Devuelve 0 si todo pasa, 1 si algo falla. No abre ventana."""
    # Sin display: el self-test tiene que poder correr en un build server o
    # desde un doble clic sin que aparezca nada.
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    pasos: list[tuple[str, bool, str]] = []

    def paso(nombre: str, fn) -> object:
        try:
            resultado = fn()
        except Exception as e:  # noqa: BLE001 — el self-test reporta, no propaga
            pasos.append((nombre, False, f"{type(e).__name__}: {e}"))
            if os.environ.get("BORUNTE_SELFTEST_TRACEBACK"):
                traceback.print_exc()
            return None
        pasos.append((nombre, True, ""))
        return resultado

    _emit(f"Borunte DSL — self-test (congelado: {getattr(sys, 'frozen', False)})")

    def cargar_gramatica():
        # El paso que importa: la gramática es un archivo de datos, no un módulo.
        from compiler.ast_builder import GRAMMAR_PATH

        texto = GRAMMAR_PATH.read_text(encoding="utf-8")
        assert "start:" in texto, f"{GRAMMAR_PATH} no parece la gramática"
        return GRAMMAR_PATH

    grammar_path = paso("leer compiler/grammar.lark", cargar_gramatica)
    if grammar_path is not None:
        _emit(f"  gramática en: {grammar_path}")

    def compilar_ejemplo():
        from compiler.codegen import compile_source
        from gui.main_window import EXAMPLE_PROGRAM

        program = compile_source(EXAMPLE_PROGRAM)
        assert program.instructions, "el programa de ejemplo compiló vacío"
        return program

    program = paso("compilar el programa de ejemplo", compilar_ejemplo)

    def construir_ventana():
        from PySide6.QtWidgets import QApplication

        from gui.main_window import MainWindow

        app = QApplication.instance() or QApplication([])
        window = MainWindow()
        window._on_compile()
        assert window._program is not None, "la ventana no pudo compilar el ejemplo"
        return app, window

    construido = paso("construir la ventana y compilar desde la GUI", construir_ventana)

    def conectar_simulador():
        assert construido is not None
        _, window = construido
        window.connection_panel.mode_sim.setChecked(True)
        window.connection_panel.connect_btn.click()
        assert window.connection_panel.is_connected(), "no se conectó al simulador"

    paso("conectar al simulador", conectar_simulador)

    def ejecutar_programa_chico():
        from comms.robot_client import BorunteRobotClient
        from comms.robot_simulator import SimulatedBorunteRobot
        from compiler.codegen import compile_source
        from runtime.plc_io_simulator import PlcIoSimulator
        from runtime.vm import ReferenceVM

        chico = compile_source(
            "VAR n : INT = 1\nIF n == 1 THEN\nSET_OUT(Y10, ON)\nENDIF\n"
        )
        robot = BorunteRobotClient(host="sim", client=SimulatedBorunteRobot(0.01))
        robot.connect()
        plc_io = PlcIoSimulator()
        ReferenceVM(chico, robot, plc_io).run_from(0)
        assert plc_io.read_output(10) is True, "la VM no ejecutó el programa"

    paso("ejecutar un programa en la VM de referencia", ejecutar_programa_chico)

    def exportar_al_pad():
        # Lee pad/template.fnc, que es un archivo de datos como la gramática:
        # si el empaquetado se lo dejó afuera, se ve acá y no recién cuando
        # alguien exporta su primer programa.
        from compiler.pad_codegen import compile_to_pad
        from pad.backup import PadBackup

        backup = compile_to_pad(
            "POINT casa = JOINT(0, 45, -45, 0, -75, 0)\nMOVEJ casa SPEED 10\nSET_OUT(Y010, ON)\n"
        )
        again = PadBackup.from_bytes(backup.to_bytes())
        assert again.act.main[-1]["action"] == 60000, "el respaldo no termina en END"
        assert again.others["fnc"], "falta pad/template.fnc"

    paso("generar un respaldo para el pad", exportar_al_pad)

    def simular_y_leer_step():
        # gmsh (lector de STEP) trae una DLL nativa que PyInstaller no ve por
        # imports: si el spec no la incluye, se ve acá.
        import tempfile
        from pathlib import Path

        import gmsh

        from compiler.pad_codegen import compile_to_pad
        from sim.kinematics import RobotModel
        from sim.meshes import load_step
        from sim.pad_sim import simulate

        result = simulate(
            # El primer MOVEJ solo ubica al robot; el segundo es el que tarda.
            compile_to_pad("MOVEJ JOINT(0, 45, -45, 0, -75, 0) SPEED 50\n"
                           "MOVEJ JOINT(30, 45, -45, 0, -75, 0) SPEED 50\n"),
            RobotModel.load("BRTIRUS1820A"),
        )
        assert result.ok and result.total_time_s > 0, "la simulación no corrió"

        # El robot de la celda, con las mallas del fabricante (sim/models/*/ *.stl).
        from sim.scene import has_real_meshes, robot_link_meshes

        celda = RobotModel.load("BRTIRUS1510A")
        assert has_real_meshes(celda), "faltan las mallas del BRTIRUS1510A"
        assert all(len(m) for m in robot_link_meshes(celda)), "malla vacía en el 1510A"

        with tempfile.TemporaryDirectory() as tmp:
            step = Path(tmp) / "caja.step"
            gmsh.initialize(interruptible=False)
            try:
                gmsh.option.setNumber("General.Terminal", 0)
                gmsh.model.occ.addBox(0, 0, 0, 100, 50, 20)
                gmsh.model.occ.synchronize()
                gmsh.write(str(step))
            finally:
                gmsh.finalize()
            assert len(load_step(step)) > 0, "el STEP no tiene triángulos"

    paso("simular, cargar el BRTIRUS1510A e importar un STEP (gmsh)", simular_y_leer_step)

    def buscar_choques():
        # python-fcl es una extensión compilada con sus propias DLL: si el spec
        # no las incluye, el import falla acá y no en la mano del usuario.
        from sim import collision
        from sim.kinematics import RobotModel, identity
        from sim.meshes import box
        from sim.scene import robot_link_meshes

        assert collision.available(), "no se pudo importar python-fcl"
        model = RobotModel.load("BRTIRUS1820A")
        # Una caja donde está la muñeca en HOME: tiene que chocar.
        caja = collision.Obstacle("caja", box((470, 0, 1117), (200, 200, 200)), identity())
        checker = collision.CollisionChecker(model, robot_link_meshes(model, tool_axis=False),
                                             [caja], margin_mm=0)
        distancias = checker.distances_at([0, 45.9, -44.9, 0, -76, 0])
        assert any(o == "caja" and d <= 0 for _p, o, d in distancias), "fcl no detecta el choque"

    paso("buscar choques (python-fcl)", buscar_choques)

    _emit()
    fallas = 0
    for nombre, ok, detalle in pasos:
        marca = "OK  " if ok else "FALLA"
        _emit(f"  {marca} {nombre}" + (f" — {detalle}" if detalle else ""))
        if not ok:
            fallas += 1

    _emit()
    if fallas:
        _emit(f"SELF-TEST FALLIDO: {fallas} de {len(pasos)} pasos fallaron.")
        return 1
    _emit(f"SELF-TEST OK: {len(pasos)} pasos.")
    if program is not None:
        _emit(f"  ({len(program.instructions)} instrucciones en el ejemplo)")
    return 0


def main() -> None:
    if "--self-test" in sys.argv:
        sys.exit(_self_test())

    from PySide6.QtWidgets import QApplication

    from gui.main_window import MainWindow

    app = QApplication(sys.argv)
    install_error_dialog()
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


def install_error_dialog() -> None:
    """Cualquier excepción que se escape de un botón se muestra en una ventana.

    En el .exe sin consola, una excepción no capturada en un slot de Qt no
    deja rastro: el usuario ve que el botón "no hace nada". Con esto, por lo
    menos ve qué pasó y lo puede reportar.
    """
    import traceback

    from PySide6.QtWidgets import QMessageBox

    def hook(exc_type, exc, tb):
        detail = "".join(traceback.format_exception(exc_type, exc, tb))
        _emit(detail)
        box = QMessageBox(QMessageBox.Icon.Critical, "Borunte DSL — error inesperado",
                          f"{exc_type.__name__}: {exc}\n\nLa aplicación sigue abierta, pero "
                          f"conviene guardar el trabajo.")
        box.setDetailedText(detail)
        box.exec()

    sys.excepthook = hook


if __name__ == "__main__":
    main()
