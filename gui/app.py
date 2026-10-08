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

    def verificar_cliente_plc():
        # Importa pymodbus: si el empaquetado se lo dejó afuera, se ve acá y no
        # recién cuando alguien intenta cargar el programa en el PLC.
        from comms.plc_client import CoolmayPlcClient
        from runtime.bytecode import Instruction

        words = CoolmayPlcClient.encode_instruction(Instruction("SET_VAR", a=1, b=2))
        assert len(words) == 8

    paso("codificar bytecode para el PLC (importa pymodbus)", verificar_cliente_plc)

    def ejecutar_en_el_plc_simulado():
        # El otro camino completo: cargar el bytecode en un PLC (simulado) por
        # Modbus y que lo ejecute ÉL. Es el modo en que el sistema va a funcionar
        # de verdad, así que conviene que el ejecutable lo pruebe.
        from comms.plc_client import CoolmayPlcClient, PlcVmState
        from comms.plc_simulator import SimulatedCoolmayPlc
        from compiler.codegen import compile_source

        sim = SimulatedCoolmayPlc(tick_s=0.0)
        plc = CoolmayPlcClient(host="sim", client=sim)
        plc.connect()
        programa = compile_source(
            "VAR n : INT = 0\nn = n + 2\nIF n == 2 THEN\nSET_OUT(Y10, ON)\nENDIF\n"
        )
        plc.upload_program(programa)
        plc.verify_program(programa)
        plc.start()
        assert sim.wait_until_done(10.0), "la VM del PLC no terminó"
        assert sim.state == PlcVmState.FINISHED, f"quedó en {sim.state}"
        assert sim.read_output(10) is True, "la VM del PLC no ejecutó el programa"
        sim.shutdown()

    paso("cargar y ejecutar en el PLC simulado", ejecutar_en_el_plc_simulado)

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
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
