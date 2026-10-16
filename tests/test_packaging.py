"""
Invariantes del empaquetado.

No construyen el .exe (eso tarda minutos y necesita PyInstaller instalado):
verifican las cosas que, si se rompen, producen un ejecutable que ABRE BIEN y
falla después — que son las que cuestan horas de encontrar.

Ninguno de estos tests hardcodea `grammar.lark`: si mañana aparece otro archivo
de datos, el test lo descubre solo y falla si no viaja en el .exe.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGING = ROOT / "packaging"
SPEC = PACKAGING / "BorunteDSL.spec"

sys.path.insert(0, str(PACKAGING))

from datafiles import (  # noqa: E402
    CODE_SUFFIXES,
    PACKAGE_DIRS,
    collect_data_files,
    data_file_paths,
    entry_script,
    project_root,
)


@pytest.fixture(scope="module")
def spec_text() -> str:
    return SPEC.read_text(encoding="utf-8")


# --- archivos de datos ----------------------------------------------------------


def test_the_spec_exists():
    assert SPEC.is_file(), f"falta {SPEC}"


def test_project_root_is_the_repo():
    assert project_root() == ROOT
    assert (project_root() / "compiler" / "grammar.lark").is_file()


def test_entry_script_exists():
    assert entry_script().is_file()


def test_data_files_are_discovered_not_hardcoded():
    """El barrido tiene que encontrar al menos la gramática. Si algún día se
    agrega otro archivo de datos, aparece acá sin tocar nada."""
    encontrados = data_file_paths()
    assert encontrados, "el barrido no encontró ningún archivo de datos"
    nombres = {p.name for p in encontrados}
    assert "grammar.lark" in nombres


def test_every_non_python_file_in_the_packages_is_declared():
    """La invariante de fondo: nada que la app lea en runtime puede quedar
    afuera del ejecutable."""
    declarados = {Path(source).resolve() for source, _dest in collect_data_files()}
    faltantes = []
    for package in PACKAGE_DIRS:
        directory = ROOT / package
        if not directory.is_dir():
            continue
        for path in directory.rglob("*"):
            if not path.is_file():
                continue
            if "__pycache__" in path.parts:
                continue
            if path.suffix.lower() in CODE_SUFFIXES:
                continue
            if path.resolve() not in declarados:
                faltantes.append(str(path.relative_to(ROOT)))
    assert not faltantes, f"archivos de datos sin declarar: {faltantes}"


def test_data_files_keep_their_package_folder():
    """`compiler/grammar.lark` tiene que caer en `compiler/` dentro del bundle:
    ast_builder.py lo busca con Path(__file__).parent / 'grammar.lark'."""
    for source, dest in collect_data_files():
        if Path(source).name == "grammar.lark":
            assert dest == "compiler"
            break
    else:
        pytest.fail("grammar.lark no está entre los archivos de datos")


def test_spec_uses_the_discovery_helper_instead_of_a_literal_list(spec_text):
    assert "collect_data_files" in spec_text
    assert "datas=collect_data_files" in spec_text.replace(" ", "")


def test_the_grammar_is_read_as_a_file_at_runtime():
    """Justifica todo lo anterior: si esto dejara de ser cierto (por ejemplo si
    la gramática pasara a estar embebida como string), el resto de este archivo
    se puede borrar."""
    source = (ROOT / "compiler" / "ast_builder.py").read_text(encoding="utf-8")
    assert "GRAMMAR_PATH" in source
    assert "read_text" in source


# --- opciones del ejecutable ------------------------------------------------------


def test_no_qt_module_is_excluded(spec_text):
    """Excluir un módulo de Qt para bajar el tamaño rompe la GUI en un solo
    diálogo que nadie prueba."""
    inicio = spec_text.index("excludes=")
    fragmento = spec_text[inicio:inicio + 200]
    assert "PySide6" not in fragmento, f"hay un módulo de Qt excluido: {fragmento!r}"
    assert "Qt" not in fragmento.split(",")[0]


def test_windowed_traceback_is_not_disabled(spec_text):
    """Con console=False y disable_windowed_traceback=True, un error no manejado
    desaparece sin dejar rastro: ni ventana, ni log, nada."""
    assert "disable_windowed_traceback=False" in spec_text.replace(" ", "")


def test_it_builds_a_windowed_app(spec_text):
    assert "console=False" in spec_text.replace(" ", "")


def test_the_exe_is_named_as_expected(spec_text):
    assert "name='BorunteDSL'" in spec_text.replace(" ", "").replace('"', "'")


# --- dependencias ------------------------------------------------------------------


def test_pyinstaller_is_not_in_the_runtime_requirements():
    """No se necesita ni para desarrollar ni para correr los tests: meterlo en
    requirements.txt obligaría a bajarlo a todos los que clonan el repo."""
    requirements = (ROOT / "requirements.txt").read_text(encoding="utf-8").lower()
    assert "pyinstaller" not in requirements


def test_pyinstaller_is_in_the_build_requirements():
    build_reqs = (PACKAGING / "requirements-build.txt").read_text(encoding="utf-8").lower()
    assert "pyinstaller" in build_reqs


def test_runtime_requirements_still_list_the_real_dependencies():
    requirements = (ROOT / "requirements.txt").read_text(encoding="utf-8").lower()
    for package in ("lark", "pymodbus", "pyside6", "pytest", "gmsh", "python-fcl"):
        assert package in requirements, f"falta {package} en requirements.txt"


# --- self-test ---------------------------------------------------------------------


def test_the_build_script_uses_the_self_test_as_acceptance_criterion():
    script = (PACKAGING / "build_exe.ps1").read_text(encoding="utf-8")
    assert "--self-test" in script
    # Start-Process -Wait y no "& $exe": un ejecutable sin consola no bloquea a
    # PowerShell, y el criterio de aceptación daría verde siempre.
    assert "Start-Process" in script
    assert "-Wait" in script
    assert "ExitCode" in script


def test_self_test_does_not_use_bare_print():
    """En modo windowed PyInstaller deja sys.stdout en None y un print() pelado
    revienta justo en el código que tiene que reportar errores."""
    source = (ROOT / "gui" / "app.py").read_text(encoding="utf-8")
    cuerpo = source[source.index("def _self_test()"):]
    assert "print(" not in cuerpo.replace("traceback.print_exc()", ""), (
        "el self-test usa print() en vez de _emit()"
    )


def test_self_test_passes_in_development():
    """El mismo criterio que corre el script de build, pero sin empaquetar."""
    from gui.app import _self_test

    assert _self_test() == 0


def test_the_spec_bundles_the_fcl_dlls(spec_text):
    """python-fcl trae ccd.dll y octomap.dll al lado de la extensión: sin
    ellas el .exe abre, pero no busca choques (el self-test lo detecta)."""
    assert 'collect_dynamic_libs("fcl")' in spec_text
    assert "FCL_BINARIES" in spec_text.split("binaries=")[1].split("\n")[0]
