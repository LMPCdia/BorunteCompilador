"""
Descubre los archivos de datos que el ejecutable tiene que llevar adentro.

Vive acá y no dentro del `.spec` para que lo puedan importar **los dos**: el
spec (que los declara) y `tests/test_packaging.py` (que verifica que no falte
ninguno). Si la lista viviera solo en el spec, el test tendría que parsear el
spec con expresiones regulares, y si viviera solo en el test no serviría para
construir.

Qué cuenta como archivo de datos: cualquier cosa dentro de los paquetes de la
aplicación que NO sea código Python. Hoy es solo `compiler/grammar.lark`, pero
está deliberadamente escrito como un barrido y no como una lista a mano: el
próximo archivo de datos que alguien agregue tiene que viajar en el .exe sin
que haya que acordarse de tocar el spec.
"""

from __future__ import annotations

from pathlib import Path

# Paquetes de la aplicación que pueden contener archivos de datos.
PACKAGE_DIRS = ("compiler", "comms", "runtime", "gui", "pad", "sim")

# Extensiones que son código, no datos.
CODE_SUFFIXES = frozenset({".py", ".pyc", ".pyo", ".pyd"})

IGNORED_DIR_NAMES = frozenset({"__pycache__"})


def project_root() -> Path:
    """Raíz del repo: el padre de `packaging/`."""
    return Path(__file__).resolve().parent.parent


def data_file_paths(root: Path | None = None) -> list[Path]:
    """Rutas absolutas de todos los archivos de datos de la aplicación."""
    root = root or project_root()
    found: list[Path] = []
    for package in PACKAGE_DIRS:
        directory = root / package
        if not directory.is_dir():
            continue
        for path in sorted(directory.rglob("*")):
            if not path.is_file():
                continue
            if any(part in IGNORED_DIR_NAMES for part in path.parts):
                continue
            if path.suffix.lower() in CODE_SUFFIXES:
                continue
            found.append(path)
    return found


def collect_data_files(root: Path | None = None) -> list[tuple[str, str]]:
    """En el formato `datas` de PyInstaller: (origen absoluto, carpeta destino)."""
    root = root or project_root()
    return [
        (str(path), str(path.parent.relative_to(root)).replace("\\", "/"))
        for path in data_file_paths(root)
    ]


def entry_script(root: Path | None = None) -> Path:
    return (root or project_root()) / "gui" / "app.py"


if __name__ == "__main__":  # pragma: no cover
    for source, dest in collect_data_files():
        print(f"{source}  ->  {dest}")
