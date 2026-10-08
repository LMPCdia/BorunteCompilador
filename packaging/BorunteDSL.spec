# -*- mode: python ; coding: utf-8 -*-
"""
Spec de PyInstaller para el ejecutable de un solo archivo.

Construir con:   pyinstaller packaging/BorunteDSL.spec --noconfirm
O mejor:         packaging/build_exe.ps1   (que además corre el self-test)

EL .EXE HAY QUE CONSTRUIRLO EN WINDOWS: PyInstaller no compila cruzado.

Los archivos de datos NO están listados a mano: los descubre
`packaging/datafiles.py`, que también usa `tests/test_packaging.py` para
verificar que no falte ninguno. El caso que motiva todo esto es
`compiler/grammar.lark`, que se lee como archivo en tiempo de ejecución y que
PyInstaller no ve por imports — sin declararlo, el ejecutable abre bien y falla
al compilar el primer programa.
"""

import sys
from pathlib import Path

# SPECPATH lo define PyInstaller: es la carpeta de este archivo.
sys.path.insert(0, SPECPATH)

from datafiles import collect_data_files, entry_script, project_root  # noqa: E402

ROOT = project_root()

# gmsh (lector de STEP) carga su librería nativa con ctypes desde la carpeta de
# su propio módulo: PyInstaller no la ve por imports. En el .exe, el módulo
# queda en la raíz de la carpeta temporal, así que la librería va ahí.
import gmsh  # noqa: E402

GMSH_BINARIES = [(gmsh.libpath, ".")]

a = Analysis(
    [str(entry_script(ROOT))],
    pathex=[str(ROOT)],
    binaries=GMSH_BINARIES,
    datas=collect_data_files(ROOT),
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # Sin excludes: excluir un modulo de Qt para bajar el tamaño es la forma
    # mas facil de romper la GUI en un solo dialogo que nadie prueba. Si algun
    # dia hace falta recortar, hay un test que verifica que no se excluya nada
    # de PySide6.
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='BorunteDSL',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    # Sin consola: es una aplicacion de escritorio.
    console=False,
    # PERO los traceback no se descartan. Con console=False y esto en True, un
    # error no manejado desaparece sin dejar rastro: ni ventana, ni log, nada.
    # Es como se pierde una tarde entera buscando por que "no hace nada".
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
