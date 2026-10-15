"""
docs/LENGUAJE.md no se puede desactualizar: cada bloque de código se compila
contra el compilador de verdad.

    ```krl            tiene que compilar para el pad
    ```krl-error      tiene que FALLAR para el pad; la primera línea,
                      `; ERROR: <texto>`, es un trozo del mensaje
    ```krl-unverified compila para el pad SOLO con allow_unverified
    ```krl-vm         compila para la VM de la PC y NO para el pad
    ```krl-src / krl-dat / krl-config
                      los archivos de un mismo programa (juntos, seguidos);
                      con `; ERROR: <texto>` en alguno, tiene que fallar así
"""

import re
from pathlib import Path

import pytest

from compiler import program_files as pf
from compiler import pad_codegen
from compiler.codegen import CompileError, compile_source
from compiler.pad_codegen import PadOptions, compile_to_pad_report

GUIDE = Path(__file__).resolve().parent.parent / "docs" / "LENGUAJE.md"
FENCE = re.compile(r"^```([\w-]*)\n(.*?)^```", re.S | re.M)
FILE_TAGS = ("krl-src", "krl-dat", "krl-config")
ERROR_LINE = re.compile(r"^; ERROR: (.+)$", re.M)


def blocks() -> list[tuple[str, str, int]]:
    text = GUIDE.read_text(encoding="utf-8")
    return [(m.group(1), m.group(2), text.count("\n", 0, m.start()) + 1)
            for m in FENCE.finditer(text)]


def groups() -> list[list[tuple[str, str, int]]]:
    """Los bloques de archivos de programa, en grupos (src/dat/config seguidos)."""
    out, current = [], {}
    for tag, body, line in blocks():
        if tag in FILE_TAGS:
            if tag in current:                  # empieza otro programa
                out.append(list(current.values()))
                current = {}
            current[tag] = (tag, body, line)
        elif current:
            out.append(list(current.values()))
            current = {}
    if current:
        out.append(list(current.values()))
    return out


def _id(item) -> str:
    return f"línea {item[2]}"


def expected_error(body: str) -> str | None:
    m = ERROR_LINE.search(body)
    return m.group(1).strip() if m else None


PLAIN = [b for b in blocks() if b[0] == "krl"]
ERRORS = [b for b in blocks() if b[0] == "krl-error"]
UNVERIFIED = [b for b in blocks() if b[0] == "krl-unverified"]
VM = [b for b in blocks() if b[0] == "krl-vm"]


def test_the_guide_has_examples_of_every_kind():
    assert len(PLAIN) >= 10 and len(ERRORS) >= 20
    assert UNVERIFIED and VM and groups()


@pytest.mark.parametrize("block", PLAIN, ids=_id)
def test_krl_blocks_compile_for_the_pad(block):
    _tag, body, _line = block
    backup, _warnings = compile_to_pad_report(body, PadOptions())
    assert backup is not None


@pytest.mark.parametrize("block", ERRORS, ids=_id)
def test_krl_error_blocks_fail_with_the_stated_message(block):
    _tag, body, _line = block
    wanted = expected_error(body)
    assert wanted, "un bloque krl-error tiene que empezar con «; ERROR: <texto>»"
    with pytest.raises(CompileError) as e:
        compile_to_pad_report(body, PadOptions())
    assert wanted in str(e.value), f"{wanted!r} no está en {str(e.value)!r}"


@pytest.mark.parametrize("block", UNVERIFIED, ids=_id)
def test_unverified_blocks_need_the_unverified_switch(block):
    _tag, body, _line = block
    with pytest.raises(CompileError):
        compile_to_pad_report(body, PadOptions())
    compile_to_pad_report(body, PadOptions(allow_unverified=True))


@pytest.mark.parametrize("block", VM, ids=_id)
def test_vm_blocks_compile_for_the_vm_but_not_for_the_pad(block):
    _tag, body, _line = block
    compile_source(body)
    with pytest.raises(CompileError):
        compile_to_pad_report(body, PadOptions())


@pytest.mark.parametrize("group", groups(), ids=lambda g: f"línea {g[0][2]}")
def test_program_file_groups(group):
    files = {tag: body for tag, body, _line in group}
    sources = pf.ProgramSources(
        pf.SourceFile("paletizado.src", files.get("krl-src", "")),
        pf.SourceFile("paletizado.dat", files.get("krl-dat", "")),
        pf.SourceFile(pf.CONFIG_DAT, files.get("krl-config", "")))
    wanted = next((w for w in map(expected_error, files.values()) if w), None)
    if wanted:
        with pytest.raises(CompileError) as e:
            pf.compile_pad(sources)
        assert wanted in str(e.value)
    else:
        pf.compile_pad(sources)


def test_documented_limits_match_the_compiler():
    text = GUIDE.read_text(encoding="utf-8")
    m = re.search(r"<!-- limites: speed=([\d.]+)-([\d.]+) wait=([\d.]+)-([\d.]+) "
                  r"tool=(\d+)-(\d+) io=(\d+)-(\d+) pos=(\d+) -->", text)
    assert m, "falta el comentario de límites al final de la guía"
    speed_lo, speed_hi, wait_lo, wait_hi, tool_lo, tool_hi, io_lo, io_hi, pos = m.groups()
    assert (float(speed_lo), float(speed_hi)) == (pad_codegen.MIN_SPEED, pad_codegen.MAX_SPEED)
    assert (float(wait_lo), float(wait_hi)) == (pad_codegen.MIN_WAIT, pad_codegen.MAX_WAIT)
    assert (int(tool_lo), int(tool_hi)) == (0, pad_codegen.MAX_TOOL_OR_COORD)
    assert int(io_lo, 8) - 8 == 0 and int(io_hi, 8) - 8 == pad_codegen.MAX_IO_POINT
    assert int(pos) == pad_codegen.MAX_ABS_POSITION
    # Y los números que la guía escribe en el texto.
    assert "de 0,1 a 100" in text and "de 0,001 a 3600" in text
    assert "entre -10000 y 10000" in text and "de 0 a 15" in text
    assert "`010` a `107`" in text


def test_the_guide_does_not_mention_removed_things():
    """Cosas de una versión vieja de la guía que ya no existen."""
    text = GUIDE.read_text(encoding="utf-8")
    for old in ("act_backend", "act_reader", "compile_to_act", "Coolmay", "PROC HOME(id=1)"):
        # (el ejemplo de error con id=1 sí puede aparecer: está en un krl-error)
        if old == "PROC HOME(id=1)":
            continue
        assert old not in text, old
    assert "BASE 1" in text and "no existe" in text        # solo como ejemplo de error
