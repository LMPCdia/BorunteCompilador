"""
Programa en archivos separados, como en KUKA:

    paletizado.src    la lógica (movimientos, esperas, salidas, IF, PROC...)
    paletizado.dat    los puntos de ese programa (solo POINT)
    config.dat        puntos comunes a todos los programas de la carpeta
                      (HOME, poses de traslado...), opcional

El compilador sigue recibiendo UN texto: acá se juntan los tres (config.dat,
después el .dat, después el .src) y se guarda de qué archivo y línea vino
cada línea, para que los errores digan «paletizado.src, línea 12» y no la
línea del texto combinado.

El respaldo del pad no cambia: ahí cada movimiento lleva su punto adentro
(ver docs/PAD_FORMAT.md). La separación es solo del lado de la PC.

Reglas (errores de compilación con su archivo y línea):
- un .dat tiene solo declaraciones POINT (y comentarios / líneas vacías);
- el .src no declara puntos: van en su .dat o en config.dat;
- un nombre de punto no se repite entre config.dat y el .dat.
Un punto del .dat que el .src no usa es solo un aviso (los de config.dat no:
son para varios programas).

Un `.krlb` de antes (todo en un archivo) sigue andando igual: es un programa
sin .dat, y ahí los POINT van donde estén.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

from compiler.ast_builder import build_ast
from compiler.ast_nodes import (
    CallStmt,
    IfStmt,
    MoveStmt,
    PointDecl,
    PointName,
    PointOffset,
    ProcDecl,
    VarRef,
)
from compiler.codegen import CompileError

CONFIG_DAT = "config.dat"
SRC_SUFFIX = ".src"
DAT_SUFFIX = ".dat"
_LINE_RE = re.compile(r"Línea (\d+)")


@dataclass
class SourceFile:
    name: str            # para los mensajes: «paletizado.src»
    text: str


@dataclass
class ProgramSources:
    """Los textos de un programa. `dat` y `config` en None: programa de un
    solo archivo (.krlb de antes), sin las reglas de separación."""

    src: SourceFile
    dat: SourceFile | None = None
    config: SourceFile | None = None

    @property
    def split(self) -> bool:
        return self.dat is not None

    def files(self) -> list[SourceFile]:
        return [f for f in (self.config, self.dat, self.src) if f is not None]


@dataclass
class Combined:
    text: str
    origins: list[tuple[str, int]] = field(default_factory=list)  # línea combinada -> (archivo, línea)

    def locate(self, line: int | None) -> tuple[str, int] | None:
        if line is None or not 1 <= line <= len(self.origins):
            return None
        return self.origins[line - 1]

    def remap(self, message: str) -> str:
        """«Línea 37: …» -> «paletizado.src, línea 5: …»."""
        def sub(m: re.Match) -> str:
            where = self.locate(int(m.group(1)))
            return f"{where[0]}, línea {where[1]}" if where else m.group(0)
        return _LINE_RE.sub(sub, message)

    def remap_error(self, error: CompileError) -> CompileError:
        where = self.locate(error.line)
        out = CompileError(self.remap(str(error)), line=where[1] if where else error.line)
        out.file = where[0] if where else None
        return out


# --- archivos ---------------------------------------------------------------------------


def paths_for(src_path: str | Path) -> tuple[Path, Path, Path]:
    """(.src, .dat, config.dat) de un programa. Acepta la ruta del .src o del .dat."""
    p = Path(src_path)
    src = p.with_suffix(SRC_SUFFIX)
    return src, src.with_suffix(DAT_SUFFIX), src.parent / CONFIG_DAT


def load(path: str | Path) -> ProgramSources:
    """Lee un programa del disco. Un .src (o su .dat) trae su .dat y el
    config.dat de la carpeta (si no existen, vacíos); cualquier otra
    extensión (.krlb, .txt) es un programa de un solo archivo."""
    p = Path(path)
    if p.suffix.lower() not in (SRC_SUFFIX, DAT_SUFFIX) or p.name.lower() == CONFIG_DAT:
        return ProgramSources(SourceFile(p.name, p.read_text(encoding="utf-8")))
    src, dat, config = paths_for(p)

    def read(f: Path) -> str:
        return f.read_text(encoding="utf-8") if f.exists() else ""

    if not src.exists():
        raise FileNotFoundError(f"No existe {src.name} (el programa de {p.name})")
    return ProgramSources(SourceFile(src.name, read(src)), SourceFile(dat.name, read(dat)),
                          SourceFile(CONFIG_DAT, read(config)))


def save(sources: ProgramSources, src_path: str | Path, write_config: bool = True) -> list[Path]:
    """Escribe el .src y su .dat (y config.dat si `write_config`). Devuelve
    lo escrito. config.dat vacío y que no existía no se crea."""
    src, dat, config = paths_for(src_path)
    written = []
    src.write_text(sources.src.text, encoding="utf-8")
    written.append(src)
    dat.write_text(sources.dat.text if sources.dat else "", encoding="utf-8")
    written.append(dat)
    if write_config and sources.config is not None and (sources.config.text.strip() or config.exists()):
        config.write_text(sources.config.text, encoding="utf-8")
        written.append(config)
    return written


def split_single(text: str, src_name: str = "programa.src") -> ProgramSources:
    """Un programa de un solo archivo separado en .src (todo lo que no es
    POINT) y .dat (las líneas POINT de nivel superior, con su comentario)."""
    points, logic = [], []
    for line in text.splitlines():
        (points if re.match(r"\s*POINT\b", line) else logic).append(line)
    while logic and not logic[0].strip():
        logic.pop(0)
    stem = Path(src_name).stem
    return ProgramSources(SourceFile(f"{stem}{SRC_SUFFIX}", "\n".join(logic) + "\n"),
                          SourceFile(f"{stem}{DAT_SUFFIX}", "\n".join(points) + ("\n" if points else "")),
                          SourceFile(CONFIG_DAT, ""))


# --- juntar y revisar -----------------------------------------------------------------------


def combine(sources: ProgramSources) -> Combined:
    lines: list[str] = []
    origins: list[tuple[str, int]] = []
    for f in sources.files():
        file_lines = f.text.splitlines()
        lines += file_lines
        origins += [(f.name, i + 1) for i in range(len(file_lines))]
    return Combined("\n".join(lines) + "\n", origins)


def _parse(f: SourceFile):
    try:
        return build_ast(f.text).statements
    except CompileError as e:
        located = CompileError(_LINE_RE.sub(lambda m: f"{f.name}, línea {m.group(1)}", str(e)),
                               line=e.line)
        located.file = f.name
        raise located from None


def _error(f: SourceFile, line: int | None, message: str) -> CompileError:
    e = CompileError(f"{f.name}, línea {line}: {message}" if line else f"{f.name}: {message}",
                     line=line)
    e.file = f.name
    return e


def _used_points(statements) -> set[str]:
    used: set[str] = set()

    def expr(e) -> None:
        if isinstance(e, PointName):
            used.add(e.name)
        elif isinstance(e, PointOffset):
            expr(e.base)

    def walk(stmts) -> None:
        for s in stmts:
            if isinstance(s, MoveStmt):
                expr(s.point)
            elif isinstance(s, PointDecl):
                expr(s.expr)
            elif isinstance(s, CallStmt):     # un punto pasado a un PROC
                used.update(a.name for a in s.args if isinstance(a, VarRef))
            elif isinstance(s, ProcDecl):
                walk(s.body)
            elif isinstance(s, IfStmt):
                walk(s.then_body)
                walk(s.else_body)
    walk(statements)
    return used


def check(sources: ProgramSources) -> list[str]:
    """Reglas de la separación. Tira CompileError (con archivo y línea);
    devuelve los avisos."""
    if not sources.split:
        return []
    declared: dict[str, tuple[str, int | None]] = {}
    dat_names: list[tuple[str, int | None]] = []
    for f in (sources.config, sources.dat):
        if f is None:
            continue
        for s in _parse(f):
            line = getattr(s, "line", None)
            if not isinstance(s, PointDecl):
                raise _error(f, line, f"un .dat solo tiene puntos (POINT); "
                                      f"lo demás va en {sources.src.name}")
            if s.name in declared:
                other, other_line = declared[s.name]
                raise _error(f, line, f"el punto {s.name} ya está en {other}"
                                      f"{f', línea {other_line}' if other_line else ''}")
            declared[s.name] = (f.name, line)
            if f is sources.dat:
                dat_names.append((s.name, line))
    src_statements = _parse(sources.src)

    def no_points(stmts) -> None:
        for s in stmts:
            if isinstance(s, PointDecl):
                raise _error(sources.src, getattr(s, "line", None),
                             f"el punto {s.name} se declara en {sources.dat.name} (o en "
                             f"{CONFIG_DAT} si lo usan varios programas), no en el .src")
            if isinstance(s, ProcDecl):
                no_points(s.body)
            if isinstance(s, IfStmt):
                no_points(s.then_body)
                no_points(s.else_body)
    no_points(src_statements)

    used = _used_points(src_statements)
    # Un punto del .dat puede usarse en otro punto del .dat (p. ej. con OFFSET).
    for f in (sources.config, sources.dat):
        if f is not None:
            used |= _used_points(_parse(f))
    return [f"{sources.dat.name}, línea {line}: el punto {name} no se usa en "
            f"{sources.src.name}" for name, line in dat_names if name not in used]


# --- compilar ------------------------------------------------------------------------------


def compile_pad(sources: ProgramSources, options=None):
    """Como `compile_to_pad_report`, con los archivos juntos: (respaldo, avisos)
    con archivo y línea en errores y avisos."""
    from compiler.pad_codegen import compile_to_pad_report

    warnings = check(sources)
    combined = combine(sources)
    try:
        backup, more = compile_to_pad_report(combined.text, options)
    except CompileError as e:
        raise combined.remap_error(e) from None
    return backup, warnings + [combined.remap(w) for w in more]


def compile_vm(sources: ProgramSources):
    """Como `compile_source` (VM de la PC): (programa, avisos)."""
    from compiler.codegen import compile_source

    warnings = check(sources)
    combined = combine(sources)
    try:
        program = compile_source(combined.text)
    except CompileError as e:
        raise combined.remap_error(e) from None
    return program, warnings
