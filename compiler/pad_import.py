"""
Respaldo del pad -> programa en `.src` (lógica) + `.dat` (puntos), para
seguir editándolo en el lenguaje y volver a exportarlo.

    python -m compiler.pad_import HCBackupRobot_*.zip --out carpeta [--listing Programa.html]

La meta es la ida y vuelta **exacta**: importar, compilar y comparar con el
respaldo original da el mismo programa (`compare`), salvo el numerado interno
de `insertedIndex`. Por eso el lenguaje tiene todo lo que trae un respaldo
real (opciones de movimiento, etiquetas y saltos, acciones desactivadas,
comentarios, nombres del pad: ver docs/LENGUAJE.md).

Lo que se conserva fuera del código:

- el `.fnc` (configuración del programa) viaja aparte, como
  `<nombre>.fnc` junto al `.src`: sin él, al exportar saldría el del respaldo
  de ejemplo y no el del robot;
- los valores de las herramientas y los sistemas de coordenadas NO están en
  el respaldo (viven en el pad): el programa guarda solo el número. Con el
  listado HTML del pad (`--listing`) se anotan sus nombres como comentarios.

Si algo del respaldo no se puede expresar (un campo con un valor que nunca se
vio, una acción desconocida), se dice en los avisos y en el `.src` queda una
línea `; NO IMPORTADO: ...` en ese lugar. Nada se descarta en silencio.
"""

from __future__ import annotations

import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

from compiler.ast_builder import _KEYWORDS
from compiler.program_files import ProgramSources, SourceFile
from pad import bindio, pad_html
from pad.backup import Action, PadBackup
from pad.listing import CMD_STOP, io_name

_NBSP8 = "&nbsp;" * 8
_IDENT = re.compile(r"[^A-Za-z0-9_]")
# Palabras que, escritas como identificador, chocarían con el lenguaje.
_RESERVED = set(_KEYWORDS) | {"WORLD", "JOINT", "OFFSET", "UNTIL", "THEN", "MOVE_DONE", "BASE"}


@dataclass
class Imported:
    sources: ProgramSources
    warnings: list[str] = field(default_factory=list)
    points: int = 0
    actions: int = 0


# --- nombres ---------------------------------------------------------------------------------------


def _identifier(text: str, taken: set[str], fallback: str) -> str:
    """Un identificador válido y único a partir de un nombre del pad."""
    name = _IDENT.sub("_", text).strip("_") or fallback
    if name[0].isdigit():
        name = f"{fallback}_{name}"
    if name in _RESERVED:
        name += "_"
    candidate, n = name, 2
    while candidate in taken:
        candidate = f"{name}_{n}"
        n += 1
    taken.add(candidate)
    return candidate


def _quote(text: str) -> str:
    return json.dumps(text, ensure_ascii=False)


def _as(original: str, ident: str) -> str:
    return f' AS {_quote(original)}' if original != ident else ""


def _num(text: str) -> str:
    value = float(text)
    shown = f"{value:.3f}".rstrip("0").rstrip(".")
    return "0" if shown in ("-0", "") else shown


def _seconds(text: str) -> str:
    return _num(text)


# --- el importador -----------------------------------------------------------------------------------


class _Importer:
    def __init__(self, backup: PadBackup, listing: str | None = None) -> None:
        self.backup = backup
        self.warnings: list[str] = []
        self.dat: list[str] = []
        self.points: dict[tuple, str] = {}        # (acción, nombre del pad, valores) -> identificador
        self.point_names: set[str] = set()
        self.proc_names: dict[int, tuple[str, str]] = {}   # id -> (identificador, nombre del pad)
        self.actions = 0
        self.listing_names = self._listing_names(listing)
        self._dat_unit: str | None = None

    # -- listado del pad (nombres de herramientas y sistemas) ---------------------------------------

    @staticmethod
    def _listing_names(listing: str | None) -> dict[str, str]:
        names: dict[str, str] = {}
        if not listing:
            return names
        for line in pad_html.to_text(listing).splitlines():
            m = re.search(r"Switch Coord:\[CoordID(\d+)\](.*)", line)
            if m:
                names[f"COORD {m.group(1)}"] = m.group(2).strip()
            m = re.search(r"Switch Tool:\[(\d+):(.*)\]", line)
            if m:
                names[f"TOOL {m.group(1)}"] = m.group(2).strip()
            m = re.search(r"tool:(\d+):(\S+) coord:(\d+):(\S+)", line)
            if m:
                names.setdefault(f"TOOL {m.group(1)}", m.group(2))
                if m.group(4) != "null":
                    names.setdefault(f"COORD {m.group(3)}", m.group(4))
        return names

    def warn(self, unit: str, index: int, text: str) -> None:
        self.warnings.append(f"{unit}[{index}]: {text}")

    # -- puntos -----------------------------------------------------------------------------------------

    def point(self, unit: str, unit_ident: str, action: Action, index: int) -> str:
        code = action["action"]
        points = action.get("points") or []
        pos = points[0]["pos"] if points else {}
        values = tuple(str(pos.get(f"m{i}", "0.000")) for i in range(6))
        if len(points) != 1 or points[0].get("pointName") != "" or pos.get("m6") != "0.000" \
                or pos.get("m7") != "0.000":
            self.warn(unit, index, "el movimiento trae ejes extra o más de un punto: no se importan")
        for v in values:
            if f"{float(v):.3f}" != v and v != "-0.000":
                self.warn(unit, index, f"valor {v!r}: se escribe con 3 decimales al exportar")
        original = action.get("customName") or ""
        if "customName" not in action:
            self.warn(unit, index, "el movimiento no trae customName (sin nombre): al exportar "
                                   "sale con el nombre vacío; el pad lo muestra igual")
        key = (code, original, values)
        if key not in self.points:
            ident = _identifier(original, self.point_names, f"{unit_ident}_p")
            if not original:
                ident = _identifier("", self.point_names, f"{unit_ident}_p")
            kind = "JOINT" if code == 4 else "WORLD"
            if self._dat_unit != unit:
                self.dat += ["", f"; --- {unit} ---"]
                self._dat_unit = unit
            self.dat.append(f"POINT {ident} = {kind}({', '.join(_num(v) for v in values)})"
                            + _as(original, ident))
            self.points[key] = ident
        return self.points[key]

    # -- acciones ---------------------------------------------------------------------------------------

    def unit(self, unit: str, unit_ident: str, actions: list[Action]) -> list[str]:
        labels: dict[int, tuple[str, str]] = {}        # número -> (identificador, nombre del pad)
        taken: set[str] = set()
        for a in actions:
            if a["action"] == 59999:
                text = a.get("comment", "")
                labels[int(a["flag"])] = (_identifier(text, taken, "etiqueta"), text)
        tool, coord = 0, 0
        out: list[str] = []
        for i, a in enumerate(actions):
            self.actions += 1
            code = a["action"]
            if code in (4, 10):
                out.append(self.move(unit, unit_ident, a, i, tool, coord))
            elif code == 100:
                self.expect(unit, i, a, {"customName": "", "isUnlimit": False, "point": 0,
                                         "pointStatus": 0, "type": 100})
                out.append(f"WAIT {_seconds(a['limit'])}s")
            elif code == 200:
                out.append(self.set_out(unit, i, a))
            elif code == 800:
                coord = int(a["coordID"])
                out.append(f"COORD {coord}" + self.name_note(f"COORD {coord}"))
            elif code == 801:
                tool = int(a["toolID"])
                out.append(f"TOOL {tool}" + self.name_note(f"TOOL {tool}"))
            elif code == 10001:
                out.append(self.goto_if(unit, i, a, labels))
            elif code == 20000:
                out.append(self.call(unit, i, a, labels))
            elif code == 20001 or code == 60000:
                continue
            elif code == 59999:
                ident, text = labels[int(a["flag"])]
                out.append(f"LABEL {ident} {int(a['flag'])}" + _as(text, ident))
            elif code == 50000:
                out.append(self.comment(unit, i, a, labels))
            elif code == 53000 and all(a.get(k) == v for k, v in CMD_STOP.items()):
                out.append("CMD STOP")
            else:
                fields = {k: v for k, v in a.items() if k not in ("action", "insertedIndex")}
                self.warn(unit, i, f"acción {code} desconocida: no se importa")
                out.append(f"; NO IMPORTADO: acción {code} {json.dumps(fields, ensure_ascii=False)}")
        return out

    def name_note(self, key: str) -> str:
        name = self.listing_names.get(key)
        return f"    ; {name}" if name else ""

    def expect(self, unit: str, index: int, action: Action, fixed: dict) -> None:
        for k, v in fixed.items():
            if action.get(k) != v:
                self.warn(unit, index, f"campo {k}={action.get(k)!r} (se esperaba {v!r}): "
                                       f"al exportar sale {v!r}")

    def move(self, unit, unit_ident, a: Action, i: int, tool: int, coord: int) -> str:
        code = a["action"]
        name = self.point(unit, unit_ident, a, i)
        self.expect(unit, i, a, {"ckStatus": "63", "passTrans": 0, "quotePoint": [0, 0, 0],
                                 "relativeType": 0})
        speed = a["speed"]
        if f"{float(speed):.1f}" != speed:
            self.warn(unit, i, f"velocidad {speed!r}: se escribe con 1 decimal al exportar")
        opts = []
        t, c = int(a["toolCoord"]) >> 16, int(a["toolCoord"]) & 0xFFFF
        if t != tool:
            opts.append(f"TOOL {t}")
        if c != coord:
            opts.append(f"COORD {c}")
        if int(a.get("smooth", 0)):
            opts.append(f"SMOOTH {int(a['smooth'])}")
        if a.get("delay", "0.000") != "0.000":
            opts.append(f"DELAY {_seconds(a['delay'])}")
        info = int(a.get("bindIOInfo") or 0)
        distance = a.get("distance", "0.000")
        if info:
            bind = bindio.decode(info)
            if bind is None:
                self.warn(unit, i, f"bindIOInfo {info} (0x{info:x}) desconocido: no se importa")
                opts.append(f"; NO IMPORTADO: bindIOInfo={info} distance={distance}")
            else:
                opts.append(f"{'NEAR_END' if bind.end else 'AWAY_START'} {io_name('Y', bind.point)} "
                            f"{'ON' if bind.on else 'OFF'} {_num(distance)}"
                            + ("" if bind.enabled else " DISABLED"))
        elif distance != "0.000":
            self.warn(unit, i, f"distance {distance} sin bindIOInfo: no se importa")
        text = f"{'MOVEJ' if code == 4 else 'MOVEL'} {name} SPEED {_num(speed)}"
        # Una nota `;` dentro de las opciones se corre al final de la línea.
        notes = [o for o in opts if o.startswith(";")]
        real = [o for o in opts if not o.startswith(";")]
        return " ".join([text, *real] + notes)

    def set_out(self, unit: str, index: int, a: Action) -> str:
        self.expect(unit, index, a, {"delay": "0.000", "isWaitInput": False, "type": 0})
        if a["valveID"] != a["point"]:
            self.warn(unit, index, "valveID distinto de point: al exportar se iguala")
        return f"SET_OUT({io_name('Y', a['point'])}, {'ON' if a['pointStatus'] else 'OFF'})"

    def goto_if(self, unit: str, index: int, a: Action, labels: dict) -> str:
        self.expect(unit, index, a, {"inout": 0, "limit": "0.000", "type": 0})
        status = a["pointStatus"]
        if status not in (0, 1):
            self.warn(unit, index, f"pointStatus {status!r} desconocido")
        if status == 0:
            self.warn(unit, index, "IF … == 0 GOTO (pointStatus 0): nunca visto en un respaldo "
                                   "real; al exportar necesita «instrucciones sin confirmar»")
        ident = labels.get(int(a["flag"]), (f"flag{a['flag']}", ""))[0]
        return f"IF {io_name('X', a['point'])} == {1 if status else 0} GOTO {ident}"

    def call(self, unit: str, index: int, a: Action, labels: dict) -> str:
        module = int(a["module"])
        name = self.proc_names.get(module, (f"modulo{module}", ""))[0]
        text = f"{name}()"
        if a["flag"] != -1:
            ident = labels.get(int(a["flag"]), (f"flag{a['flag']}", ""))[0]
            text += f" THEN GOTO {ident}"
            if a["flag"] != str(int(a["flag"])):
                self.warn(unit, index, f"flag {a['flag']!r}: al exportar sale como texto")
        return text

    def comment(self, unit: str, index: int, a: Action, labels: dict) -> str:
        inner = a.get("commentAction")
        if inner is None:
            return f"COMMENT {_quote(a.get('comment', ''))}"
        code = inner.get("action")
        if code == 200:
            line = "DISABLED " + self.set_out(unit, index, inner)
            want = _NBSP8 + (f"Output:{io_name('Y', inner['point'])}"
                             f"{'ON' if inner['pointStatus'] else 'OFF'} DLY0.000")
        elif code == 10001:
            line = "DISABLED " + self.goto_if(unit, index, inner, labels)
            label_text = labels.get(int(inner["flag"]), ("", ""))[1]
            want = _NBSP8 + (f"IF:{io_name('X', inner['point'])}{'ON' if inner['pointStatus'] else 'OFF'}"
                             f" Lim:0.000 Go to Flag[{inner['flag']}]:{label_text}")
        else:
            self.warn(unit, index, f"acción {code} desactivada: solo se importan SET_OUT e IF … GOTO")
            fields = {k: v for k, v in inner.items() if k != "insertedIndex"}
            return f"; NO IMPORTADO (desactivada): {json.dumps(fields, ensure_ascii=False)}"
        if a.get("comment") != want:
            self.warn(unit, index, f"el texto guardado de la acción desactivada es distinto del "
                                   f"que genera el compilador ({a.get('comment')!r}): al exportar "
                                   f"sale {want!r}")
        return line

    # -- el programa entero ------------------------------------------------------------------------------

    def run(self, source_name: str) -> Imported:
        backup = self.backup
        modules = backup.act.modules
        taken: set[str] = set()
        for m in modules:
            self.proc_names[m.id] = (_identifier(m.name, taken, "modulo"), m.name)
        if [m.id for m in modules] != list(range(len(modules))):
            self.warnings.append("los módulos no tienen los ids 0, 1, 2…: al exportar se "
                                 "renumeran por orden")
        src = [f"; Importado de {source_name} (programa {backup.name}).",
               "; Los puntos están en el .dat; el .fnc (configuración) viaja junto a este archivo.",
               "; Herramientas y sistemas: acá solo va el número; sus valores están en el pad."]
        used = sorted(self.listing_names)
        for key in used:
            src.append(f"; {key} = {self.listing_names[key]}")
        src.append("")
        for m in modules:
            ident, text = self.proc_names[m.id]
            body = self.unit(f"módulo {m.name}", ident.lower(), m.actions)
            src.append(f"PROC {ident}(){_as(text, ident)}")
            src += [f"    {line}" for line in body]
            src += ["ENDPROC", ""]
        main = self.unit("principal", "principal", backup.act.main)
        if not backup.act.main or backup.act.main[0]["action"] != 50000 \
                or "commentAction" in backup.act.main[0]:
            self.warnings.append("el principal no empieza con un comentario: al exportar se agrega "
                                 "el comentario «Generado por Borunte DSL»")
        src += main
        dat = ([f"; Puntos de {backup.name}, importados del respaldo del pad."] + self.dat)
        sources = ProgramSources(
            SourceFile(f"{backup.name}.src", "\n".join(src) + "\n"),
            SourceFile(f"{backup.name}.dat", "\n".join(dat) + "\n"),
            SourceFile("config.dat", ""),
            fnc=backup.others.get("fnc"))
        return Imported(sources, self.warnings, len(self.points), self.actions)


def import_backup(backup: PadBackup, source_name: str = "respaldo del pad",
                  listing: str | None = None) -> Imported:
    return _Importer(backup, listing).run(source_name)


# --- ida y vuelta -----------------------------------------------------------------------------------


def _strip(value):
    """Sin `insertedIndex` (el compilador lo renumera) en ningún nivel."""
    if isinstance(value, dict):
        return {k: _strip(v) for k, v in value.items() if k != "insertedIndex"}
    if isinstance(value, list):
        return [_strip(v) for v in value]
    return value


def compare(original: PadBackup, rebuilt: PadBackup) -> list[str]:
    """Diferencias entre dos respaldos, acción por acción (vacía = iguales,
    salvo `insertedIndex`)."""
    diffs: list[str] = []

    def units(b: PadBackup):
        return [("principal", b.act.main)] + [(f"módulo {m.id} {m.name}", m.actions)
                                              for m in b.act.modules]

    a_units, b_units = units(original), units(rebuilt)
    if [n for n, _ in a_units] != [n for n, _ in b_units]:
        diffs.append(f"módulos distintos: {[n for n, _ in a_units]} / {[n for n, _ in b_units]}")
    for (name, a), (_n, b) in zip(a_units, b_units):
        if len(a) != len(b):
            diffs.append(f"{name}: {len(a)} acciones y {len(b)}")
        for i, (x, y) in enumerate(zip(a, b)):
            # Un movimiento sin `customName` y uno con el nombre vacío son lo
            # mismo para el pad (muestra igual): no es una diferencia.
            if x.get("action") in (4, 10) and "customName" not in x:
                x = {**x, "customName": ""}
            if _strip(x) != _strip(y):
                changed = {k: (x.get(k), y.get(k)) for k in set(x) | set(y)
                           if k != "insertedIndex" and _strip(x.get(k)) != _strip(y.get(k))}
                diffs.append(f"{name}[{i}] acción {x.get('action')}: {json.dumps(changed, ensure_ascii=False)[:300]}")
    for i, (x, y) in enumerate(zip(original.act.lines, rebuilt.act.lines)):
        if i not in (0, 10) and x != y:
            diffs.append(f"línea {i + 1} del .act distinta")
    if original.others != rebuilt.others:
        diffs.append("archivos distintos: " + ", ".join(
            sorted(k for k in set(original.others) | set(rebuilt.others)
                   if original.others.get(k) != rebuilt.others.get(k))))
    return diffs


def rebuild(imported: Imported, name: str, **options) -> PadBackup:
    from compiler import program_files
    from compiler.pad_codegen import PadOptions

    backup, _ = program_files.compile_pad(imported.sources, PadOptions(program_name=name, **options))
    return backup


def main(argv: list[str] | None = None) -> int:
    import argparse

    from compiler import program_files

    parser = argparse.ArgumentParser(
        prog="python -m compiler.pad_import",
        description="Respaldo del pad -> .src + .dat (+ .fnc) para seguir editándolo")
    parser.add_argument("respaldo", help="HCBackupRobot_*.zip")
    parser.add_argument("--out", default=".", help="carpeta donde escribir los archivos")
    parser.add_argument("--listing", help="listado HTML del pad (anota los nombres de las "
                                          "herramientas y los sistemas)")
    parser.add_argument("--check", action="store_true",
                        help="reexporta y compara con el original")
    args = parser.parse_args(argv)

    backup = PadBackup.read(args.respaldo)
    listing = Path(args.listing).read_text(encoding="utf-8") if args.listing else None
    imported = import_backup(backup, Path(args.respaldo).name, listing)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    written = program_files.save(imported.sources, out / f"{backup.name}.src")
    print(f"{imported.actions} acciones, {imported.points} puntos distintos.")
    for path in written:
        print(f"escrito {path}")
    for w in imported.warnings:
        print("AVISO:", w)
    if args.check:
        diffs = compare(backup, rebuild(imported, backup.name))
        print("Ida y vuelta:", "idéntico" if not diffs else f"{len(diffs)} diferencias")
        for d in diffs[:30]:
            print("  ", d)
        return 0 if not diffs else 1
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
