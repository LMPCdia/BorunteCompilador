"""
El listado del programa que el propio pad exporta como HTML (el texto que se
ve en la pantalla del pad), y la comprobación de que coincide con el respaldo.

    python -m pad.pad_html HCBackupRobot_*.zip Programa.html

Sirve para confirmar qué significa cada campo del `.act`: el pad lo muestra
con sus propias palabras (`Free Path` = MOVEJ, `Line3D-Pose` = MOVEL, `DLY` =
espera, `Smooth`, `Cmd Set:Stop`, ...). Cada entrada del listado corresponde a
UNA acción del respaldo, en el mismo orden (visto en un programa de 1032
acciones: coinciden las 1032).

Nada de esto prueba que el pad ACEPTE un respaldo generado por nosotros: solo
que entendemos cómo rotula lo que ya guarda.
"""

from __future__ import annotations

import html
import re
import sys
from dataclasses import dataclass, field

from pad import bindio
from pad.backup import Action, PadBackup
from pad.listing import io_name


def to_text(page: str) -> str:
    page = re.sub(r"<br\s*/?>", "\n", page, flags=re.I)
    page = re.sub(r"<[^>]+>", "", page)
    return html.unescape(page).replace("\xa0", " ")


Entry = list[str]            # la línea de la instrucción y sus continuaciones


def parse_listing(page: str) -> dict[int | str, list[Entry]]:
    """{"main" o id de módulo: [entrada, ...]}. Cada entrada es una lista de
    líneas de texto (el movimiento ocupa varias)."""
    units: dict[int | str, list[Entry]] = {}
    current: int | str | None = None
    entry: Entry | None = None
    for line in to_text(page).splitlines():
        if line.startswith("mainProgram:"):
            current, entry = "main", None
            units[current] = []
            continue
        m = re.match(r"^fuction\[(\d+)\]:", line)
        if m:
            current, entry = int(m.group(1)), None
            units[current] = []
            continue
        if line.startswith("subProgram"):
            current = None
            continue
        if current is None:
            continue
        m = re.match(r"^\s*(\d+):\s+(.*)$", line)
        if m:
            entry = [m.group(2).strip()]
            units[current].append(entry)
        elif line.strip() and entry is not None:
            entry.append(line.strip())
    return units


@dataclass
class Report:
    entries: int = 0
    checked: dict[str, int] = field(default_factory=dict)
    problems: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.problems


_HEAD = {4: "Free Path:", 10: "Line3D-Pose:", 100: "DLY:", 200: "Output:", 800: "Switch Coord",
         801: "Switch Tool", 10001: "IF:", 20000: "Call Fun", 20001: "Module End",
         59999: "Flag[", 60000: "Program End", 53000: "Cmd Set"}
_NUMBER = r"-?\d+(?:\.\d+)?"


def _near(a: str, b: str) -> bool:
    return abs(float(a) - float(b)) < 1e-6


def check(backup: PadBackup, listing: dict[int | str, list[Entry]]) -> Report:
    """Cruza cada acción del respaldo con su entrada del listado del pad."""
    rep = Report()
    units = [("main", backup.act.main)] + [(m.id, m.actions) for m in backup.act.modules]

    def ok(key: str, good: bool, where, detail: str = "") -> None:
        rep.checked[key] = rep.checked.get(key, 0) + 1
        if not good:
            rep.problems.append(f"{where}: {key} {detail}".strip())

    for unit, actions in units:
        entries = listing.get(unit)
        if entries is None:
            rep.problems.append(f"{unit}: no está en el listado")
            continue
        if len(entries) != len(actions):
            rep.problems.append(f"{unit}: {len(actions)} acciones y {len(entries)} entradas")
            continue
        for i, (a, e) in enumerate(zip(actions, entries)):
            rep.entries += 1
            where = f"{unit}[{i}]"
            code, head, body = a["action"], e[0], " ".join(e)
            if code in _HEAD:
                ok(f"tipo {code}", _HEAD[code] in head, where, head[:50])
            if code in (4, 10):
                _check_move(a, body, head, where, ok)
            elif code == 200:
                m = re.search(r"Output:(Y\d+)(ON|OFF) DLY([\d.]+)", body)
                ok("SET_OUT", bool(m) and m.group(1) == io_name("Y", a["point"])
                   and (m.group(2) == "ON") == bool(a["pointStatus"])
                   and _near(m.group(3), a["delay"]), where, body[:60])
            elif code == 100:
                m = re.match(r"DLY:([\d.]+)s", head)
                ok("WAIT", bool(m) and _near(m.group(1), a["limit"]), where, head)
            elif code == 800:
                ok("COORD", f"[CoordID{a['coordID']}]" in head, where, head)
            elif code == 801:
                ok("TOOL", f"[{a['toolID']}:" in head, where, head)
            elif code == 10001:
                m = re.match(rf"IF:(X\d+)(ON|OFF) Lim:({_NUMBER}) Go to Flag\[(-?\d+)\]", head)
                ok("IF", bool(m) and m.group(1) == io_name("X", a["point"])
                   and (m.group(2) == "ON") == bool(a["pointStatus"])
                   and int(m.group(4)) == int(a["flag"]), where, head)
            elif code == 20000:
                m = re.match(r"Call Fun\[(\d+)\]:.*? And then return to "
                             r"(next line|Flag\[(-?\d+)\])", head)
                ok("CALL", bool(m) and int(m.group(1)) == int(a["module"])
                   and ((m.group(2) == "next line") == (str(a["flag"]) == "-1"))
                   and (m.group(3) is None or int(m.group(3)) == int(a["flag"])), where, head)
            elif code == 59999:
                m = re.match(r"Flag\[(-?\d+)\]:(.*)", head)
                ok("etiqueta", bool(m) and int(m.group(1)) == int(a["flag"])
                   and m.group(2) == a["comment"], where, head)
            elif code == 50000 and "commentAction" in a:
                # El pad guarda en `comment` el texto con que muestra la acción
                # original. En un IF agrega el nombre de la etiqueta destino
                # (`Go to Flag[0]:RejaDe6Fila2`), que en pantalla no se ve.
                text = html.unescape(a["comment"]).replace("\xa0", " ").strip()
                good = text.startswith(head)
                inner = a["commentAction"]
                if good and inner.get("action") == 10001:
                    names = {x["flag"]: x["comment"] for x in actions if x["action"] == 59999}
                    good = text == head + names.get(inner["flag"], "")
                ok("desactivada", good, where, f"{text[:60]!r} / {head[:40]!r}")
            elif code == 50000:
                ok("comentario", html.unescape(a["comment"]).strip() == head, where, head[:40])
    return rep


def _check_move(a: Action, body: str, head: str, where, ok) -> None:
    pos = a["points"][0]["pos"]
    nums = re.findall(rf"(?:J\d|[XYZUVW]):({_NUMBER})", body)
    ok("valores del punto", nums[:6] == [f"{float(pos[f'm{i}']):.3f}" for i in range(6)], where)
    v = re.search(r"\bV([\d.]+)", body)
    ok("velocidad", bool(v) and _near(v.group(1), a["speed"]), where)
    d = re.search(r"DLY([\d.]+)", body)
    ok("demora del movimiento", bool(d) and _near(d.group(1), a["delay"]), where)
    s = re.search(r"Smooth:(\w+)", body)
    ok("suavizado", bool(s) and (int(a["smooth"]) == 0 if s.group(1) == "None"
                                 else int(s.group(1)) == int(a["smooth"])), where)
    name = a.get("customName") or ""
    ok("nombre del punto", head.startswith("*" + name + " ") if name else not head.startswith("*"),
       where, head[:40])
    if a["action"] == 10:
        t, c = int(a["toolCoord"]) >> 16, int(a["toolCoord"]) & 0xFFFF
        m = re.search(r"tool:(\d+):.*?coord:(\d+):", body)
        ok("herramienta y coordenada", bool(m) and (int(m.group(1)), int(m.group(2))) == (t, c), where)
    texts = re.findall(r"whiledistance[^\n]*?mm then output Y\d+ (?:on|off)", body)
    bind = bindio.decode(int(a.get("bindIOInfo") or 0))
    if bind is not None and bind.enabled:
        want = bindio.pad_text(bind, float(a["distance"]), io_name("Y", bind.point))
        ok("salida por distancia", [t.replace(" ", "") for t in texts] == [want.replace(" ", "")],
           where, f"{texts} / {want}")
    elif int(a.get("bindIOInfo") or 0) == 0 or (bind is not None and not bind.enabled):
        ok("sin salida por distancia", not texts, where, str(texts))
    else:
        ok("bindIOInfo conocido", False, where, hex(int(a["bindIOInfo"])))


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 2:
        print("uso: python -m pad.pad_html RESPALDO.zip LISTADO.html")
        return 2
    backup = PadBackup.read(args[0])
    with open(args[1], encoding="utf-8") as f:
        report = check(backup, parse_listing(f.read()))
    print(f"{report.entries} entradas cruzadas")
    for key, n in sorted(report.checked.items()):
        print(f"  {key}: {n}")
    for p in report.problems[:50]:
        print("PROBLEMA:", p)
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
