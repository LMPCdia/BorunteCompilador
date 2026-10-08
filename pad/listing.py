"""
Listado legible de un respaldo del pad: la forma rápida de ver qué entendimos
del formato y qué no.

    python -m pad.listing HCBackupRobot_20260814213843.zip

Cada acción conocida se muestra con un nombre tipo KRL. Las desconocidas se
muestran con su código y sus campos crudos, para que salten a la vista en vez
de esconderse. Los significados son DEDUCIDOS de un solo respaldo real — ver
`docs/PAD_FORMAT.md`, donde cada uno lleva su grado de confianza.
"""

from __future__ import annotations

import json
import re
import sys
from typing import Any

from pad.backup import Action, Module, PadBackup

MOVEJ = 4
MOVEL = 10
WAIT_TIME = 100
SET_OUT = 200
SET_COORD = 800
SET_TOOL = 801
IF_INPUT_GOTO = 10001
CALL_MODULE = 20000
MODULE_END = 20001
COMMENT = 50000
SPECIAL_WRITE = 53000
LABEL = 59999
PROGRAM_END = 60000

# Ignorados en el listado cuando tienen el valor de todos los movimientos del
# respaldo real. Si alguno aparece con otro valor, se muestra.
_MOVE_DEFAULTS = {
    "bindIOInfo": 0,
    "ckStatus": "63",
    "delay": "0.000",
    "distance": "0.000",
    "passTrans": 0,
    "quotePoint": [0, 0, 0],
    "relativeType": 0,
    "smooth": 0,
}
_MOVE_SHOWN = {"action", "insertedIndex", "points", "speed", "toolCoord", "customName"}


def io_name(prefix: str, point: int) -> str:
    """Nombre que muestra el pad para una E/S, a partir del `point` del archivo.

    Deducido de 3 casos (`point` 2 -> X012, 3 -> X013, 20 -> Y034): numeración
    octal que empieza en 010. Sin confirmar para el resto del rango.
    """
    return f"{prefix}{point + 8:03o}"


def split_tool_coord(value: int) -> tuple[int, int]:
    """`toolCoord` = herramienta en los 16 bits altos, sistema de coordenadas
    en los bajos. Deducido: coincide con las acciones 800/801 de cada módulo."""
    return value >> 16, value & 0xFFFF


def _labels(actions: list[Action]) -> dict[int, str]:
    return {a["flag"]: a.get("comment", "") for a in actions if a["action"] == LABEL}


def describe(action: Action, labels: dict[int, str], modules: dict[int, str]) -> str:
    code = action["action"]
    if code in (MOVEJ, MOVEL):
        kind = "MOVEJ" if code == MOVEJ else "MOVEL"
        pos = action["points"][0]["pos"]
        values = ", ".join(pos[f"m{i}"] for i in range(6))
        tool, coord = split_tool_coord(action["toolCoord"])
        extra = [f"{k}={v}" for k, v in action.items()
                 if k not in _MOVE_SHOWN and _MOVE_DEFAULTS.get(k, object()) != v]
        name = f"  ; {action['customName']}" if action.get("customName") else ""
        text = f"{kind} ({values}) SPEED {action['speed']} TOOL {tool} COORD {coord}"
        return text + (" " + " ".join(extra) if extra else "") + name
    if code == WAIT_TIME:
        return f"WAIT {action['limit']} s"
    if code == SET_OUT:
        state = "ON" if action["pointStatus"] else "OFF"
        return f"SET_OUT({io_name('Y', action['point'])}, {state}) delay={action['delay']}"
    if code == SET_COORD:
        return f"COORD {action['coordID']}"
    if code == SET_TOOL:
        return f"TOOL {action['toolID']}"
    if code == IF_INPUT_GOTO:
        state = "ON" if action["pointStatus"] else "OFF"
        target = labels.get(action["flag"], f"flag{action['flag']}")
        return f"IF {io_name('X', action['point'])} {state} GOTO {target}"
    if code == CALL_MODULE:
        module_id = int(action["module"])
        text = f"CALL {modules.get(module_id, f'modulo{module_id}')}()"
        # flag -1 (entero): sin salto. flag "0" (string): se observó solo en
        # llamadas seguidas de un salto lógico a esa etiqueta. Hipótesis.
        if action["flag"] != -1:
            text += f" THEN GOTO {labels.get(int(action['flag']), 'flag' + str(action['flag']))}"
        return text
    if code == MODULE_END:
        return "ENDPROC"
    if code == PROGRAM_END:
        return "END"
    if code == LABEL:
        return f"{action.get('comment', '')}:   ; flag {action['flag']}"
    if code == COMMENT:
        text = re.sub(r"(&nbsp;)+", " ", action.get("comment", "")).strip()
        if "commentAction" in action:
            inner = describe(action["commentAction"], labels, modules)
            return f"; [desactivada] {inner}"
        return f"; {text}"
    fields = {k: v for k, v in action.items() if k not in ("action", "insertedIndex")}
    return f"?? acción {code} {json.dumps(fields, ensure_ascii=False)}"


def list_program(title: str, actions: list[Action], modules: dict[int, str]) -> list[str]:
    labels = _labels(actions)
    out = [title]
    for action in actions:
        out.append(f"  [{action.get('insertedIndex', '?'):>3}] {describe(action, labels, modules)}")
    return out


def list_backup(backup: PadBackup) -> list[str]:
    modules: list[Module] = backup.act.modules
    names = {m.id: m.name for m in modules}
    out = [f"Programa: {backup.name}", ""]
    out += list_program("MAIN", backup.act.main, names)
    for module in modules:
        out.append("")
        out += list_program(f"PROC {module.name}()   ; módulo {module.id}", module.actions, names)
    return out


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("Uso: python -m pad.listing HCBackupRobot_XXXXXXXXXXXXXX.zip", file=sys.stderr)
        return 2
    for line in list_backup(PadBackup.read(argv[0])):
        print(line)
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
