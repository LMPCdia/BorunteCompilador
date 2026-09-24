"""
Lector del formato `.act` nativo del robot Borunte (parser inverso).

Sirve para dos cosas: verificar en los tests lo que genera
`compiler/act_backend.py`, y leer programas existentes exportados del robot.

## El formato, tal como se observa en un export real

El `.act` NO es un único documento JSON: es **JSON Lines**, un documento JSON
por línea. En el export analizado (`AberturaSanLorenzo2026.act`, 338 KB):

    línea  0      programa principal — lista de acciones
    líneas 1-8    ocho documentos `[{"action":60000,"insertedIndex":0}]`
    línea  9      `{}`
    línea  10     biblioteca de subprogramas — lista de {id, name, program}
    líneas 11-18  otros ocho `[{"action":60000,"insertedIndex":0}]`

El campo `program` de cada entrada de la biblioteca **no es una lista: es un
string que contiene JSON** — o sea, está codificado dos veces. Hay que hacerle
`json.loads` aparte.

Las líneas que no son ni el programa principal ni la biblioteca no se
interpretan: se conservan tal cual para poder reescribir el archivo sin
perderlas. No sabemos qué significan y no hace falta saberlo para leer o
generar programas.

## Orden de ejecución

El orden de ejecución es **el orden del array**, no el campo `insertedIndex`.
En el export real el principal tiene `insertedIndex` 13, 15, 14, 16 en ese
orden dentro de la lista: `insertedIndex` es un identificador del orden en que
el operario fue insertando las líneas en el pad, no la secuencia de ejecución.
Confundirlos daría un programa reordenado.

⚠️ Este módulo describe **lo que se observa en un export real**, no un formato
documentado por Borunte. Lo que está confirmado es la estructura (cómo se
serializa); la semántica de cada código de acción está anotada en
`ACTION_NAMES` con su nivel de certeza.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# Serialización idéntica a la del robot: claves ordenadas alfabéticamente y sin
# espacios. Verificado con un round-trip byte a byte contra el export real (ver
# tests/test_act_backend.py).
JSON_SEPARATORS = (",", ":")


def dumps_act(value: Any) -> str:
    """Serializa como lo hace el robot."""
    return json.dumps(value, sort_keys=True, separators=JSON_SEPARATORS, ensure_ascii=True)


# Códigos de acción observados en el export real. El nivel de certeza importa:
# "confirmado" significa que la forma del registro no deja lugar a dudas;
# "hipótesis" significa que lo dedujimos del contexto y hay que validarlo con el
# robot antes de depender de ello.
ACTION_NAMES: dict[int, str] = {
    4: "MOVE_A",            # hipótesis: un tipo de movimiento (ver nota abajo)
    10: "MOVE_B",           # hipótesis: el otro tipo de movimiento
    100: "WAIT",            # hipótesis: espera con límite en segundos
    200: "SET_OUT",         # confirmado por la forma: valveID + pointStatus booleano
    800: "BASE",            # confirmado por la forma: coordID
    801: "TOOL",            # confirmado por la forma: toolID
    10001: "IF_INPUT",      # confirmado por la forma: point + pointStatus + flag destino
    20000: "CALL",          # confirmado por la forma: module = id de la biblioteca
    20001: "END_SUB",       # confirmado: aparece exactamente una vez al final de cada subprograma
    50000: "COMMENT",       # confirmado por la forma: comment
    53000: "REG_WRITE",     # desconocido: addr/data/op/specialType/typeSel
    59999: "LABEL",         # confirmado por la forma: flag que emparejan los IF_INPUT
    60000: "END_MAIN",      # confirmado: aparece exactamente una vez al final del principal
}

# 4 vs 10: los dos tienen EXACTAMENTE los mismos campos (points, speed, smooth,
# toolCoord, ...), así que por la forma no se pueden distinguir. Cuál es MOVEJ y
# cuál MOVEL no se puede deducir del archivo — hay que verlo en el pad.

#: Los saltos NO van por índice: van por etiqueta. Un `IF_INPUT` (10001) lleva un
#: `flag`, y salta al `LABEL` (59999) que tenga el mismo `flag`. Eso hace que
#: generar código sea más simple que con el bytecode del PLC: alcanza con emitir
#: etiquetas únicas, sin backpatching de direcciones.
JUMP_FIELD = "flag"


@dataclass
class ActSubprogram:
    """Una entrada de la biblioteca: un subprograma con nombre e id."""
    id: int
    name: str
    actions: list[dict]

    def action_codes(self) -> list[int]:
        return [a["action"] for a in self.actions]


@dataclass
class ActFile:
    """Un `.act` completo, conservando lo que no interpretamos."""

    #: Cada línea del archivo ya parseada, en orden. Las que sí interpretamos
    #: siguen acá también: `main_line` y `library_line` dicen cuáles son.
    documents: list[Any] = field(default_factory=list)
    main_line: int | None = None
    library_line: int | None = None

    @property
    def main(self) -> list[dict]:
        """Acciones del programa principal, en orden de ejecución."""
        if self.main_line is None:
            return []
        return self.documents[self.main_line]

    @property
    def library(self) -> list[ActSubprogram]:
        if self.library_line is None:
            return []
        return [
            ActSubprogram(id=e["id"], name=e["name"], actions=json.loads(e["program"]))
            for e in self.documents[self.library_line]
        ]

    def subprogram(self, key: int | str) -> ActSubprogram:
        """Busca un subprograma por id o por nombre."""
        for sub in self.library:
            if sub.id == key or sub.name == key:
                return sub
        raise KeyError(f"No hay subprograma {key!r} en la biblioteca")

    def action_codes(self) -> list[int]:
        return [a["action"] for a in self.main]

    def to_text(self) -> str:
        """Reescribe el archivo.

        Round-trip byte a byte con el export real del robot: sin salto de línea
        final, claves ordenadas y sin espacios. Verificado sobre un export de
        338 KB en `tests/test_act_backend.py`.
        """
        return "\n".join(dumps_act(doc) for doc in self.documents)


def _looks_like_actions(doc: Any) -> bool:
    return (
        isinstance(doc, list)
        and bool(doc)
        and all(isinstance(a, dict) and "action" in a for a in doc)
    )


def _looks_like_library(doc: Any) -> bool:
    return (
        isinstance(doc, list)
        and bool(doc)
        and all(isinstance(e, dict) and {"id", "name", "program"} <= set(e) for e in doc)
    )


def parse_act(text: str) -> ActFile:
    """Parsea el contenido de un `.act`.

    El programa principal es la **primera** línea que sea una lista de acciones
    con más de una acción: las líneas de relleno son listas de una sola acción
    `60000`, y tomar una de esas como programa principal daría un programa vacío.
    """
    documentos: list[Any] = []
    main_line: int | None = None
    library_line: int | None = None

    for numero, linea in enumerate(text.splitlines()):
        if not linea.strip():
            continue
        try:
            doc = json.loads(linea)
        except json.JSONDecodeError as e:
            raise ValueError(f"La línea {numero} del .act no es JSON válido: {e}") from e

        indice = len(documentos)
        documentos.append(doc)

        if library_line is None and _looks_like_library(doc):
            library_line = indice
        elif main_line is None and _looks_like_actions(doc) and len(doc) > 1:
            main_line = indice

    return ActFile(documents=documentos, main_line=main_line, library_line=library_line)


def read_act(path: str | Path) -> ActFile:
    # utf-8-sig: el export del robot puede traer BOM.
    return parse_act(Path(path).read_text(encoding="utf-8-sig"))
