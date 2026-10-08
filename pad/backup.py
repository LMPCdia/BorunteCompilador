"""
Lectura y escritura de los respaldos que el pad del Borunte exporta e importa
por pendrive (`HCBackupRobot_<AAAAMMDDhhmmss>.zip`).

Todo lo que sabemos del formato salió de comparar UN respaldo real, así que
es **deducido, no confirmado por Borunte** — ver `docs/PAD_FORMAT.md`.

La regla de diseño de este módulo es no perder nada que no entendamos:

- Las acciones se guardan como `dict` tal cual vienen del archivo, con todos
  sus campos, incluso los que todavía no sabemos qué hacen.
- Las líneas del `.act` cuyo significado desconocemos se conservan crudas.
- Los demás archivos del respaldo (`.fnc`, `.timers`, ...) viajan como bytes.

Con eso, leer y volver a escribir un respaldo da el mismo contenido byte a
byte, y es la prueba de que el modelo no se está comiendo información.
"""

from __future__ import annotations

import io
import json
import re
import zipfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

# Línea del .act (base 0) con la lista de módulos/subrutinas. Deducido de un
# solo respaldo de 19 líneas: la 0 es el programa principal, la 10 los
# módulos, y el resto (1-9, 11-18) no se sabe todavía qué son.
MAIN_LINE = 0
MODULES_LINE = 10

# `.fnc` del respaldo real analizado. Se usa al generar un respaldo nuevo si no
# hay uno del robot propio para copiarlo (ver compiler/pad_codegen.py). Se lee
# como archivo en tiempo de ejecución: tiene que viajar en el .exe.
TEMPLATE_FNC = Path(__file__).resolve().parent / "template.fnc"

# El pad solo reimporta zips con este nombre (reportado en el foro de RoboDK,
# y coincide con el respaldo real que tenemos).
BACKUP_NAME_RE = re.compile(r"HCBackupRobot_(\d{14})\.zip$")
BACKUP_TIMESTAMP_FORMAT = "%Y%m%d%H%M%S"

Action = dict[str, Any]


def _dumps(value: Any) -> str:
    # Compacto y sin escapar no-ASCII: es lo que reproduce el archivo del pad
    # byte a byte. Las claves NO se ordenan acá: se respeta el orden en que
    # vienen (que en el respaldo real resulta ser alfabético).
    return json.dumps(value, separators=(",", ":"), ensure_ascii=False)


@dataclass
class Module:
    """Una subrutina del pad. Se llama desde otra con la acción 20000."""

    id: int
    name: str
    actions: list[Action]

    def to_json(self) -> dict[str, Any]:
        # `program` es un string con JSON adentro, no una lista: así lo guarda
        # el pad.
        return {"id": self.id, "name": self.name, "program": _dumps(self.actions)}

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "Module":
        return cls(id=data["id"], name=data["name"], actions=json.loads(data["program"]))


@dataclass
class ActFile:
    """El `.act`: una línea JSON por bloque, separadas por `\\n`, sin `\\n` final."""

    lines: list[Any]

    @property
    def main(self) -> list[Action]:
        return self.lines[MAIN_LINE]

    @main.setter
    def main(self, actions: list[Action]) -> None:
        self.lines[MAIN_LINE] = actions

    @property
    def modules(self) -> list[Module]:
        return [Module.from_json(m) for m in self.lines[MODULES_LINE]]

    @modules.setter
    def modules(self, modules: list[Module]) -> None:
        self.lines[MODULES_LINE] = [m.to_json() for m in modules]

    def module_by_id(self, module_id: int) -> Module | None:
        return next((m for m in self.modules if m.id == module_id), None)

    @classmethod
    def loads(cls, text: str) -> "ActFile":
        lines = [json.loads(line) for line in text.split("\n")]
        if len(lines) <= MODULES_LINE or not isinstance(lines[MODULES_LINE], list):
            raise ValueError(
                f"El .act tiene {len(lines)} líneas; se esperaba la lista de módulos "
                f"en la línea {MODULES_LINE + 1}. ¿Otra versión del pad?"
            )
        return cls(lines)

    def dumps(self) -> str:
        return "\n".join(_dumps(line) for line in self.lines)


@dataclass
class PadBackup:
    """Un respaldo completo: un programa (`name`) con todos sus archivos."""

    name: str
    act: ActFile
    # Los demás archivos (`.fnc`, `.counters`, ...), por extensión, crudos.
    others: dict[str, bytes] = field(default_factory=dict)
    timestamp: datetime | None = None

    @classmethod
    def read(cls, path: str | Path) -> "PadBackup":
        path = Path(path)
        match = BACKUP_NAME_RE.search(path.name)
        timestamp = datetime.strptime(match.group(1), BACKUP_TIMESTAMP_FORMAT) if match else None
        with zipfile.ZipFile(path) as zf:
            return cls._from_zip(zf, timestamp)

    @classmethod
    def from_bytes(cls, data: bytes, timestamp: datetime | None = None) -> "PadBackup":
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            return cls._from_zip(zf, timestamp)

    @classmethod
    def _from_zip(cls, zf: zipfile.ZipFile, timestamp: datetime | None) -> "PadBackup":
        acts = [n for n in zf.namelist() if n.endswith(".act")]
        if len(acts) != 1:
            raise ValueError(f"Se esperaba exactamente un .act en el respaldo, hay {len(acts)}: {acts}")
        name = acts[0][: -len(".act")]
        act = ActFile.loads(zf.read(acts[0]).decode("utf-8"))
        others: dict[str, bytes] = {}
        for entry in zf.namelist():
            if entry == acts[0]:
                continue
            if not entry.startswith(name + "."):
                raise ValueError(f"Archivo inesperado en el respaldo: {entry!r}")
            others[entry[len(name) + 1 :]] = zf.read(entry)
        return cls(name=name, act=act, others=others, timestamp=timestamp)

    def file_name(self) -> str:
        stamp = self.timestamp or datetime.now()
        return f"HCBackupRobot_{stamp.strftime(BACKUP_TIMESTAMP_FORMAT)}.zip"

    def to_bytes(self) -> bytes:
        stamp = self.timestamp or datetime.now()
        files = {"act": self.act.dumps().encode("utf-8"), **self.others}
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            # Mismo orden (alfabético por extensión) y mismos metadatos que el
            # respaldo real: creado en Unix, permisos 0644.
            for ext in sorted(files):
                info = zipfile.ZipInfo(f"{self.name}.{ext}", date_time=stamp.timetuple()[:6])
                info.compress_type = zipfile.ZIP_DEFLATED
                info.create_system = 3
                info.external_attr = 0o100644 << 16
                zf.writestr(info, files[ext])
        return buffer.getvalue()

    def write(self, directory: str | Path) -> Path:
        """Escribe el zip con el nombre que el pad acepta y devuelve la ruta."""
        path = Path(directory) / self.file_name()
        path.write_bytes(self.to_bytes())
        return path
