"""
Panel "Campos de trabajo", inspirado en los de WorkVisual: cambiar de campo
reorganiza qué paneles se ven, en vez de tener todo abierto siempre.

La idea es que las dos tareas reales del sistema necesitan cosas distintas:
programar quiere el editor y la estructura del programa; poner en servicio
quiere la conexión, los puntos y los mensajes del robot. Tener los dos juegos
de paneles abiertos a la vez deja la ventana ilegible en una pantalla de
notebook.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QListWidget, QListWidgetItem


@dataclass(frozen=True)
class WorkField:
    key: str
    label: str
    description: str
    # Nombres de los docks que este campo muestra. Los que no estén listados
    # se ocultan al entrar al campo.
    visible_docks: tuple[str, ...] = field(default_factory=tuple)


PROGRAMACION = WorkField(
    key="programacion",
    label="Programación",
    description="Escribir y compilar el programa",
    # La conexión también va acá: ejecutar el programa la necesita, y mandar al
    # usuario a cambiar de campo solo para apretar "Conectar" es una molestia
    # sin razón.
    visible_docks=("estructura", "campos", "propiedades", "conexion", "plc", "mensajes", "log"),
)

PUESTA_EN_SERVICIO = WorkField(
    key="puesta_en_servicio",
    label="Puesta en servicio",
    description="Conectar el robot, digitalizar puntos y probar movimientos",
    visible_docks=("campos", "conexion", "plc", "mensajes", "log"),
)

WORK_FIELDS = [PROGRAMACION, PUESTA_EN_SERVICIO]


class WorkFieldsPanel(QListWidget):
    field_changed = Signal(str)  # key del campo elegido

    def __init__(self) -> None:
        super().__init__()
        for wf in WORK_FIELDS:
            item = QListWidgetItem(wf.label)
            item.setToolTip(wf.description)
            item.setData(int(0x0100), wf.key)  # Qt.UserRole
            self.addItem(item)
        self.setCurrentRow(0)
        self.currentRowChanged.connect(self._on_row_changed)

    def _on_row_changed(self, row: int) -> None:
        if 0 <= row < len(WORK_FIELDS):
            self.field_changed.emit(WORK_FIELDS[row].key)

    def current_field(self) -> WorkField:
        return WORK_FIELDS[max(0, self.currentRow())]

    def select(self, key: str) -> None:
        for row, wf in enumerate(WORK_FIELDS):
            if wf.key == key:
                self.setCurrentRow(row)
                return
        raise KeyError(f"Campo de trabajo desconocido: {key!r}")


def field_by_key(key: str) -> WorkField:
    for wf in WORK_FIELDS:
        if wf.key == key:
            return wf
    raise KeyError(f"Campo de trabajo desconocido: {key!r}")
