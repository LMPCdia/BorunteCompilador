"""
Ventana de mensajes: una fila por evento, con severidad, hora y origen.

Reemplaza al hábito anterior de escribir los errores de compilación DENTRO del
tab de bytecode, que era un lugar raro para buscarlos: el usuario abría
"Bytecode" esperando bytecode y encontraba un traceback. Acá los eventos de
todas las partes del sistema (compilador, conexión, VM, digitalización) quedan
en un solo lugar y en orden cronológico.
"""

from __future__ import annotations

import time
from enum import Enum

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


class Severity(Enum):
    INFO = ("Info", "#3573b8")
    WARNING = ("Advertencia", "#b8860b")
    ERROR = ("Error", "#c0392b")

    @property
    def label(self) -> str:
        return self.value[0]

    @property
    def color(self) -> str:
        return self.value[1]


class MessageWindow(QWidget):
    COLUMNS = ["Severidad", "Hora", "Origen", "Mensaje"]

    def __init__(self) -> None:
        super().__init__()
        self._counts = {s: 0 for s in Severity}

        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)

        self.summary_label = QLabel()
        self.clear_btn = QPushButton("Limpiar")
        self.clear_btn.clicked.connect(self.clear)

        top = QHBoxLayout()
        top.addWidget(self.summary_label)
        top.addStretch()
        top.addWidget(self.clear_btn)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addLayout(top)
        layout.addWidget(self.table)

        self._refresh_summary()

    # -- API ----------------------------------------------------------------

    def add(self, severity: Severity, message: str, source: str = "") -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        values = [severity.label, time.strftime("%H:%M:%S"), source, message]
        for col, value in enumerate(values):
            item = QTableWidgetItem(value)
            if col == 0:
                item.setForeground(QColor(severity.color))
            item.setTextAlignment(
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
            )
            self.table.setItem(row, col, item)
        self._counts[severity] += 1
        self._refresh_summary()
        self.table.scrollToBottom()

    def info(self, message: str, source: str = "") -> None:
        self.add(Severity.INFO, message, source)

    def warning(self, message: str, source: str = "") -> None:
        self.add(Severity.WARNING, message, source)

    def error(self, message: str, source: str = "") -> None:
        self.add(Severity.ERROR, message, source)

    def clear(self) -> None:
        self.table.setRowCount(0)
        self._counts = {s: 0 for s in Severity}
        self._refresh_summary()

    def clear_source(self, source: str) -> None:
        """Borra solo los mensajes de un origen (p. ej. los de la simulación
        anterior, para que cada corrida no se apile sobre la otra)."""
        for row in reversed(range(self.table.rowCount())):
            if self.table.item(row, 2).text() == source:
                severity = next(s for s in Severity if s.label == self.table.item(row, 0).text())
                self._counts[severity] -= 1
                self.table.removeRow(row)
        self._refresh_summary()
        self._refresh_summary()

    def count(self, severity: Severity) -> int:
        return self._counts[severity]

    def messages(self) -> list[tuple[str, str, str]]:
        """(severidad, origen, mensaje) de cada fila — pensado para los tests."""
        out = []
        for row in range(self.table.rowCount()):
            out.append((
                self.table.item(row, 0).text(),
                self.table.item(row, 2).text(),
                self.table.item(row, 3).text(),
            ))
        return out

    # -- interno --------------------------------------------------------------

    def _refresh_summary(self) -> None:
        self.summary_label.setText(
            f"{self._counts[Severity.ERROR]} errores · "
            f"{self._counts[Severity.WARNING]} advertencias · "
            f"{self._counts[Severity.INFO]} informativos"
        )
