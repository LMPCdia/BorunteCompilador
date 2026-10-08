"""
Panel "Propiedades": detalle del nodo seleccionado en el árbol del proyecto.

Las propiedades llegan como **lista de pares**, no como dict. Es a propósito y
no es un detalle de estilo: guardar un dict en los datos de un QTreeWidgetItem
hace que Qt lo convierta a `QVariantMap`, que está **ordenado por clave**. El
síntoma era que las coordenadas de un punto se mostraban `U, V, W, X, Y, Z` en
vez de `X, Y, Z, U, V, W`. Con una lista de pares el orden es el que puso quien
armó el nodo, y hay un test que lo fija.
"""

from __future__ import annotations

from PySide6.QtWidgets import (
    QAbstractItemView,
    QHeaderView,
    QTableWidget,
    QTableWidgetItem,
)

from gui.project_tree import ROLE_PROPERTIES


class PropertiesPanel(QTableWidget):
    COLUMNS = ["Propiedad", "Valor"]

    def __init__(self) -> None:
        super().__init__(0, 2)
        self.setHorizontalHeaderLabels(self.COLUMNS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        header = self.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)

    def show_item(self, item) -> None:
        """Muestra las propiedades de un QTreeWidgetItem (o se limpia con None)."""
        if item is None:
            self.clear_properties()
            return
        properties = item.data(0, ROLE_PROPERTIES)
        if not properties:
            self.clear_properties()
            return
        self.show_properties([(str(k), str(v)) for k, v in properties])

    def show_properties(self, pairs: list[tuple[str, str]]) -> None:
        self.setRowCount(len(pairs))
        for row, (key, value) in enumerate(pairs):
            self.setItem(row, 0, QTableWidgetItem(key))
            self.setItem(row, 1, QTableWidgetItem(value))

    def clear_properties(self) -> None:
        self.setRowCount(0)

    def keys_in_order(self) -> list[str]:
        """Nombres de las propiedades mostradas, en orden — para los tests."""
        return [self.item(r, 0).text() for r in range(self.rowCount())]

    def value_of(self, key: str) -> str | None:
        for row in range(self.rowCount()):
            if self.item(row, 0).text() == key:
                return self.item(row, 1).text()
        return None
