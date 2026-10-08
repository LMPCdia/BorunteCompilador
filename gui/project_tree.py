"""
Panel "Estructura del proyecto": puntos, procedimientos, variables y slots de
parámetros del programa compilado. Un clic lleva el cursor del editor a la
línea de la declaración.

Sobre cómo se ubican esas líneas: con una pasada de expresiones regulares
sobre el texto fuente, NO reparseando con Lark. Dos razones:

1. Así el árbol también funciona con el archivo a medio escribir, que es justo
   cuando más se necesita ver la estructura. Un reparseo fallaría con un error
   de sintaxis y no mostraría nada.
2. `Program` no guarda números de línea, y el bytecode no los necesita.
   Agregárselos solo para que la GUI navegue sería meter información de la
   interfaz en el contrato del compilador.

El precio es que la regex puede errarle en casos raros (un `POINT` dentro de un
comentario multilinea, que el DSL no tiene). Si no encuentra la línea, el nodo
simplemente no navega.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QTreeWidget, QTreeWidgetItem

from runtime.bytecode import Program

# Qt.UserRole guarda el tipo de nodo; UserRole+1 la línea; UserRole+2 las
# propiedades.
ROLE_KIND = int(Qt.ItemDataRole.UserRole)
ROLE_LINE = int(Qt.ItemDataRole.UserRole) + 1
ROLE_PROPERTIES = int(Qt.ItemDataRole.UserRole) + 2

# Los 6 valores de una Pose, en el orden en que los nombra el robot. El orden
# importa: ver el comentario sobre QVariantMap en properties_panel.py.
POSE_AXIS_NAMES = ["X", "Y", "Z", "U", "V", "W"]

_DECL_PATTERNS = {
    "point": re.compile(r"^\s*POINT\s+([A-Za-z_]\w*)\s*="),
    "var": re.compile(r"^\s*VAR\s+([A-Za-z_]\w*)\s*:"),
    "timer": re.compile(r"^\s*TIMER\s+([A-Za-z_]\w*)\s*="),
    "proc": re.compile(r"^\s*PROC\s+([A-Za-z_]\w*)\s*\("),
}


@dataclass(frozen=True)
class Declaration:
    kind: str
    name: str
    line: int  # 1-based, como las muestra un editor


def find_declarations(source: str) -> dict[tuple[str, str], int]:
    """(tipo, nombre) -> número de línea 1-based."""
    found: dict[tuple[str, str], int] = {}
    for lineno, line in enumerate(source.splitlines(), start=1):
        # un comentario que arranca la línea no declara nada
        if line.lstrip().startswith(";"):
            continue
        code = line.split(";", 1)[0]
        for kind, pattern in _DECL_PATTERNS.items():
            match = pattern.match(code)
            if match:
                found.setdefault((kind, match.group(1)), lineno)
    return found


class ProjectTree(QTreeWidget):
    """Árbol de la estructura. Emite `line_requested` al clickear un nodo que
    tenga una línea conocida."""

    line_requested = Signal(int)

    def __init__(self) -> None:
        super().__init__()
        self.setHeaderLabels(["Elemento", "Detalle"])
        self.setColumnWidth(0, 200)
        self.itemClicked.connect(self._on_item_clicked)

    def _on_item_clicked(self, item: QTreeWidgetItem, _column: int) -> None:
        line = item.data(0, ROLE_LINE)
        if line:
            self.line_requested.emit(int(line))

    # -- construcción ---------------------------------------------------------

    def rebuild(self, program: Program | None, source: str = "") -> None:
        self.clear()
        if program is None:
            placeholder = QTreeWidgetItem(["(sin compilar)", ""])
            self.addTopLevelItem(placeholder)
            return

        declarations = find_declarations(source)
        self._add_points(program, declarations)
        self._add_procs(program, declarations)
        self._add_variables(program, declarations)
        self.expandAll()

    def _make_node(
        self,
        parent: QTreeWidgetItem,
        label: str,
        detail: str,
        kind: str,
        properties: list[tuple[str, str]],
        line: int | None = None,
    ) -> QTreeWidgetItem:
        item = QTreeWidgetItem([label, detail])
        item.setData(0, ROLE_KIND, kind)
        # Lista de pares, NO dict: ver properties_panel.py.
        item.setData(0, ROLE_PROPERTIES, properties)
        if line:
            item.setData(0, ROLE_LINE, line)
        parent.addChild(item)
        return item

    def _add_points(self, program: Program, declarations: dict) -> None:
        total = len(program.points)
        nombrados = len(program.point_names)
        root = QTreeWidgetItem([
            "Puntos",
            f"{nombrados} declarado(s), {total} entrada(s) en la tabla",
        ])
        self.addTopLevelItem(root)

        for name, index in sorted(program.point_names.items(), key=lambda kv: kv[1]):
            pose = program.points[index]
            coords = list(zip(POSE_AXIS_NAMES, (pose.a, pose.b, pose.c, pose.d, pose.e, pose.f)))
            properties = [
                ("Tipo", "Punto"),
                ("Nombre", name),
                ("Índice en la tabla", str(index)),
                *[(axis, f"{value:g}") for axis, value in coords],
            ]
            self._make_node(
                root,
                name,
                f"#{index}  ({pose.a:g}, {pose.b:g}, {pose.c:g})",
                "point",
                properties,
                declarations.get(("point", name)),
            )

    def _add_procs(self, program: Program, declarations: dict) -> None:
        root = QTreeWidgetItem(["Procedimientos", f"{len(program.proc_addresses)}"])
        self.addTopLevelItem(root)

        for name, pc in sorted(program.proc_addresses.items(), key=lambda kv: kv[1]):
            params = program.proc_params.get(name, [])
            properties = [
                ("Tipo", "Procedimiento"),
                ("Nombre", name),
                ("PC de entrada", str(pc)),
                ("Parámetros", ", ".join(params) or "(ninguno)"),
            ]
            proc_item = self._make_node(
                root,
                f"{name}({', '.join(params)})",
                f"PC {pc}",
                "proc",
                properties,
                declarations.get(("proc", name)),
            )

            # Los slots de parámetros cuelgan del PROC: es donde uno los busca.
            for param in params:
                slot_name = f"{name}.{param}"
                index = program.var_names.get(slot_name)
                if index is None:
                    continue
                self._make_node(
                    proc_item,
                    param,
                    f"slot {slot_name}",
                    "param",
                    [
                        ("Tipo", "Slot de parámetro"),
                        ("Nombre del slot", slot_name),
                        ("Pertenece a", name),
                        ("Índice de variable", str(index)),
                        ("Nota", "Sin pila de frames: no es reentrante"),
                    ],
                    declarations.get(("proc", name)),
                )

    def _add_variables(self, program: Program, declarations: dict) -> None:
        # Los slots de parámetros están en var_names pero se muestran bajo su
        # PROC, no acá — se distinguen por el punto en el nombre.
        user_vars = {n: i for n, i in program.var_names.items() if "." not in n}
        root = QTreeWidgetItem(["Variables", f"{len(user_vars)}"])
        self.addTopLevelItem(root)

        for name, index in sorted(user_vars.items(), key=lambda kv: kv[1]):
            self._make_node(
                root,
                name,
                f"índice {index}",
                "var",
                [
                    ("Tipo", "Variable"),
                    ("Nombre", name),
                    ("Índice de variable", str(index)),
                ],
                declarations.get(("var", name)),
            )
