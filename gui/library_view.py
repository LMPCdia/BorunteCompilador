"""
Pestaña "Biblioteca": los modelos 3D de la carpeta de Google Drive
(`sim/library.py`), clasificados por carpeta, listos para insertar en la
celda, montar como herramienta o usar como robot.

La red y la importación de robots (un par de minutos) corren en un hilo
aparte: la interfaz no se congela. Regla aprendida a golpes (CLAUDE.md): un
QThread no se suelta hasta que terminó (`wait()`), si no Qt crashea.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from PySide6.QtCore import QSettings, Qt, QThread, Signal
from PySide6.QtWidgets import (
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from sim import library
from sim.kinematics import USER_MODELS_DIR, RobotModel

KIND_LABEL = {"robot": "Robot", "herramienta": "Herramienta", "pieza": "Pieza"}
JOINTS_FROM = "BRTIRUS1510A"   # rangos y velocidades para un robot nuevo (HIPÓTESIS)


class _Task(QThread):
    done = Signal(object)
    failed = Signal(str)
    progress = Signal(str)

    def __init__(self, fn: Callable[[Callable[[str], None]], object]) -> None:
        super().__init__()
        self._fn = fn

    def run(self) -> None:  # corre en el hilo
        try:
            self.done.emit(self._fn(self.progress.emit))
        except Exception as e:  # noqa: BLE001 — todo error llega a la interfaz
            self.failed.emit(str(e) or repr(e))


class LibraryPanel(QWidget):
    def __init__(self, on_insert: Callable[[Path, library.Item], None],
                 on_tool: Callable[[Path, library.Item], None],
                 on_robot: Callable[[str], None], report: Callable[[str, str], None],
                 settings: QSettings) -> None:
        super().__init__()
        self._on_insert, self._on_tool, self._on_robot = on_insert, on_tool, on_robot
        self._report = report
        self._settings = settings
        self.fetch = library._http          # los tests lo reemplazan
        self.cache_dir = library.CACHE_DIR
        self.models_dir = USER_MODELS_DIR
        self.sync = False                   # tests: sin hilos
        self.catalog: library.Catalog | None = None
        self._task: _Task | None = None
        self._pending_done: Callable[[object], None] | None = None
        self._loaded_once = False

        self.folder_edit = QLineEdit(str(settings.value("biblioteca/carpeta", library.DEFAULT_FOLDER)))
        self.folder_edit.setToolTip("Enlace o ID de la carpeta de Google Drive (compartida como "
                                    "«Cualquier persona con el enlace»)")
        self.refresh_btn = QPushButton("Actualizar")
        self.refresh_btn.clicked.connect(self.refresh)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Modelo", "Tipo"])
        self.tree.header().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.tree.header().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.tree.itemSelectionChanged.connect(self._update_buttons)
        self.tree.itemDoubleClicked.connect(lambda *_: self.use_selected())
        self.insert_btn = QPushButton("Insertar en la celda")
        self.insert_btn.clicked.connect(lambda: self.use_selected("pieza"))
        self.tool_btn = QPushButton("Montar en la brida")
        self.tool_btn.clicked.connect(lambda: self.use_selected("herramienta"))
        self.robot_btn = QPushButton("Usar este robot")
        self.robot_btn.clicked.connect(lambda: self.use_selected("robot"))
        self.status = QLabel("La biblioteca se lee de Google Drive al abrir esta pestaña.")
        self.status.setWordWrap(True)

        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        row.addWidget(QLabel("Carpeta:"))
        row.addWidget(self.folder_edit, 1)
        row.addWidget(self.refresh_btn)
        layout.addLayout(row)
        layout.addWidget(self.tree, 1)
        for button in (self.insert_btn, self.tool_btn, self.robot_btn):
            layout.addWidget(button)
        layout.addWidget(self.status)
        note = QLabel("Se clasifica por carpeta: «Robots» (o un STEP que diga BRTIRUS) → robot; "
                      "«Herramientas», «Grippers», «Pinzas», «Antorchas» → herramienta; el resto, "
                      "piezas. Doble clic: lo que corresponda.")
        note.setWordWrap(True)
        layout.addWidget(note)
        self._update_buttons()

    # -- hilo ------------------------------------------------------------------------

    @property
    def busy(self) -> bool:
        return self._task is not None

    def _run(self, fn, on_done: Callable[[object], None], busy_text: str) -> None:
        if self.busy:
            self._report("warning", "La biblioteca está ocupada: esperá a que termine.")
            return
        self.status.setText(busy_text)
        self._set_enabled(False)
        if self.sync:
            try:
                result = fn(self.status.setText)
            except Exception as e:  # noqa: BLE001
                self._failed(str(e))
            else:
                self._finish()
                on_done(result)
            return
        task = _Task(fn)
        # Métodos de objetos del hilo de la interfaz (no lambdas): así Qt
        # entrega las señales en ESE hilo. Una lambda correría en el hilo de
        # trabajo, tocaría la interfaz desde ahí y se esperaría a sí misma.
        task.progress.connect(self.status.setText)
        task.done.connect(self._task_done)
        task.failed.connect(self._failed)
        self._pending_done = on_done
        self._task = task
        task.start()

    def _task_done(self, result) -> None:
        on_done, self._pending_done = self._pending_done, None
        self._finish()
        if on_done is not None:
            on_done(result)

    def _finish(self) -> None:
        if self._task is not None:
            self._task.wait()        # que termine de salir antes de soltarlo
            self._task = None
        self._set_enabled(True)

    def _failed(self, message: str) -> None:
        self._finish()
        self.status.setText(message)
        self._report("error", f"Biblioteca: {message}")

    def _set_enabled(self, enabled: bool) -> None:
        self.refresh_btn.setEnabled(enabled)
        self.folder_edit.setEnabled(enabled)
        self._update_buttons(enabled)

    def shutdown(self) -> None:
        if self._task is not None:
            self._task.wait(10000)
            self._task = None

    # -- listar ------------------------------------------------------------------------

    def ensure_loaded(self) -> None:
        """Primera vez que se abre la pestaña: leer la carpeta de la web."""
        if not self._loaded_once:
            self._loaded_once = True
            self.refresh()

    def refresh(self) -> None:
        try:
            fid = library.folder_id(self.folder_edit.text())
        except library.LibraryError as e:
            self.status.setText(str(e))
            return
        self._settings.setValue("biblioteca/carpeta", self.folder_edit.text().strip())
        fetch = self.fetch
        self._run(lambda _p: library.scan(fid, fetch), self._show, "Leyendo la carpeta de Drive…")

    def _show(self, catalog: library.Catalog) -> None:
        self.catalog = catalog
        self.tree.clear()
        groups: dict[str, QTreeWidgetItem] = {}
        for index, item in enumerate(catalog.items):
            category = item.category or "(sin carpeta)"
            if category not in groups:
                groups[category] = QTreeWidgetItem(self.tree, [category, ""])
                groups[category].setExpanded(True)
            child = QTreeWidgetItem(groups[category], [item.name, KIND_LABEL[item.kind]])
            child.setData(0, Qt.ItemDataRole.UserRole, index)
            child.setToolTip(0, f"{item.folder}/{item.name}" if item.folder else item.name)
        text = f"{len(catalog.items)} modelo(s) en {len(groups)} carpeta(s)."
        if catalog.unusable:
            text += (f" {len(catalog.unusable)} archivo(s) de CAD nativo (SolidWorks, Inventor…) "
                     f"no se pueden usar: exportalos a STEP.")
        self.status.setText(text)
        self._update_buttons()

    def fetch_by_id(self, drive_id: str, name: str) -> Path:
        """Bajar (o tomar de la caché) un archivo de la biblioteca por su ID,
        sin hilo: para abrir una celda armada en otra PC."""
        item = library.Item(drive_id, name, "", "", "pieza")
        return library.download(item, self.cache_dir, self.fetch)

    # -- usar -----------------------------------------------------------------------------

    def selected(self) -> library.Item | None:
        items = self.tree.selectedItems()
        if not items or self.catalog is None:
            return None
        index = items[0].data(0, Qt.ItemDataRole.UserRole)
        return None if index is None else self.catalog.items[int(index)]

    def _update_buttons(self, enabled: bool = True) -> None:
        item = self.selected() if enabled else None
        self.insert_btn.setEnabled(item is not None)
        self.tool_btn.setEnabled(item is not None)
        self.robot_btn.setEnabled(item is not None and item.kind == "robot")

    def use_selected(self, how: str | None = None) -> None:
        item = self.selected()
        if item is None:
            return
        how = how or item.kind
        fetch, cache = self.fetch, self.cache_dir
        if how == "robot":
            self._use_robot(item)
            return
        on_done = self._on_tool if how == "herramienta" else self._on_insert
        self._run(lambda _p: library.download(item, cache, fetch),
                  lambda path: (self.status.setText(f"«{item.name}» listo."), on_done(Path(path), item)),
                  f"Bajando «{item.name}»…")

    def _use_robot(self, item: library.Item) -> None:
        name = item.robot_name or Path(item.name).stem.split()[0]
        if name in RobotModel.available():
            mine = Path(self.models_dir) / f"{name}.json"
            again = (f" Para volver a importarlo, borrá {mine}." if mine.exists()
                     else " (Viene con la app.)")
            self.status.setText(f"{name} ya está instalado: se usa ese.{again}")
            self._on_robot(name)
            return
        fetch, cache, out = self.fetch, self.cache_dir, self.models_dir

        def work(progress):
            from sim.robot_import import import_robot_step

            progress(f"Bajando «{item.name}»…")
            path = library.download(item, cache, fetch)
            progress(f"Importando {name} (unos minutos)…")
            model_file, robot = import_robot_step(
                path, name, out, JOINTS_FROM,
                progress=lambda i, n, part: progress(f"Importando {name}: parte {i + 1} de {n} "
                                                     f"({part})"))
            return name, robot.reach_mm

        def done(result):
            robot_name, reach = result
            self.status.setText(f"{robot_name} importado: alcance medido {reach:.0f} mm. Rangos y "
                                f"velocidades copiados del {JOINTS_FROM} (confirmar).")
            self._report("warning", f"{robot_name}: rangos y velocidades copiados del "
                                    f"{JOINTS_FROM} hasta tener su tabla (hipótesis).")
            self._on_robot(robot_name)

        self._run(work, done, f"Preparando {name}…")
