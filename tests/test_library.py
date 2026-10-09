"""Biblioteca de Google Drive (sim/library.py, gui/library_view.py), contra
un "Drive" falso que sirve páginas con el mismo formato que el real."""

import struct

import pytest

from sim import library
from sim.meshes import box, write_stl


def entry(entry_id: str, name: str, folder: bool) -> str:
    href = (f"https://drive.google.com/drive/folders/{entry_id}" if folder
            else f"https://drive.google.com/file/d/{entry_id}/view?usp=drive_web")
    return (f'<div class="flip-entry" id="entry-{entry_id}" tabindex="0" role="link">'
            f'<div class="flip-entry-info"><a href="{href}" target="_blank"><div '
            f'class="flip-entry-visual"></div><div class="flip-entry-title">{name}</div></a></div>'
            f'<div class="flip-entry-last-modified"><div>3/7/22</div></div></div>')


def page(*entries: str) -> bytes:
    return ("<html><body><div class='flip-entries'>" + "".join(entries) +
            "</div></body></html>").encode("utf-8")


class FakeDrive:
    """Carpetas y archivos por ID; cuenta las descargas."""

    def __init__(self, tmp_path):
        mesa = tmp_path / "src_mesa.stl"
        write_stl(box((0, 0, 375), (1000, 600, 750)), mesa)
        antorcha = tmp_path / "src_antorcha.stl"
        write_stl(box((0, 0, 100), (30, 30, 200)), antorcha)
        self.files = {"F_MESA": mesa.read_bytes(), "F_ANT": antorcha.read_bytes(),
                      "F_ROBOT": b"ISO-10303-21;"}
        self.modified = {k: "Fri, 09 Oct 2026 11:51:41 GMT" for k in self.files}
        self.folders = {
            "ROOT00000000": page(entry("D_ROBOTS0000", "Robots", True),
                                 entry("D_TOOLS00000", "Herramientas", True),
                                 entry("D_MESAS00000", "Mesas &amp; bases", True),
                                 entry("D_1510A00000", "1510A", True)),
            "D_ROBOTS0000": page(entry("F_OTRO", "otro.step", False)),
            "D_TOOLS00000": page(entry("F_ANT", "antorcha.stl", False)),
            "D_MESAS00000": page(entry("F_MESA", "mesa.stl", False),
                                 entry("F_SW", "mesa.SLDPRT", False)),
            "D_1510A00000": page(entry("F_ROBOT", "BRTIRUS1510A modelo.STEP", False),
                                 entry("ROOT00000000", "vuelta a la raíz", True)),
        }
        self.downloads = []
        self.online = True

    def __call__(self, url: str, method: str):
        if not self.online:
            raise library.LibraryError("sin red")
        if "embeddedfolderview" in url:
            fid = url.split("id=")[1]
            if fid not in self.folders:
                return b"<html>Necesitas permiso</html>", {"content-type": "text/html"}
            return self.folders[fid], {"content-type": "text/html"}
        fid = url.split("id=")[1].split("&")[0]
        if fid not in self.files:
            return b"<html>login</html>", {"content-type": "text/html; charset=utf-8"}
        body = self.files[fid]
        headers = {"content-type": "application/octet-stream", "content-length": str(len(body)),
                   "last-modified": self.modified[fid]}
        if method == "GET":
            self.downloads.append(fid)
        return (b"" if method == "HEAD" else body), headers


@pytest.fixture
def drive(tmp_path):
    return FakeDrive(tmp_path)


def test_folder_id_from_links():
    fid = "1BnYko-GDci726XcNMSW4Adn8om8zg6oO"
    assert library.folder_id(fid) == fid
    assert library.folder_id(f"https://drive.google.com/drive/folders/{fid}?usp=sharing") == fid
    assert library.folder_id(f"https://drive.google.com/embeddedfolderview?id={fid}") == fid
    with pytest.raises(library.LibraryError):
        library.folder_id("mi carpeta")


def test_scan_classifies_by_folder(drive):
    catalog = library.scan("ROOT00000000", drive)
    kinds = {i.name: (i.kind, i.category) for i in catalog.items}
    assert kinds["otro.step"] == ("robot", "Robots")
    assert kinds["antorcha.stl"] == ("herramienta", "Herramientas")
    assert kinds["mesa.stl"] == ("pieza", "Mesas & bases")            # entidades HTML
    assert kinds["BRTIRUS1510A modelo.STEP"] == ("robot", "1510A")     # por el nombre
    assert catalog.unusable == ["Mesas & bases/mesa.SLDPRT"]
    robot = next(i for i in catalog.items if i.kind == "robot" and i.category == "1510A")
    assert robot.robot_name == "BRTIRUS1510A"
    assert len(catalog.items) == 4                                     # la vuelta no repite


def test_folder_not_shared_is_explained(drive):
    with pytest.raises(library.LibraryError, match="Cualquier persona con el enlace"):
        library.scan("NOCOMPARTIDA0", drive)


def test_download_caches_and_refreshes_when_drive_changes(drive, tmp_path):
    item = next(i for i in library.scan("ROOT00000000", drive).items if i.name == "mesa.stl")
    cache = tmp_path / "cache"
    path = library.download(item, cache, drive)
    assert path.read_bytes() == drive.files["F_MESA"]
    library.download(item, cache, drive)
    assert drive.downloads == ["F_MESA"]                     # la segunda, de la caché
    drive.files["F_MESA"] += struct.pack("<I", 0)            # alguien la actualizó
    library.download(item, cache, drive)
    assert drive.downloads == ["F_MESA", "F_MESA"]
    drive.online = False
    assert library.download(item, cache, drive) == path       # sin red: lo bajado


def test_download_of_a_private_file_is_explained(drive, tmp_path):
    item = library.Item("PRIVADO", "x.step", "", "", "pieza")
    with pytest.raises(library.LibraryError, match="Cualquier persona con el enlace"):
        library.download(item, tmp_path, drive)


# --- pestaña Biblioteca ----------------------------------------------------------------------


@pytest.fixture
def view(drive, tmp_path):
    from PySide6.QtWidgets import QApplication

    QApplication.instance() or QApplication([])
    from gui.sim_view import SimView

    v = SimView(lambda: "", enable_3d=False)
    v.library.fetch = drive
    v.library.cache_dir = tmp_path / "cache"
    v.library.models_dir = tmp_path / "modelos"
    v.library.sync = True
    v.library.folder_edit.setText("https://drive.google.com/drive/folders/ROOT00000000")
    return v


def _select(v, name):
    for top in range(v.library.tree.topLevelItemCount()):
        group = v.library.tree.topLevelItem(top)
        for k in range(group.childCount()):
            if group.child(k).text(0) == name:
                v.library.tree.setCurrentItem(group.child(k))
                return group.child(k)
    raise AssertionError(name)


def test_opening_the_tab_reads_the_folder(view):
    view.side_tabs.setCurrentIndex(view.library_tab_index)
    tree = view.library.tree
    groups = [tree.topLevelItem(i).text(0) for i in range(tree.topLevelItemCount())]
    assert groups == ["Robots", "Herramientas", "Mesas & bases", "1510A"]
    assert "1 archivo(s) de CAD nativo" in view.library.status.text()


def test_insert_a_piece_and_mount_a_tool(view, tmp_path):
    view.library.refresh()
    _select(view, "mesa.stl")
    assert not view.library.robot_btn.isEnabled()
    view.library.use_selected()                     # doble clic: pieza -> a la celda
    [obj] = view.layout_data.objects
    assert obj.name == "mesa" and obj.drive_id == "F_MESA"
    assert view.selected_piece() == 0               # lista para ubicar
    _select(view, "antorcha.stl")
    view.library.use_selected()                     # herramienta -> a la brida
    assert view.layout_data.tool_mesh.endswith("antorcha.stl")
    assert view.layout_data.tool_drive_id == "F_ANT"


def test_a_cell_from_another_pc_downloads_its_library_pieces(view, drive, tmp_path):
    import shutil

    view.library.refresh()
    _select(view, "mesa.stl")
    view.library.use_selected()
    _select(view, "antorcha.stl")
    view.library.use_selected()
    cell = tmp_path / "celda.layout.json"
    view.save_layout(cell)
    shutil.rmtree(tmp_path / "cache")               # "otra PC": sin la caché
    before = len(drive.downloads)
    assert view.load_layout(cell)
    assert len(drive.downloads) == before + 2
    assert (tmp_path / "cache" / "F_MESA").exists()
    assert view.layout_data.objects[0].drive_id == "F_MESA"


def test_robot_already_installed_is_just_selected(view):
    view.library.refresh()
    _select(view, "BRTIRUS1510A modelo.STEP")
    assert view.library.robot_btn.isEnabled()
    view.model_combo.setCurrentText("BRTIRUS1820A")
    view.library.use_selected()
    assert view.model.name == "BRTIRUS1510A"
    assert "ya está instalado" in view.library.status.text()


def test_a_new_robot_is_imported_and_selected(view, tmp_path, monkeypatch):
    import json
    import shutil

    import sim.kinematics as kinematics
    import sim.robot_import as robot_import

    models = tmp_path / "modelos"
    models.mkdir()
    monkeypatch.setattr(kinematics, "USER_MODELS_DIR", models)
    view.library.models_dir = models

    def fake_import(step, name, out_dir, joints_from, progress=None):
        # Lo que dejaría la importación real (que tarda minutos): un modelo más.
        data = json.loads((kinematics.MODELS_DIR / "BRTIRUS1820A.json").read_text(encoding="utf-8"))
        data["name"] = name
        (out_dir / f"{name}.json").write_text(json.dumps(data), encoding="utf-8")

        class R:
            reach_mm = 1731.5
        return out_dir / f"{name}.json", R()

    monkeypatch.setattr(robot_import, "import_robot_step", fake_import)
    view.library.refresh()
    item = _select(view, "otro.step")
    del item
    view.library.catalog.items[[i.name for i in view.library.catalog.items].index("otro.step")] = \
        library.Item("F_ROBOT", "BRTIRUS2010A.step", "Robots", "Robots", "robot")
    view.library.use_selected()
    assert view.model.name == "BRTIRUS2010A"
    assert "importado" in view.library.status.text()
    shutil.rmtree(models)


def test_errors_reach_the_user(view):
    reports = []
    view.library._report = lambda s, m: reports.append((s, m))
    view.library.folder_edit.setText("NOCOMPARTIDA0")
    view.library.refresh()
    assert reports and reports[-1][0] == "error"
    assert view.library.refresh_btn.isEnabled()      # no queda trabada


def test_refresh_in_a_background_thread(view):
    import time

    from PySide6.QtWidgets import QApplication

    view.library.sync = False
    view.library.refresh()
    deadline = time.time() + 10
    while view.library.busy and time.time() < deadline:
        QApplication.processEvents()
        time.sleep(0.01)
    QApplication.processEvents()
    assert not view.library.busy
    assert view.library.tree.topLevelItemCount() == 4
    assert view.library.refresh_btn.isEnabled()
