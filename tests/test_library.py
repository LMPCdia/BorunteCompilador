"""Biblioteca de Google Drive (sim/library.py, gui/library_view.py), contra
un "Drive" falso que sirve páginas con el mismo formato que el real."""

import struct

import pytest

from sim import library
from sim.meshes import box, write_stl


def entry(entry_id: str, name: str, folder: bool, sheet: bool = False) -> str:
    href = (f"https://drive.google.com/drive/folders/{entry_id}" if folder
            else f"https://docs.google.com/spreadsheets/d/{entry_id}/edit?usp=drive_web" if sheet
            else f"https://drive.google.com/file/d/{entry_id}/view?usp=drive_web")
    return (f'<div class="flip-entry" id="entry-{entry_id}" tabindex="0" role="link">'
            f'<div class="flip-entry-info"><a href="{href}" target="_blank"><div '
            f'class="flip-entry-visual"></div><div class="flip-entry-title">{name}</div></a></div>'
            f'<div class="flip-entry-last-modified"><div>3/7/22</div></div></div>')


def page(*entries: str) -> bytes:
    return ('<html><body><div class="flip-entries">' + "".join(entries) +
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
            "D_ROBOTS0000": page(entry("F_OTRO", "otro.step", False),
                                 entry("D_VACIA00000", "carpeta vacía", True)),
            "D_VACIA00000": page(),
            "D_TOOLS00000": page(entry("F_ANT", "antorcha.stl", False),
                                 entry("D_GRIP000000", "Grippers", True)),
            "D_GRIP000000": page(entry("F_IPT", "gripper.ipt", False)),
            "D_MESAS00000": page(entry("F_MESA", "mesa.stl", False),
                                 entry("F_SW", "mesa.SLDPRT", False),
                                 entry("F_DOC", "LEEME - cómo cargar modelos", False),
                                 entry("F_PDF", "plano mesa.pdf", False)),
            "D_1510A00000": page(entry("F_ROBOT", "BRTIRUS1510A modelo.STEP", False),
                                 entry("S_PARAMS", "Parámetros BRTIRUS1510A", False, sheet=True),
                                 entry("F_PDF1510", "BRTIRUS1510A datasheet.pdf", False),
                                 entry("ROOT00000000", "vuelta a la raíz", True)),
        }
        from tests.test_robot_params import SHEET

        self.sheets = {"S_PARAMS": SHEET}
        self.downloads = []
        self.online = True

    def __call__(self, url: str, method: str):
        if not self.online:
            raise library.LibraryError("sin red")
        if "/spreadsheets/d/" in url:
            sid = url.split("/spreadsheets/d/")[1].split("/")[0]
            if sid not in self.sheets:
                return b"<html>login</html>", {"content-type": "text/html"}
            return self.sheets[sid].encode("utf-8"), {"content-type": "text/csv; charset=utf-8"}
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
    assert sorted(catalog.unusable) == ["Herramientas/Grippers/gripper.ipt", "Mesas & bases/mesa.SLDPRT"]
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
    assert "2 archivo(s) de CAD nativo" in view.library.status.text()


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


def test_robot_from_the_app_gets_the_sheet_without_reimporting(view, tmp_path, monkeypatch):
    import sim.kinematics as kinematics

    models = tmp_path / "modelos"
    monkeypatch.setattr(kinematics, "USER_MODELS_DIR", models)
    view.library.models_dir = models
    view.library.refresh()
    robot = next(i for i in view.library.catalog.items if i.name.startswith("BRTIRUS1510A"))
    assert (robot.params_kind, robot.params_name) == ("sheet", "Parámetros BRTIRUS1510A")
    _select(view, "BRTIRUS1510A modelo.STEP")
    assert view.library.robot_btn.isEnabled()
    view.model_combo.setCurrentText("BRTIRUS1820A")
    view.library.use_selected()
    assert view.model.name == "BRTIRUS1510A"
    assert view.model.has_accelerations and view.model.max_linear_speed_mms == 1800
    assert "F_ROBOT" not in view.library.fetch.downloads          # no bajó el STEP
    text = view.library.status.text()
    assert "ejes de «Parámetros BRTIRUS1510A»" in text and "60/60 poses" in text
    assert (models / "BRTIRUS1510A.json").exists()                 # override del usuario
    assert view.accel_auto.isEnabled()                             # aceleraciones del datasheet


def test_a_broken_sheet_is_explained(view):
    reports = []
    view.library._report = lambda s, m: reports.append((s, m))
    view.library.fetch.sheets["S_PARAMS"] = "Eje,Mínimo,Máximo,Velocidad\nJ1,1,0,10\n"
    view.library.refresh()
    _select(view, "BRTIRUS1510A modelo.STEP")
    view.library.use_selected()
    assert reports[-1][0] == "error"
    assert "el mínimo (1) no es menor que el máximo (0)" in reports[-1][1]
    assert "faltan los ejes J2" in reports[-1][1]


def test_a_new_robot_is_imported_once_and_again_only_if_the_step_changes(view, tmp_path,
                                                                         monkeypatch):
    import json

    import sim.kinematics as kinematics
    import sim.robot_import as robot_import

    models = tmp_path / "modelos"
    models.mkdir()
    monkeypatch.setattr(kinematics, "USER_MODELS_DIR", models)
    view.library.models_dir = models
    imports = []

    def fake_import(step, name, out_dir, joints_from, progress=None):
        # Lo que dejaría la importación real (que tarda minutos): un modelo más.
        imports.append(name)
        data = json.loads((kinematics.MODELS_DIR / "BRTIRUS1820A.json").read_text(encoding="utf-8"))
        data["name"] = name
        (out_dir / f"{name}.json").write_text(json.dumps(data), encoding="utf-8")

        class R:
            reach_mm = 1731.5
        return out_dir / f"{name}.json", R()

    monkeypatch.setattr(robot_import, "import_robot_step", fake_import)
    view.library.refresh()
    _select(view, "otro.step")
    new = library.Item("F_ROBOT", "BRTIRUS2010A.step", "Robots", "Robots", "robot",
                       params_id="S_PARAMS", params_kind="sheet", params_name="Parámetros")
    view.library.catalog.items[[i.name for i in view.library.catalog.items].index("otro.step")] = new

    view.library.use_selected()
    assert imports == ["BRTIRUS2010A"]
    assert view.model.name == "BRTIRUS2010A" and view.model.has_accelerations
    assert "CAD importado" in view.library.status.text()
    saved = json.loads((models / "BRTIRUS2010A.json").read_text(encoding="utf-8"))
    assert saved["cad"]["drive_id"] == "F_ROBOT" and saved["joints"][0]["max_accel_dps2"] == 450

    view.library.use_selected()                       # otra vez: solo relee la planilla
    assert imports == ["BRTIRUS2010A"]
    view.library.fetch.files["F_ROBOT"] += b" "        # subieron otro STEP
    view.library.use_selected()
    assert imports == ["BRTIRUS2010A", "BRTIRUS2010A"]


def test_robot_without_sheet_warns(view, tmp_path, monkeypatch):
    import json

    import sim.kinematics as kinematics
    import sim.robot_import as robot_import

    models = tmp_path / "modelos"
    models.mkdir()
    monkeypatch.setattr(kinematics, "USER_MODELS_DIR", models)
    view.library.models_dir = models
    reports = []
    view.library._report = lambda s, m: reports.append((s, m))

    def fake_import(step, name, out_dir, joints_from, progress=None):
        data = json.loads((kinematics.MODELS_DIR / "BRTIRUS1820A.json").read_text(encoding="utf-8"))
        (out_dir / f"{name}.json").write_text(json.dumps(data), encoding="utf-8")
        return out_dir / f"{name}.json", None

    monkeypatch.setattr(robot_import, "import_robot_step", fake_import)
    view.library.refresh()
    _select(view, "otro.step")
    view.library.catalog.items[[i.name for i in view.library.catalog.items].index("otro.step")] = \
        library.Item("F_ROBOT", "BRTIRUS2010A.step", "Robots", "Robots", "robot")
    view.library.use_selected()
    assert view.model.name == "BRTIRUS2010A"
    assert any("sin planilla de parámetros" in m for _s, m in reports)


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


def test_command_line_lists_and_points_out_what_to_fix(drive, capsys):
    assert library.main(["ROOT00000000"], fetch=drive) == 0
    out = capsys.readouterr().out
    assert "Herramientas/\n  [herramienta] antorcha.stl" in out
    assert "«1510A/BRTIRUS1510A modelo.STEP» es un robot: va en Robots/BRTIRUS1510A/" in out
    assert "«Robots/otro.step» está en Robots pero el nombre no dice el modelo" in out
    assert "«Herramientas/Grippers/gripper.ipt» es CAD nativo: exportarlo a STEP" in out
    assert "mesa.SLDPRT" not in out          # al lado de su STL: es una referencia
    assert library.main(["NOCOMPARTIDA0"], fetch=drive) == 2


def test_datasheet_pdf_is_found_and_reported(drive, view, capsys, monkeypatch):
    catalog = library.scan("ROOT00000000", drive)
    robot = next(i for i in catalog.items if i.robot_name == "BRTIRUS1510A")
    assert robot.datasheet_name == "BRTIRUS1510A datasheet.pdf"
    assert robot.datasheet_url == "https://drive.google.com/file/d/F_PDF1510/view"
    issues = library.problems(catalog)
    assert any("otro.step" in i and "falta el datasheet en PDF" in i for i in issues)
    # PDF sin planilla: se pide armarla desde el PDF.
    drive.folders["D_ROBOTS0000"] = page(entry("F_OTRO", "BRTIRUS2010A.step", False),
                                         entry("F_PDF2010", "BRTIRUS2010A.pdf", False))
    issues = library.problems(library.scan("ROOT00000000", drive))
    assert any("pedirle a Claude que la arme desde el PDF" in i for i in issues)

    opened = []
    import PySide6.QtGui as gui
    monkeypatch.setattr(gui.QDesktopServices, "openUrl", lambda url: opened.append(url.toString()) or True)
    view.library.refresh()
    item = _select(view, "BRTIRUS1510A modelo.STEP")
    assert item.text(1) == "Robot (planilla, PDF)"
    assert view.library.datasheet_btn.isEnabled()
    assert view.library.open_datasheet()
    assert opened == ["https://drive.google.com/file/d/F_PDF1510/view"]


def test_pdf_without_sheet_is_pending_even_without_step(drive, capsys):
    import json as _json

    drive.folders["D_ROBOTS0000"] = page(entry("D_2010000000", "BRTIRUS2010A", True))
    drive.folders["D_2010000000"] = page(entry("F_PDF2010", "Datasheet 2010.pdf", False))
    catalog = library.scan("ROOT00000000", drive)
    [pending] = catalog.pending
    assert (pending.model, pending.folder, pending.folder_id) == (
        "BRTIRUS2010A", "Robots/BRTIRUS2010A", "D_2010000000")
    assert pending.pdf_id == "F_PDF2010" and not pending.has_step
    assert library.main(["ROOT00000000", "--pendientes"], fetch=drive) == 0
    listed = _json.loads(capsys.readouterr().out)
    assert listed[0]["pdf_name"] == "Datasheet 2010.pdf"
    # Con la planilla ya hecha, deja de estar pendiente.
    drive.folders["D_2010000000"] = page(entry("F_PDF2010", "Datasheet 2010.pdf", False),
                                         entry("S_P2010", "Parámetros BRTIRUS2010A", False, sheet=True))
    assert library.scan("ROOT00000000", drive).pending == []


def test_sheet_not_yet_reviewed_against_its_pdf_is_pending(drive, capsys):
    import json as _json

    assert library.main(["ROOT00000000", "--pendientes"], fetch=drive) == 0
    [review] = _json.loads(capsys.readouterr().out)
    assert review["action"] == "revisar" and review["model"] == "BRTIRUS1510A"
    assert review["params_id"] == "S_PARAMS" and review["pdf_id"] == "F_PDF1510"
    assert review["folder_id"] == "D_1510A00000"
    # Una vez revisada (la fila guarda el ID del PDF), no vuelve a salir.
    drive.sheets["S_PARAMS"] += "Datasheet revisado,F_PDF1510,,,\n"
    assert library.main(["ROOT00000000", "--pendientes"], fetch=drive) == 0
    assert _json.loads(capsys.readouterr().out) == []
    from sim.robot_params import parse_params
    parse_params(drive.sheets["S_PARAMS"])        # la fila nueva no rompe la planilla
