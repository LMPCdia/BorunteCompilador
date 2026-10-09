"""
Biblioteca de modelos 3D en una carpeta de Google Drive.

La carpeta se lee EN LÍNEA cada vez (la vista web pública de una carpeta
compartida "cualquier persona con el enlace"), sin cuenta de Google ni
claves: lo que se suba ahí aparece en todas las PC. Los archivos se bajan
cuando se usan y quedan en una caché local (`~/BorunteDSL/biblioteca`), que
se vuelve a bajar solo si en Drive cambió (tamaño o fecha).

Clasificación: por la CARPETA, no por el nombre del archivo.

    RobotsBoruntesSimulador/
    ├── Robots/1510A/…STEP          -> robot
    ├── Herramientas/antorcha.step  -> herramienta (también Grippers, Pinzas…)
    ├── Bases/pedestal.step         -> pieza, categoría "Bases"
    └── 1510A/…STEP                 -> robot igual: el nombre dice BRTIRUS

Solo se listan formatos que el simulador lee (STEP, STL, OBJ); los de
SolidWorks/Inventor se cuentan aparte para avisar.

Límite conocido: la vista web de Drive muestra hasta unos cientos de
archivos por carpeta. Si una carpeta crece más, conviene dividirla.
"""

from __future__ import annotations

import html
import json
import re
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

DEFAULT_FOLDER = "1BnYko-GDci726XcNMSW4Adn8om8zg6oO"   # RobotsBoruntesSimulador
CACHE_DIR = Path.home() / "BorunteDSL" / "biblioteca"
FOLDER_URL = "https://drive.google.com/embeddedfolderview?id={id}"
DOWNLOAD_URL = "https://drive.usercontent.google.com/download?id={id}&export=download&confirm=t"
USABLE = (".step", ".stp", ".stl", ".obj")
# CAD que el simulador no lee: se cuentan para avisar que hay que exportarlos.
# Lo demás (un LEEME, fotos, planos en PDF) se ignora sin decir nada.
NATIVE_CAD = (".sldprt", ".sldasm", ".slddrw", ".ipt", ".iam", ".idw", ".x_t", ".x_b",
              ".igs", ".iges", ".dwg", ".dxf", ".3dm", ".prt", ".asm", ".catpart", ".catproduct",
              ".f3d", ".sat", ".jt", ".3mf", ".ply", ".fbx")
MAX_DEPTH = 5
TIMEOUT_S = 30

ROBOT_WORDS = ("robot",)
TOOL_WORDS = ("herramienta", "gripper", "pinza", "antorcha", "torcha", "tool", "efector",
              "ventosa", "garra")

_ENTRY = re.compile(
    r'<div class="flip-entry" id="entry-([^"]+)".*?<a href="([^"]+)".*?'
    r'flip-entry-title">([^<]*)<', re.S)


class LibraryError(Exception):
    pass


@dataclass(frozen=True)
class Entry:
    id: str
    name: str
    is_folder: bool


@dataclass(frozen=True)
class Item:
    file_id: str
    name: str
    category: str           # carpeta de primer nivel ("" = en la raíz)
    folder: str             # ruta de carpetas dentro de la biblioteca
    kind: str               # "robot" | "herramienta" | "pieza"

    @property
    def robot_name(self) -> str | None:
        """BRTIRUS1510A, si el nombre del archivo lo dice."""
        m = re.search(r"BRTIRUS\d+[A-Z]?", self.name.upper())
        return m.group(0) if m else None


@dataclass
class Catalog:
    items: list[Item]
    unusable: list[str]     # archivos que el simulador no lee (SLDPRT, IPT…)

    def categories(self) -> list[str]:
        return sorted({i.category for i in self.items}, key=lambda c: (c == "", c.lower()))


# --- red ---------------------------------------------------------------------------------


def _http(url: str, method: str = "GET") -> tuple[bytes, dict[str, str]]:
    request = urllib.request.Request(url, method=method,
                                     headers={"User-Agent": "BorunteDSL (biblioteca)"})
    try:
        with urllib.request.urlopen(request, timeout=TIMEOUT_S) as response:
            headers = {k.lower(): v for k, v in response.headers.items()}
            return (b"" if method == "HEAD" else response.read()), headers
    except OSError as e:  # URLError, timeout, sin red
        raise LibraryError(f"No se pudo conectar con Google Drive: {e}") from e


Fetch = Callable[[str, str], tuple[bytes, dict[str, str]]]


def folder_id(text: str) -> str:
    """Acepta el ID o el enlace de la carpeta tal como lo copia Drive."""
    text = text.strip()
    m = re.search(r"/folders/([A-Za-z0-9_-]{10,})", text) or re.search(r"[?&]id=([A-Za-z0-9_-]{10,})", text)
    if m:
        return m.group(1)
    if re.fullmatch(r"[A-Za-z0-9_-]{10,}", text):
        return text
    raise LibraryError(f"«{text}» no parece un enlace ni un ID de carpeta de Google Drive")


def parse_folder(page: str) -> list[Entry]:
    out = []
    for m in _ENTRY.finditer(page):
        entry_id, href, name = m.group(1), m.group(2), html.unescape(m.group(3)).strip()
        out.append(Entry(entry_id, name, "/folders/" in href))
    return out


def list_folder(fid: str, fetch: Fetch = _http) -> list[Entry]:
    body, _headers = fetch(FOLDER_URL.format(id=fid), "GET")
    page = body.decode("utf-8", "replace")
    # Una carpeta vacía igual trae el contenedor "flip-entries"; lo que no
    # lo trae es la página de "pedí acceso" de una carpeta no compartida.
    if "flip-entries" not in page and "flip-entry" not in page:
        raise LibraryError("Drive no mostró la carpeta. ¿Está compartida como «Cualquier persona "
                           "con el enlace»?")
    return parse_folder(page)


def classify(category: str, folder: str, name: str) -> str:
    where = f"{category} {folder}".lower()
    if "BRTIRUS" in name.upper() or any(w in where for w in ROBOT_WORDS):
        return "robot"
    if any(w in where for w in TOOL_WORDS):
        return "herramienta"
    return "pieza"


def scan(root: str, fetch: Fetch = _http) -> Catalog:
    """Recorre la carpeta y sus subcarpetas (hasta MAX_DEPTH niveles)."""
    items, unusable = [], []
    pending = [(folder_id(root), [], 0)]
    seen = set()
    while pending:
        fid, path, depth = pending.pop(0)
        if fid in seen:
            continue
        seen.add(fid)
        for entry in list_folder(fid, fetch):
            if entry.is_folder:
                if depth < MAX_DEPTH:
                    pending.append((entry.id, path + [entry.name], depth + 1))
                continue
            if not entry.name.lower().endswith(USABLE):
                if entry.name.lower().endswith(NATIVE_CAD):
                    unusable.append("/".join(path + [entry.name]))
                continue
            category = path[0] if path else ""
            folder = "/".join(path)
            items.append(Item(entry.id, entry.name, category, folder,
                              classify(category, folder, entry.name)))
    return Catalog(items, unusable)


# --- descarga con caché --------------------------------------------------------------------


def cached_path(item: Item, cache_dir: Path = CACHE_DIR) -> Path:
    safe = re.sub(r'[<>:"/\\|?*]', "_", item.name)
    return cache_dir / item.file_id / safe


def download(item: Item, cache_dir: Path = CACHE_DIR, fetch: Fetch = _http,
             check_remote: bool = True) -> Path:
    """Archivo local del ítem: lo baja si no está o si cambió en Drive."""
    path = cached_path(item, cache_dir)
    meta_path = path.parent / "meta.json"
    url = DOWNLOAD_URL.format(id=item.file_id)
    if path.exists() and meta_path.exists():
        if not check_remote:
            return path
        try:
            _b, headers = fetch(url, "HEAD")
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            if (headers.get("content-length") == meta.get("content-length")
                    and headers.get("last-modified") == meta.get("last-modified")):
                return path
        except (LibraryError, ValueError):
            return path  # sin red: sirve lo que ya está bajado
    body, headers = fetch(url, "GET")
    if "text/html" in headers.get("content-type", ""):
        raise LibraryError(f"Drive no dejó bajar «{item.name}»: ¿está compartido como "
                           f"«Cualquier persona con el enlace»?")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".parcial")
    tmp.write_bytes(body)
    tmp.replace(path)
    meta = {"content-length": headers.get("content-length", str(len(body))),
            "last-modified": headers.get("last-modified", ""), "name": item.name}
    meta_path.write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    return path


# --- revisar la carpeta (línea de comandos) --------------------------------------------------


def problems(catalog: Catalog) -> list[str]:
    """Lo que está fuera de lugar según la estructura acordada (ver el LEEME
    de la carpeta y docs/SIMULATOR.md)."""
    out = []
    for item in catalog.items:
        where = f"{item.folder}/{item.name}" if item.folder else item.name
        if not item.category:
            out.append(f"«{where}» está suelto en la raíz: moverlo a la carpeta de su categoría")
        if item.kind == "robot" and item.category.lower() != "robots":
            out.append(f"«{where}» es un robot: va en Robots/{item.robot_name or '<MODELO>'}/")
        if item.kind == "robot" and not item.robot_name:
            out.append(f"«{where}» está en Robots pero el nombre no dice el modelo (BRTIRUSxxxxA)")
    # CAD nativo al lado de un STEP es una copia de referencia (como las piezas
    # de SolidWorks junto al ensamble del robot): solo molesta si en esa
    # carpeta no hay nada que el simulador pueda usar.
    usable_folders = {item.folder for item in catalog.items}
    for path in catalog.unusable:
        folder = path.rsplit("/", 1)[0] if "/" in path else ""
        if folder not in usable_folders:
            out.append(f"«{path}» es CAD nativo: exportarlo a STEP")
    return out


def main(argv: list[str] | None = None, fetch: Fetch = _http) -> int:
    """python -m sim.library [CARPETA]: qué hay en la biblioteca y qué está mal ubicado."""
    import sys

    args = sys.argv[1:] if argv is None else argv
    try:
        catalog = scan(args[0] if args else DEFAULT_FOLDER, fetch)
    except LibraryError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2
    for category in catalog.categories():
        print(f"{category or '(raíz)'}/")
        for item in catalog.items:
            if item.category == category:
                sub = item.folder[len(category):].strip("/")
                print(f"  [{item.kind}] {sub + '/' if sub else ''}{item.name}")
    issues = problems(catalog)
    print(f"\n{len(catalog.items)} modelo(s); {len(issues)} cosa(s) para acomodar")
    for issue in issues:
        print(f"  - {issue}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
