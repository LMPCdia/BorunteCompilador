"""
Mallas de triángulos para el visor 3D: carga de STL/OBJ (Python puro) y de
STEP (con gmsh, que trae OpenCascade adentro), y primitivas para dibujar el
robot mientras no haya modelos 3D de sus eslabones.

Una `Mesh` es una lista plana de triángulos sin índices, con una normal por
triángulo (sombreado plano): es lo más simple de pasarle a la GPU y alcanza
para piezas mecánicas. Unidades: mm.
"""

from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass, field
from pathlib import Path

try:  # Opcional, como en sim/collision.py: sin numpy las mallas grandes tardan más.
    import numpy as np
except ImportError:  # pragma: no cover — depende de la instalación
    np = None

Vec = tuple[float, float, float]

# Una pieza de CAD convertida sin límite puede tener millones de triángulos;
# el visor no los necesita para ver un layout.
STEP_ELEMENTS_PER_CIRCLE = 16
MAX_SKIPPED_FACES = 50  # caras que gmsh no puede mallar y se dejan afuera

# Tope de triángulos de un STEP. Medido con una mesa de 1,5 m (MesaSanLorenzo,
# 2184 caras): con detalle mínimo = diagonal/2000 (~1 mm) dio 8,2 M de
# triángulos en 384 s y cerró el programa; con /200 (~10 mm), 205 mil en ~36 s.
# Se malla primero a /200 y se afina de a pasos x2 mientras la predicción
# entre en el tope; si ya se pasa, se engrosa.
STEP_MAX_TRIANGLES = 400_000
STEP_START_RATIO = 200.0
STEP_COARSEST_RATIO = 30.0  # más grueso no baja: manda MeshSizeMax y la curvatura
# Cuánto crecen los triángulos al afinar: en esa mesa, triángulos ~ relación^e
# con e entre 0,5 (muy grueso) y 1,7 (fino). Se predice con 2, que sobreestima: es
# preferible quedarse un paso más grueso que pasarse y tener que rehacer.
_REFINE_EXPONENT = 2.0


class MeshError(Exception):
    pass


def _sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _normalize(v: Vec) -> Vec:
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    return (0.0, 0.0, 1.0) if n < 1e-12 else (v[0] / n, v[1] / n, v[2] / n)


@dataclass
class Mesh:
    triangles: list[tuple[Vec, Vec, Vec]] = field(default_factory=list)
    skipped_faces: int = 0  # caras del STEP que gmsh no pudo mallar (ver load_step)
    # Detalle mínimo con que se malló un STEP (mm), y si se engrosó más de lo
    # pedido para no pasar de STEP_MAX_TRIANGLES. None si no viene de un STEP.
    detail_mm: float | None = None
    coarsened: bool = False

    def add(self, a: Vec, b: Vec, c: Vec) -> None:
        self.triangles.append((a, b, c))

    def extend(self, other: "Mesh") -> "Mesh":
        self.triangles.extend(other.triangles)
        return self

    def __len__(self) -> int:
        return len(self.triangles)

    def bounds(self) -> tuple[Vec, Vec]:
        if not self.triangles:
            raise MeshError("Malla vacía")
        pts = [p for tri in self.triangles for p in tri]
        return (
            (min(p[0] for p in pts), min(p[1] for p in pts), min(p[2] for p in pts)),
            (max(p[0] for p in pts), max(p[1] for p in pts), max(p[2] for p in pts)),
        )

    def interleaved(self) -> bytes:
        """Posición + normal por vértice, float32: lo que consume la GPU."""
        if np is not None and self.triangles:
            return _interleaved_numpy(self.triangles)
        out = bytearray()
        for a, b, c in self.triangles:
            n = _normalize(_cross(_sub(b, a), _sub(c, a)))
            for p in (a, b, c):
                out += struct.pack("<6f", *p, *n)
        return bytes(out)


def _interleaved_numpy(triangles: list[tuple[Vec, Vec, Vec]]) -> bytes:
    """Lo mismo que el bucle de `Mesh.interleaved`, byte a byte: la normal se
    calcula en float64 y recién después se pasa a float32, y una normal nula
    queda en (0, 0, 1) como en `_normalize`."""
    tri = np.asarray(triangles, dtype=np.float64).reshape(-1, 3, 3)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    n = np.cross(b - a, c - a)
    length = np.sqrt((n * n).sum(axis=1))
    flat = length < 1e-12
    n[flat] = (0.0, 0.0, 1.0)
    length[flat] = 1.0
    n /= length[:, None]
    out = np.empty((len(tri), 3, 6), dtype=np.float32)
    out[:, :, :3] = tri
    out[:, :, 3:] = n[:, None, :]
    return out.tobytes()


# --- carga de archivos ------------------------------------------------------------


def load_mesh(path: str | Path) -> Mesh:
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix == ".stl":
        mesh = load_stl(path)
    elif suffix == ".obj":
        mesh = load_obj(path)
    elif suffix in (".step", ".stp"):
        mesh = load_step(path)
    else:
        raise MeshError(f"Formato no soportado: {path.suffix} (se aceptan STEP, STL y OBJ)")
    if not mesh.triangles:
        raise MeshError(f"{path.name} no tiene triángulos")
    return mesh


def load_stl(path: Path) -> Mesh:
    data = path.read_bytes()
    # Binario: 80 de encabezado + cantidad + 50 bytes por triángulo. Un STL
    # binario puede empezar con "solid", así que se decide por el tamaño.
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + count * 50 == len(data):
            mesh = Mesh()
            for i in range(count):
                v = struct.unpack_from("<12f", data, 84 + i * 50)
                mesh.add(v[3:6], v[6:9], v[9:12])
            return mesh
    mesh, pending = Mesh(), []
    for line in data.decode("utf-8", errors="replace").splitlines():
        parts = line.split()
        if parts[:1] == ["vertex"]:
            pending.append(tuple(float(x) for x in parts[1:4]))
            if len(pending) == 3:
                mesh.add(*pending)
                pending = []
    return mesh


def load_obj(path: Path) -> Mesh:
    vertices: list[Vec] = []
    mesh = Mesh()
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v":
            vertices.append(tuple(float(x) for x in parts[1:4]))
        elif parts[0] == "f":
            idx = []
            for item in parts[1:]:
                i = int(item.split("/")[0])
                idx.append(i - 1 if i > 0 else len(vertices) + i)
            for k in range(1, len(idx) - 1):  # polígono -> abanico de triángulos
                mesh.add(vertices[idx[0]], vertices[idx[k]], vertices[idx[k + 1]])
    return mesh


def load_step(path: Path, elements_per_circle: int = STEP_ELEMENTS_PER_CIRCLE,
              min_size_ratio: float = 2000.0,
              max_triangles: int = STEP_MAX_TRIANGLES) -> Mesh:
    """STEP -> triángulos con gmsh. Unidades del archivo convertidas a mm.
    `elements_per_circle` y `min_size_ratio` (detalle más chico = diagonal /
    esto) bajan la cantidad de triángulos de piezas con mucho detalle.
    `min_size_ratio` es el detalle más fino que se quiere: se llega hasta ahí
    solo si entra en `max_triangles` (ver `_mesh_within_budget`)."""
    try:
        import gmsh
    except Exception as e:  # noqa: BLE001 — falta la librería o su DLL
        raise MeshError(f"No se puede leer STEP: gmsh no está disponible ({e})") from e

    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setString("Geometry.OCCTargetUnit", "MM")
        try:
            gmsh.model.occ.importShapes(str(path))
        except Exception as e:  # noqa: BLE001
            raise MeshError(f"No se pudo leer {path.name}: {e}") from e
        gmsh.model.occ.synchronize()
        xmin, ymin, zmin, xmax, ymax, zmax = gmsh.model.getBoundingBox(-1, -1)
        diag = math.dist((xmin, ymin, zmin), (xmax, ymax, zmax)) or 1.0
        # Malla de superficie gruesa, que sigue la curvatura: suficiente para ver.
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", elements_per_circle)
        gmsh.option.setNumber("Mesh.MeshSizeMax", diag / 15)
        ratio, skipped = _mesh_within_budget(gmsh, diag, min_size_ratio, max_triangles)

        tags, coords, _ = gmsh.model.mesh.getNodes()
        index = {int(t): i for i, t in enumerate(tags)}
        mesh = Mesh()
        types, _, node_lists = gmsh.model.mesh.getElements(2)
        for etype, nodes in zip(types, node_lists):
            if etype != 2:  # 2 = triángulo de 3 nodos
                continue
            for k in range(0, len(nodes), 3):
                pts = []
                for n in nodes[k:k + 3]:
                    i = index[int(n)] * 3
                    pts.append((coords[i], coords[i + 1], coords[i + 2]))
                mesh.add(*pts)
        mesh.skipped_faces = skipped
        mesh.detail_mm = diag / ratio
        mesh.coarsened = ratio < min_size_ratio
        return mesh
    finally:
        gmsh.finalize()


def _mesh_within_budget(gmsh, diag: float, finest_ratio: float,
                        max_triangles: int) -> tuple[float, int]:
    """Malla con el detalle más fino que entre en `max_triangles`, sin pasar de
    `finest_ratio`. No se puede saber cuántos triángulos da un detalle sin
    mallarlo, y mallar demasiado fino es justo lo que tarda minutos: por eso se
    arranca en STEP_START_RATIO y se afina de a x2 solo si la predicción
    (exponente _REFINE_EXPONENT, que sobreestima) entra. Si el primer mallado
    ya se pasa, se engrosa. Devuelve la relación usada y las caras salteadas.
    Un STEP mucho más pesado que la mesa medida puede tardar igual en el
    primer mallado: el tope evita que el resultado cierre el programa, no
    acota ese primer tiempo."""
    skipped = 0

    def mesh_at(ratio: float) -> int:
        nonlocal skipped
        gmsh.model.mesh.clear()
        gmsh.option.setNumber("Mesh.MeshSizeMin", diag / ratio)
        skipped += _generate_skipping_bad_faces(gmsh)
        types, _, node_lists = gmsh.model.mesh.getElements(2)
        return sum(len(nodes) // 3 for etype, nodes in zip(types, node_lists) if etype == 2)

    ratio = min(STEP_START_RATIO, finest_ratio)
    count = mesh_at(ratio)
    if count > max_triangles:
        # Achicar la relación en la misma proporción que sobran triángulos
        # engrosa de más (el exponente real es < 2), así que suele bastar una vez.
        while count > max_triangles and ratio > STEP_COARSEST_RATIO:
            ratio = max(STEP_COARSEST_RATIO, ratio * max_triangles / count * 0.8)
            count = mesh_at(ratio)
        return ratio, skipped
    while ratio < finest_ratio:
        finer = min(finest_ratio, ratio * 2)
        if count * (finer / ratio) ** _REFINE_EXPONENT > max_triangles:
            break
        finer_count = mesh_at(finer)
        if finer_count > max_triangles:  # la predicción se quedó corta: volver
            mesh_at(ratio)
            break
        ratio, count = finer, finer_count
    return ratio, skipped


def _generate_skipping_bad_faces(gmsh) -> int:
    """Malla de superficie. Si gmsh no puede con una cara (p. ej. "Impossible to
    mesh periodic surface 602", que pasa con CAD de fabricantes), la saca y
    sigue: para ver la pieza y buscar choques, una cara de menos es mucho
    mejor que no poder abrir el archivo. Devuelve cuántas caras se sacaron."""
    skipped = 0
    for _ in range(MAX_SKIPPED_FACES + 1):
        try:
            gmsh.model.mesh.generate(2)
            return skipped
        except Exception as e:  # noqa: BLE001 — gmsh tira Exception pelada
            m = re.search(r"surface (\d+)", str(e))
            if m is None or skipped >= MAX_SKIPPED_FACES:
                raise MeshError(f"gmsh no pudo mallar la pieza: {e}") from e
            # Sin borrar lo ya mallado: gmsh sigue con las caras pendientes.
            gmsh.model.removeEntities([(2, int(m.group(1)))])
            skipped += 1
    return skipped


def simplify(mesh: Mesh, cell_mm: float) -> Mesh:
    """Menos triángulos agrupando los vértices en una grilla de `cell_mm`
    (cada grupo pasa a su promedio; los triángulos que se achican a una línea
    o un punto se van). El error queda por debajo de la grilla: para ver un
    robot y buscar choques con margen, unos mm no cambian nada."""
    def key(p: Vec) -> tuple[int, int, int]:
        return (round(p[0] / cell_mm), round(p[1] / cell_mm), round(p[2] / cell_mm))

    sums: dict[tuple[int, int, int], list[float]] = {}
    for tri in mesh.triangles:
        for p in tri:
            s = sums.setdefault(key(p), [0.0, 0.0, 0.0, 0.0])
            s[0] += p[0]
            s[1] += p[1]
            s[2] += p[2]
            s[3] += 1
    rep = {k: (s[0] / s[3], s[1] / s[3], s[2] / s[3]) for k, s in sums.items()}
    out, seen = Mesh(skipped_faces=mesh.skipped_faces), set()
    for tri in mesh.triangles:
        keys = [key(p) for p in tri]
        face = tuple(sorted(keys))
        if len(set(keys)) < 3 or face in seen:
            continue
        seen.add(face)
        out.add(*(rep[k] for k in keys))
    return out


def write_stl(mesh: Mesh, path: str | Path) -> None:
    """STL binario (para cachear un STEP ya convertido)."""
    out = bytearray(b"borunte-dsl".ljust(80, b" "))
    out += struct.pack("<I", len(mesh))
    for a, b, c in mesh.triangles:
        n = _normalize(_cross(_sub(b, a), _sub(c, a)))
        out += struct.pack("<12fH", *n, *a, *b, *c, 0)
    Path(path).write_bytes(bytes(out))


# --- primitivas --------------------------------------------------------------------


def cylinder(p0: Vec, p1: Vec, radius: float, segments: int = 20) -> Mesh:
    """Cilindro cerrado entre dos puntos."""
    axis = _normalize(_sub(p1, p0))
    helper = (1.0, 0.0, 0.0) if abs(axis[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = _normalize(_cross(axis, helper))
    v = _cross(axis, u)

    def ring(center: Vec) -> list[Vec]:
        pts = []
        for i in range(segments):
            a = 2 * math.pi * i / segments
            c, s = math.cos(a) * radius, math.sin(a) * radius
            pts.append(tuple(center[k] + u[k] * c + v[k] * s for k in range(3)))
        return pts

    r0, r1 = ring(p0), ring(p1)
    mesh = Mesh()
    for i in range(segments):
        j = (i + 1) % segments
        mesh.add(r0[i], r0[j], r1[j])
        mesh.add(r0[i], r1[j], r1[i])
        mesh.add(p0, r0[j], r0[i])
        mesh.add(p1, r1[i], r1[j])
    return mesh


def box(center: Vec, size: Vec) -> Mesh:
    cx, cy, cz = center
    hx, hy, hz = (s / 2 for s in size)
    c = [(cx + sx * hx, cy + sy * hy, cz + sz * hz)
         for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    mesh = Mesh()
    for a, b, cc, d in faces:
        mesh.add(c[a], c[b], c[cc])
        mesh.add(c[a], c[cc], c[d])
    return mesh
