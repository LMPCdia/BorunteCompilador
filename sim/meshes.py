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
import struct
from dataclasses import dataclass, field
from pathlib import Path

Vec = tuple[float, float, float]

# Una pieza de CAD convertida sin límite puede tener millones de triángulos;
# el visor no los necesita para ver un layout.
STEP_ELEMENTS_PER_CIRCLE = 16


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
        out = bytearray()
        for a, b, c in self.triangles:
            n = _normalize(_cross(_sub(b, a), _sub(c, a)))
            for p in (a, b, c):
                out += struct.pack("<6f", *p, *n)
        return bytes(out)


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


def load_step(path: Path) -> Mesh:
    """STEP -> triángulos con gmsh. Unidades del archivo convertidas a mm."""
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
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", STEP_ELEMENTS_PER_CIRCLE)
        gmsh.option.setNumber("Mesh.MeshSizeMax", diag / 15)
        gmsh.option.setNumber("Mesh.MeshSizeMin", diag / 2000)
        gmsh.model.mesh.generate(2)

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
        return mesh
    finally:
        gmsh.finalize()


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
