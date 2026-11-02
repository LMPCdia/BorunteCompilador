"""
Fresado con el robot: de una pieza CAD y un área 2D a un programa .src + .dat.

Ver docs/FRESADO.md. El camino completo:

1. `build_heightmap`: la pieza (STEP/STL/OBJ ya mallada, cualquier cantidad de
   polígonos) vista desde una dirección se convierte en un mapa de alturas.
2. El usuario delimita áreas en ese plano 2D (`Polygon`, en mm de la vista).
3. `compute_path`: pasadas dentro del área "extrapoladas" a la superficie. La
   altura sale de pasar la forma de la fresa por el mapa (`_dilate`, lo que en
   CAM se llama *drop cutter*): la herramienta apoya sin clavarse, tenga la
   pieza los polígonos que tenga.
4. `solve_robot`: de cada punto a una pose del robot, verificando cinemática
   inversa y límites. El giro alrededor del eje de la fresa es libre (la fresa
   gira igual), y se usa para elegir la postura con más margen.
5. `generate_program`: el programa en nuestro lenguaje. Si un punto no se
   alcanza NO hay programa (`CamError`): el robot no se manda a "lo más cerca".

El mapa de alturas ve la cara de arriba de la pieza: lo que queda debajo de un
voladizo no se mecaniza desde esa vista. Para eso están las otras vistas.

Hipótesis (nada de esto se probó en un robot real):
- `SPEED` del lenguaje es un % (`docs/LENGUAJE.md`): acá no se convierte a mm/s
  porque los modelos no traen la velocidad lineal a 100 % confirmada.
- +Z de la herramienta es el eje de la fresa, saliendo de la brida (es como lo
  modela `RobotModel._home_flange`), y `pose_matrix` es la convención del pad.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Sequence

import numpy as np

from compiler.program_files import CONFIG_DAT, DAT_SUFFIX, SRC_SUFFIX, ProgramSources, SourceFile
from sim.kinematics import RobotModel, matrix_to_pose, pose_matrix
from sim.meshes import Mesh

Polygon = list[tuple[float, float]]

# Vistas: filas = ejes u, v, w en coordenadas de la celda. w mira al observador
# (la fresa baja en -w) y u x v = w.
VIEWS: dict[str, tuple[tuple[float, float, float], ...]] = {
    "Arriba": ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
    "Frente (+X)": ((0, 1, 0), (0, 0, 1), (1, 0, 0)),
    "Atrás (−X)": ((0, -1, 0), (0, 0, 1), (-1, 0, 0)),
    "Izquierda (+Y)": ((-1, 0, 0), (0, 0, 1), (0, 1, 0)),
    "Derecha (−Y)": ((1, 0, 0), (0, 0, 1), (0, -1, 0)),
}

MAX_CELLS = 6_000_000      # tope del mapa de alturas: se engrosa antes que colgar la PC
MAX_FOOTPRINT_MM = 40      # celdas de radio de la porta: arriba de eso se agrupa el mapa
PAIRS_PER_CHUNK = 4_000_000
UNREACHABLE_LISTED = 5     # cuántos puntos sin alcanzar se nombran en el error


class CamError(Exception):
    """Lo que el usuario pidió no se puede hacer: el mensaje dice por qué."""


class Cancelled(Exception):
    pass


@dataclass(frozen=True)
class CutterTool:
    kind: str = "bola"           # "bola" o "plana"
    diameter: float = 10.0
    cut_length: float = 40.0     # de la punta a donde termina el filo
    holder_diameter: float = 0.0  # porta/husillo; 0 = no se revisa

    @property
    def radius(self) -> float:
        return self.diameter / 2


@dataclass(frozen=True)
class CamParams:
    strategy: str = "zigzag"     # "zigzag" (llena el área) o "contorno" (sigue el borde)
    side: str = "interior"       # contorno: "interior", "exterior" o "sobre" la línea
    stepover: float = 3.0        # mm entre pasadas
    depth: float = 0.0           # mm debajo de la superficie (0 = copiarla)
    stepdown: float = 2.0        # mm por nivel
    angle_deg: float = 0.0       # dirección del zigzag en el plano
    axis_mode: str = "fijo"      # "fijo" (como una CNC de 3 ejes) o "normal" (a la superficie)
    max_tilt_deg: float = 30.0   # "normal": cuánto puede inclinarse respecto de la vista
    safe_mm: float = 30.0        # sobre lo más alto de la zona, para los traslados
    tolerance_mm: float = 0.05   # desvío máximo al reducir puntos
    res_mm: float = 0.0          # celda del mapa (0 = automático)
    cut_speed: float = 2.0       # SPEED (%) cortando
    plunge_speed: float = 1.0    # SPEED (%) entrando al material
    rapid_speed: float = 10.0    # SPEED (%) en el aire
    roll_deg: float | None = None  # giro sobre el eje de la fresa (None = el mejor)
    spindle_output: str = ""     # salida del husillo (p. ej. "Y034"); vacío = a mano
    spindle_wait_s: float = 3.0
    tool_number: int = 0
    coord_number: int = 0


# --- mapa de alturas ----------------------------------------------------------------------


@dataclass
class HeightMap:
    view: np.ndarray              # filas u, v, w (celda)
    lo: tuple[float, float]       # (u, v) de la celda [0, 0]
    res: float
    h: np.ndarray                 # [fila v, columna u]; -inf = no hay pieza

    def hi(self) -> tuple[float, float]:
        return (self.lo[0] + (self.h.shape[1] - 1) * self.res,
                self.lo[1] + (self.h.shape[0] - 1) * self.res)

    def to_cell(self, uvw: np.ndarray) -> np.ndarray:
        return np.asarray(uvw, dtype=float) @ self.view

    def padded(self, mm: float) -> "HeightMap":
        n = int(math.ceil(mm / self.res))
        if n <= 0:
            return self
        return HeightMap(self.view, (self.lo[0] - n * self.res, self.lo[1] - n * self.res),
                         self.res, np.pad(self.h, n, constant_values=-np.inf))

    def sample(self, values: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Bilineal en `values` (mismo tamaño que `h`). Sin dato en una de las
        cuatro celdas o fuera del mapa: -inf."""
        x, y = (u - self.lo[0]) / self.res, (v - self.lo[1]) / self.res
        ny, nx = values.shape
        i, j = np.floor(x).astype(int), np.floor(y).astype(int)
        out = np.full(x.shape, -np.inf)
        ok = (i >= 0) & (j >= 0) & (i < nx - 1) & (j < ny - 1)
        i, j, fx, fy = i[ok], j[ok], (x - np.floor(x))[ok], (y - np.floor(y))[ok]
        corners = values[j, i], values[j, i + 1], values[j + 1, i], values[j + 1, i + 1]
        finite = np.isfinite(corners[0]) & np.isfinite(corners[1]) & np.isfinite(
            corners[2]) & np.isfinite(corners[3])
        with np.errstate(invalid="ignore"):
            mixed = (corners[0] * (1 - fx) * (1 - fy) + corners[1] * fx * (1 - fy)
                     + corners[2] * (1 - fx) * fy + corners[3] * fx * fy)
        out[ok] = np.where(finite, mixed, -np.inf)
        return out


def auto_resolution(tool: CutterTool) -> float:
    return max(0.2, tool.diameter / 10)


def _triangles_in_view(mesh: Mesh, matrix: Sequence[Sequence[float]], view: np.ndarray) -> np.ndarray:
    tris = np.asarray(mesh.triangles, dtype=float).reshape(-1, 3, 3)
    m = np.asarray(matrix, dtype=float)
    cell = tris @ m[:3, :3].T + m[:3, 3]
    return cell @ view.T


def _rasterize(tris: np.ndarray, lo: np.ndarray, res: float, shape: tuple[int, int]) -> np.ndarray:
    """Altura máxima de los triángulos en cada celda. Todo vectorizado: un
    bucle de Python por triángulo no aguanta las 400 mil caras de un STEP."""
    ny, nx = shape
    h = np.full(shape, -np.inf)
    uv = (tris[:, :, :2] - lo) / res
    i0 = np.maximum(np.ceil(uv[:, :, 0].min(axis=1)).astype(int), 0)
    i1 = np.minimum(np.floor(uv[:, :, 0].max(axis=1)).astype(int), nx - 1)
    j0 = np.maximum(np.ceil(uv[:, :, 1].min(axis=1)).astype(int), 0)
    j1 = np.minimum(np.floor(uv[:, :, 1].max(axis=1)).astype(int), ny - 1)
    a, b, c = uv[:, 0], uv[:, 1], uv[:, 2]
    # De canto a la vista (área 0): no aportan altura, las caras vecinas sí.
    den = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    keep = np.flatnonzero((i1 >= i0) & (j1 >= j0) & (np.abs(den) > 1e-12))
    if len(keep) == 0:
        return h
    wide, tall = (i1 - i0 + 1)[keep], (j1 - j0 + 1)[keep]
    counts = wide * tall
    start = 0
    while start < len(keep):
        end = max(start + 1, int(np.searchsorted(np.cumsum(counts[start:]), PAIRS_PER_CHUNK)) + start)
        ids, n, w = keep[start:end], counts[start:end], wide[start:end]
        which = np.repeat(np.arange(len(ids)), n)
        first = np.repeat(np.cumsum(n) - n, n)
        k = np.arange(len(which)) - first
        t = ids[which]
        col = i0[t] + k % w[which]
        row = j0[t] + k // w[which]
        l1 = ((b[t, 1] - c[t, 1]) * (col - c[t, 0]) + (c[t, 0] - b[t, 0]) * (row - c[t, 1])) / den[t]
        l2 = ((c[t, 1] - a[t, 1]) * (col - c[t, 0]) + (a[t, 0] - c[t, 0]) * (row - c[t, 1])) / den[t]
        l3 = 1 - l1 - l2
        eps = -1e-9
        inside = (l1 >= eps) & (l2 >= eps) & (l3 >= eps)
        z = l1 * tris[t, 0, 2] + l2 * tris[t, 1, 2] + l3 * tris[t, 2, 2]
        np.maximum.at(h, (row[inside], col[inside]), z[inside])
        start = end
    return h


def build_heightmap(mesh: Mesh, matrix: Sequence[Sequence[float]], view_name: str = "Arriba",
                    res: float = 0.0) -> HeightMap:
    """Mapa de alturas de la pieza vista desde `view_name`, en coordenadas de
    la celda (`matrix` = `object_matrix(pieza)`). Con `res` 0 elige ~1000
    celdas por lado; nunca pasa de `MAX_CELLS` (se engrosa)."""
    if len(mesh) == 0:
        raise CamError("La pieza no tiene triángulos")
    view = np.asarray(VIEWS[view_name], dtype=float)
    tris = _triangles_in_view(mesh, matrix, view)
    lo, hi = tris[:, :, :2].min(axis=(0, 1)), tris[:, :, :2].max(axis=(0, 1))
    size = hi - lo
    res = res if res > 0 else max(float(size.max()) / 1000, 0.05)
    res = max(res, math.sqrt(float(max(size[0], 1e-6) * max(size[1], 1e-6)) / MAX_CELLS))
    shape = (int(size[1] / res) + 2, int(size[0] / res) + 2)
    return HeightMap(view, (float(lo[0]), float(lo[1])), res, _rasterize(tris, lo, res, shape))


def _dilate(h: np.ndarray, res: float, radius: float, profile: Callable[[float], float]) -> np.ndarray:
    """out[c] = max de h[c + d] - profile(|d|) con |d| <= radius: la altura más
    baja a la que puede estar la punta sin que la herramienta entre en la pieza."""
    ny, nx = h.shape
    out = np.full(h.shape, -np.inf)
    r = int(math.ceil(radius / res))
    for dj in range(-r, r + 1):
        for di in range(-r, r + 1):
            rho = math.hypot(di, dj) * res
            if rho > radius:
                continue
            p = profile(rho)
            src = h[max(dj, 0):ny + min(dj, 0), max(di, 0):nx + min(di, 0)]
            dst = out[max(-dj, 0):ny + min(-dj, 0), max(-di, 0):nx + min(-di, 0)]
            np.maximum(dst, src - p, out=dst)
    return out


def _dilate_coarse(h: np.ndarray, res: float, radius: float) -> np.ndarray:
    """`_dilate` de un cilindro grande (la porta): con el mapa agrupado de a
    `f` celdas tomando el máximo, y un radio un poco mayor. Sobra, nunca falta."""
    f = max(1, int(math.ceil(radius / res / MAX_FOOTPRINT_MM)))
    ny, nx = h.shape
    py, px = (-ny) % f, (-nx) % f
    pooled = np.pad(h, ((0, py), (0, px)), constant_values=-np.inf)
    pooled = pooled.reshape(pooled.shape[0] // f, f, pooled.shape[1] // f, f).max(axis=(1, 3))
    grown = _dilate(pooled, res * f, radius + res * f * 1.5, lambda _rho: 0.0)
    return np.repeat(np.repeat(grown, f, axis=0), f, axis=1)[:ny, :nx]


def tip_map(hm: HeightMap, tool: CutterTool) -> np.ndarray:
    """Altura de la punta apoyada sobre la pieza, en cada celda de `hm`."""
    r = tool.radius
    if tool.kind == "bola":
        return _dilate(hm.h, hm.res, r, lambda rho: r - math.sqrt(max(r * r - rho * rho, 0.0)))
    return _dilate(hm.h, hm.res, r, lambda _rho: 0.0)


def holder_floor(hm: HeightMap, tool: CutterTool) -> np.ndarray | None:
    """Lo más abajo que puede ir la punta sin que la porta toque la pieza."""
    if tool.holder_diameter <= tool.diameter or tool.cut_length <= 0:
        return None
    return _dilate_coarse(hm.h, hm.res, tool.holder_diameter / 2) - tool.cut_length


# --- polígonos ------------------------------------------------------------------------------


def _inside(poly: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Par-impar. u, v: 1D."""
    a, b = poly, np.roll(poly, -1, axis=0)
    out = np.zeros(u.shape, dtype=bool)
    for (ax, ay), (bx, by) in zip(a, b):
        if ay == by:
            continue
        crosses = (ay > v) != (by > v)
        out ^= crosses & (u < ax + (v - ay) * (bx - ax) / (by - ay))
    return out


def _edge_distance(poly: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    a, b = poly, np.roll(poly, -1, axis=0)
    best = np.full(u.shape, np.inf)
    for (ax, ay), (bx, by) in zip(a, b):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = np.zeros(u.shape) if length2 == 0 else np.clip(((u - ax) * dx + (v - ay) * dy) / length2, 0, 1)
        best = np.minimum(best, np.hypot(u - (ax + t * dx), v - (ay + t * dy)))
    return best


def _area(poly: np.ndarray) -> float:
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def offset_polygon(poly: Polygon, distance: float) -> np.ndarray:
    """Polígono corrido `distance` hacia afuera (negativo: hacia adentro). Si un
    lado se invierte, la fresa no entra en esa parte: error, no un camino raro."""
    pts = np.asarray(poly, dtype=float)
    if _area(pts) < 0:
        pts = pts[::-1]
    if distance == 0:
        return pts
    edges = np.roll(pts, -1, axis=0) - pts
    normals = np.column_stack((edges[:, 1], -edges[:, 0]))
    normals /= np.maximum(np.hypot(normals[:, 0], normals[:, 1]), 1e-12)[:, None]
    out = []
    for i in range(len(pts)):
        n0, n1 = normals[i - 1], normals[i]
        bisector = n0 + n1
        den = 1 + float(n0 @ n1)
        # Ángulo muy cerrado: la esquina a inglete se iría lejos; queda el vértice corrido.
        out.append(pts[i] + distance * (bisector / den if den > 0.1 else n1))
    out = np.array(out)
    new_edges = np.roll(out, -1, axis=0) - out
    if np.any(np.sum(new_edges * edges, axis=1) <= 0) or _area(out) <= 0:
        raise CamError(f"El contorno corrido {abs(distance):g} mm se cruza consigo mismo: "
                       f"la fresa no entra en alguna parte del área")
    return out


def _sample_closed(poly: np.ndarray, step: float) -> np.ndarray:
    closed = np.vstack((poly, poly[:1]))
    out = []
    for a, b in zip(closed[:-1], closed[1:]):
        n = max(1, int(math.ceil(math.dist(a, b) / step)))
        out.append(a + (b - a) * (np.arange(n) / n)[:, None])
    return np.vstack(out + [closed[-1:]])


def _zigzag_runs(polygon: Polygon, inset: float, stepover: float, angle_deg: float,
                 step: float) -> list[np.ndarray]:
    """Renglones dentro del polígono, a `inset` del borde o más, en vaivén.
    Cada tramo es (n, 2) en u, v. Un polígono cóncavo da varios por renglón."""
    poly = np.asarray(polygon, dtype=float)
    c, s = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    a = poly[:, 0] * c + poly[:, 1] * s
    b = -poly[:, 0] * s + poly[:, 1] * c
    rows = np.arange(b.min() + inset, b.max() - inset + 1e-9, stepover)
    if len(rows) and b.max() - inset - rows[-1] > stepover * 0.1:
        rows = np.append(rows, b.max() - inset)   # que el último renglón llegue al borde
    along = np.arange(a.min(), a.max() + step, step)
    runs: list[np.ndarray] = []
    for index, row in enumerate(rows):
        u, v = along * c - row * s, along * s + row * c
        ok = _inside(poly, u, v)
        ok[ok] &= _edge_distance(poly, u[ok], v[ok]) >= inset - 1e-9
        edges = np.flatnonzero(np.diff(np.concatenate(([0], ok.astype(np.int8), [0]))))
        spans = [np.column_stack((u[p:q], v[p:q])) for p, q in zip(edges[::2], edges[1::2])
                 if q - p >= 2]
        if index % 2:
            spans = [span[::-1] for span in spans[::-1]]
        runs.extend(spans)
    if not runs:
        raise CamError(f"La fresa de {inset * 2:g} mm no entra en el área")
    return runs


def _join_runs(runs: list[np.ndarray], polygon: Polygon, inset: float, reach: float,
               step: float) -> list[np.ndarray]:
    """Une renglones vecinos sin levantar la fresa, si el paso queda en el área."""
    poly = np.asarray(polygon, dtype=float)
    joined = [runs[0]]
    for run in runs[1:]:
        p, q = joined[-1][-1], run[0]
        gap = math.dist(p, q)
        if gap > reach:
            joined.append(run)
            continue
        n = max(2, int(gap / step) + 2)
        link = p + (q - p) * (np.arange(n) / (n - 1))[:, None]
        u, v = link[:, 0], link[:, 1]
        if np.all(_inside(poly, u, v)) and np.all(_edge_distance(poly, u, v) >= inset - 1e-6):
            joined[-1] = np.vstack((joined[-1], link[1:-1], run))
        else:
            joined.append(run)
    return joined


# --- trayectoria ----------------------------------------------------------------------------


@dataclass
class ToolPath:
    """Puntos de la punta. `pos` y `axis` en coordenadas de la vista; `axis`
    es el eje de la fresa apuntando hacia la pieza. `kinds[i]` dice cómo se
    llega a `pos[i]`: "rapid" (aire), "plunge" (entrar) o "cut"."""

    hm: HeightMap
    pos: np.ndarray
    axis: np.ndarray
    kinds: list[str]
    warnings: list[str] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.kinds)

    @property
    def cut_length_mm(self) -> float:
        total = 0.0
        for i in range(1, len(self.kinds)):
            if self.kinds[i] == "cut":
                total += float(np.linalg.norm(self.pos[i] - self.pos[i - 1]))
        return total


def _simplify(pos: np.ndarray, axis: np.ndarray, tol: float) -> list[int]:
    """Índices que quedan (Douglas-Peucker con posición y eje): una recta sobre
    un plano son dos puntos, no cien."""
    rotation_mm = 200.0   # mm por radián: el mismo peso que usa `RobotModel.ik`
    keep = {0, len(pos) - 1}
    stack = [(0, len(pos) - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        chord = pos[b] - pos[a]
        length = float(np.linalg.norm(chord))
        rel = pos[a + 1:b] - pos[a]
        if length > 1e-9:
            along = rel @ (chord / length)
            dist = np.linalg.norm(rel - along[:, None] * (chord / length), axis=1)
        else:
            dist = np.linalg.norm(rel, axis=1)
        t = (np.arange(1, b - a) / (b - a))[:, None]
        twist = np.linalg.norm(axis[a + 1:b] - (axis[a] * (1 - t) + axis[b] * t), axis=1) * rotation_mm
        worst = np.maximum(dist, twist)
        i = int(np.argmax(worst))
        if worst[i] > tol:
            keep.add(a + 1 + i)
            stack += [(a, a + 1 + i), (a + 1 + i, b)]
    return sorted(keep)


def _surface_axes(hm: HeightMap, tip: np.ndarray, tool: CutterTool, params: CamParams) -> np.ndarray:
    """Pendiente de la superficie por donde pasa el centro de la bola, medida
    a la escala de la fresa: con la normal de cada triángulo el eje temblaría
    con el mallado. Devuelve (filas, columnas, 2): du, dv del eje."""
    k = max(1, int(round(tool.radius / 2 / hm.res)))
    safe = np.where(np.isfinite(tip), tip, np.nan)
    du = (np.roll(safe, -k, axis=1) - np.roll(safe, k, axis=1)) / (2 * k * hm.res)
    dv = (np.roll(safe, -k, axis=0) - np.roll(safe, k, axis=0)) / (2 * k * hm.res)
    slopes = np.dstack((np.nan_to_num(du), np.nan_to_num(dv)))
    limit = math.tan(math.radians(params.max_tilt_deg))
    norm = np.maximum(np.hypot(slopes[..., 0], slopes[..., 1]), 1e-12)
    return slopes * np.minimum(limit / norm, 1.0)[..., None]


def compute_path(hm: HeightMap, polygons: list[Polygon], tool: CutterTool, params: CamParams,
                 progress: Callable[[str], None] | None = None) -> ToolPath:
    """Pasadas de todas las áreas, en orden, con traslados por arriba."""
    say = progress or (lambda _text: None)
    if not polygons:
        raise CamError("Falta delimitar el área a cortar")
    if any(len(p) < 3 for p in polygons):
        raise CamError("Un área necesita al menos 3 puntos")
    if tool.kind not in ("bola", "plana") or tool.diameter <= 0:
        raise CamError("Herramienta inválida: hace falta bola o plana, con diámetro mayor que 0")
    if params.stepover <= 0 or params.stepdown <= 0 or params.depth < 0:
        raise CamError("Paso entre pasadas y paso por nivel tienen que ser mayores que 0")
    if params.axis_mode == "normal" and tool.kind != "bola":
        # El centro de la bola no depende de la inclinación; el canto de una
        # plana sí, y `tip_map` ya no garantizaría que no se clave.
        raise CamError("Seguir la normal de la superficie solo se calcula con fresa de bola")

    margin = tool.radius * 2 + hm.res
    work = hm.padded(margin + max(tool.holder_diameter / 2, 0.0))
    say("Apoyando la fresa sobre la pieza…")
    tip = tip_map(work, tool)
    floor = holder_floor(work, tool)
    slopes = _surface_axes(work, tip, tool, params) if params.axis_mode == "normal" else None

    step = work.res
    inset = 0.0 if params.strategy == "contorno" else tool.radius
    runs: list[np.ndarray] = []
    say("Armando las pasadas…")
    for polygon in polygons:
        if params.strategy == "zigzag":
            found = _zigzag_runs(polygon, inset, params.stepover, params.angle_deg, step)
            runs += _join_runs(found, polygon, inset, params.stepover * 2.5, step)
        elif params.strategy == "contorno":
            shift = {"interior": -tool.radius, "exterior": tool.radius, "sobre": 0.0}[params.side]
            runs.append(_sample_closed(offset_polygon(polygon, shift), step))
        else:
            raise CamError(f"Estrategia desconocida: {params.strategy}")

    levels = [0.0]
    if params.depth > 0:
        n = int(math.ceil(params.depth / params.stepdown - 1e-9))
        levels = [min(params.stepdown * (k + 1), params.depth) for k in range(n)]

    say("Extrapolando el área a la superficie…")
    passes: list[tuple[np.ndarray, np.ndarray]] = []
    clipped = over_air = 0
    for depth in levels:
        for run in runs:
            u, v = run[:, 0], run[:, 1]
            w = work.sample(tip, u, v)
            solid = np.isfinite(w)
            over_air += int(np.count_nonzero(~solid))
            w = w - depth
            if floor is not None:
                limit = work.sample(np.where(np.isfinite(floor), floor, -1e12), u, v)
                clipped += int(np.count_nonzero(solid & (w < limit)))
                w = np.maximum(w, limit)
            bounds = np.flatnonzero(np.diff(np.concatenate(([0], solid.astype(np.int8), [0]))))
            for p, q in zip(bounds[::2], bounds[1::2]):
                if q - p < 2:
                    continue
                pos = np.column_stack((u[p:q], v[p:q], w[p:q]))
                axis = np.tile((0.0, 0.0, -1.0), (q - p, 1))
                if slopes is not None:
                    du = work.sample(slopes[..., 0], u[p:q], v[p:q])
                    dv = work.sample(slopes[..., 1], u[p:q], v[p:q])
                    du, dv = np.nan_to_num(du, neginf=0.0), np.nan_to_num(dv, neginf=0.0)
                    axis = np.column_stack((du, dv, -np.ones(q - p)))
                    axis /= np.linalg.norm(axis, axis=1)[:, None]
                    # El centro de la bola es el mismo: la punta rota a su alrededor.
                    pos = pos + np.array((0.0, 0.0, tool.radius)) + axis * tool.radius
                keep = _simplify(pos, axis, params.tolerance_mm)
                passes.append((pos[keep], axis[keep]))
    if not passes:
        raise CamError("El área no pasa por encima de la pieza")

    top = max(float(p[:, 2].max()) for p, _a in passes) + params.safe_mm
    positions, axes, kinds = [], [], []

    def add(kind: str, p: np.ndarray, a: np.ndarray) -> None:
        positions.append(p)
        axes.append(a)
        kinds.append(kind)

    def above(p: np.ndarray, a: np.ndarray) -> np.ndarray:
        return p + a * ((top - p[2]) / a[2])   # por el eje de la fresa, hasta el plano seguro

    for pos, axis in passes:
        add("rapid", above(pos[0], axis[0]), axis[0])
        add("plunge", pos[0], axis[0])
        for p, a in zip(pos[1:], axis[1:]):
            add("cut", p, a)
        add("rapid", above(pos[-1], axis[-1]), axis[-1])

    warnings = []
    if clipped:
        warnings.append(f"{clipped} muestras no llegan a la profundidad pedida: la porta tocaría "
                        f"la pieza. Hace falta más largo de corte.")
    if over_air:
        warnings.append("Parte del área queda fuera de la pieza: ahí no se generaron pasadas.")
    if params.axis_mode == "normal" and tool.holder_diameter > 0:
        warnings.append("Con el eje inclinado la porta no se revisa acá: simulá con su modelo 3D.")
    if params.depth > tool.cut_length > 0:
        warnings.append(f"Profundidad {params.depth:g} mm mayor que el largo de corte "
                        f"{tool.cut_length:g} mm.")
    return ToolPath(hm, np.array(positions), np.array(axes), kinds, warnings)


# --- robot ----------------------------------------------------------------------------------


@dataclass
class RobotPath:
    path: ToolPath
    poses: list[tuple[float, ...]]     # WORLD(...) de cada punto
    start_q: list[float]               # pose articular sobre el primer punto
    roll_deg: float
    unreachable: list[int] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def _tcp_matrices(path: ToolPath, roll_deg: float) -> np.ndarray:
    """Punta de la herramienta en la celda. +Z = eje de la fresa, hacia la pieza."""
    pos, axis = path.hm.to_cell(path.pos), path.hm.to_cell(path.axis)
    ref = path.hm.view[0]
    x = ref - (axis @ ref)[:, None] * axis
    x /= np.linalg.norm(x, axis=1)[:, None]
    y = np.cross(axis, x)
    c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    x, y = x * c + y * s, y * c - x * s
    out = np.tile(np.eye(4), (len(pos), 1, 1))
    out[:, :3, 0], out[:, :3, 1], out[:, :3, 2], out[:, :3, 3] = x, y, axis, pos
    return out


def _start_seeds(model: RobotModel, target: np.ndarray) -> list[list[float]]:
    angle = math.degrees(math.atan2(target[1, 3], target[0, 3])) * model.joints[0].sign
    return [[j1, j2, j3, 0.0, j5, 0.0]
            for j1 in (angle, angle + 180 if angle < 0 else angle - 180)
            for j5 in (-90.0, -60.0, -120.0, -30.0, 60.0, 90.0)
            for j2, j3 in ((0.0, 0.0), (30.0, -30.0), (-30.0, 30.0))]


def _first_solution(model: RobotModel, target: np.ndarray, seeds: list[list[float]]) -> list[float] | None:
    for seed in seeds:
        q = model.ik(target.tolist(), seed, max_iter=60)
        if q is not None:
            q = model.wrap_into_limits(q, seed)
            if not model.out_of_limits(q):
                return q
    return None


def _chain(model: RobotModel, flanges: np.ndarray, q: list[float],
           progress: Callable[[int], None] | None = None) -> tuple[list[int], list[float], float]:
    """Sigue los puntos en orden. Devuelve (sin alcanzar, primera pose, peor
    salto de un eje en grados)."""
    bad, first, worst = [], None, 0.0
    for i, target in enumerate(flanges):
        if progress:
            progress(i)
        nq = model.ik(target.tolist(), q)
        if nq is None or model.out_of_limits(nq):
            bad.append(i)
            continue
        worst = max(worst, max(abs(a - b) for a, b in zip(nq, q))) if first is not None else worst
        q = nq
        first = first or nq
    return bad, first or q, worst


def solve_robot(path: ToolPath, model: RobotModel, robot_base: Sequence[float],
                tool_pose: Sequence[float], frame_pose: Sequence[float], roll_deg: float | None = None,
                progress: Callable[[str], None] | None = None) -> RobotPath:
    """Lleva la trayectoria al robot: elige el giro de la fresa y verifica
    todos los puntos con la cinemática inversa."""
    say = progress or (lambda _text: None)
    base_inv = np.linalg.inv(np.array(pose_matrix(*robot_base)))
    frame_inv = np.linalg.inv(np.array(pose_matrix(*frame_pose)))
    tool_inv = np.linalg.inv(np.array(pose_matrix(*tool_pose)))

    def in_base(roll: float) -> np.ndarray:
        return base_inv @ _tcp_matrices(path, roll)

    sample = np.unique(np.linspace(0, len(path) - 1, min(len(path), 24)).astype(int))
    best: tuple[tuple, float, list[float]] | None = None
    for roll in ([roll_deg] if roll_deg is not None else [float(a) for a in range(0, 360, 30)]):
        say(f"Probando la fresa girada {roll:.0f}°…")
        flanges = in_base(roll) @ tool_inv
        start = _first_solution(model, flanges[0], _start_seeds(model, flanges[0]))
        if start is None:
            continue
        bad, first, jump = _chain(model, flanges[sample], start)
        wrist = min(abs(first[4]), 180 - abs(first[4]))
        score = (len(bad), jump > 30, -wrist)   # J5 lejos de 0: lejos de la singularidad
        if best is None or score < best[0]:
            best = (score, roll, start)
    if best is None:
        raise CamError("El robot no llega al primer punto de la trayectoria con ningún giro de "
                       "la fresa: acercá la pieza o revisá la herramienta")
    _score, roll, start = best

    matrices = in_base(roll)
    flanges = matrices @ tool_inv
    every = max(1, len(path) // 50)

    def tick(i: int) -> None:
        if i % every == 0:
            say(f"Verificando el punto {i + 1} de {len(path)}…")

    bad, first, jump = _chain(model, flanges, start, tick)
    warnings = []
    if jump > 30:
        warnings.append(f"Un eje salta {jump:.0f}° entre dos puntos: probable paso cerca de una "
                        f"singularidad. Revisalo en la simulación.")
    poses = [tuple(matrix_to_pose((frame_inv @ m).tolist())) for m in matrices]
    return RobotPath(path, poses, first, roll, bad, warnings)


# --- programa -------------------------------------------------------------------------------


def generate_program(robot: RobotPath, tool: CutterTool, params: CamParams,
                     name: str = "fresado") -> ProgramSources:
    """Programa .src + .dat, en el lenguaje de siempre: lo compila, lo simula y
    lo exporta al pad el camino normal. Con puntos sin alcanzar no hay programa."""
    if robot.unreachable:
        where = robot.path.hm.to_cell(robot.path.pos[robot.unreachable[:UNREACHABLE_LISTED]])
        listed = "; ".join(f"({x:.0f}, {y:.0f}, {z:.0f})" for x, y, z in where)
        raise CamError(f"El robot no alcanza {len(robot.unreachable)} de {len(robot.path)} puntos "
                       f"(celda, mm: {listed}…). Acercá o girá la pieza; no se generó programa.")

    def speed(value: float) -> str:
        return f"{value:g}"

    dat = ["; Puntos generados por el módulo de fresado (sim/cam.py): no editar a mano",
           "POINT f_inicio = JOINT(" + ", ".join(f"{q:.3f}" for q in robot.start_q) + ")"]
    src = [f"; Fresado: {tool.kind} de {tool.diameter:g} mm, {params.strategy}, "
           f"profundidad {params.depth:g} mm, {robot.path.cut_length_mm:.0f} mm de corte",
           f"; {len(robot.poses)} puntos. Fresa girada {robot.roll_deg:.0f} grados sobre su eje.",
           "; NO PROBADO EN EL ROBOT: la primera vez, sin fresa y a velocidad baja."]
    if params.tool_number:
        src.append(f"TOOL {params.tool_number}")
    if params.coord_number:
        src.append(f"COORD {params.coord_number}")
    src.append(f"MOVEJ f_inicio SPEED {speed(params.rapid_speed)}")
    if params.spindle_output:
        src.append(f"SET_OUT({params.spindle_output}, ON)")
        src.append(f"WAIT {params.spindle_wait_s:g} s")
    speeds = {"rapid": params.rapid_speed, "plunge": params.plunge_speed, "cut": params.cut_speed}
    for i, (pose, kind) in enumerate(zip(robot.poses, robot.path.kinds), 1):
        dat.append(f"POINT f{i:05d} = WORLD(" + ", ".join(f"{x:.3f}" for x in pose) + ")")
        src.append(f"MOVEL f{i:05d} SPEED {speed(speeds[kind])}")
    if params.spindle_output:
        src.append(f"SET_OUT({params.spindle_output}, OFF)")
    src.append(f"MOVEJ f_inicio SPEED {speed(params.rapid_speed)}")
    return ProgramSources(SourceFile(f"{name}{SRC_SUFFIX}", "\n".join(src) + "\n"),
                          SourceFile(f"{name}{DAT_SUFFIX}", "\n".join(dat) + "\n"),
                          SourceFile(CONFIG_DAT, ""))
