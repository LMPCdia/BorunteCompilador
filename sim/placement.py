"""
Ubicar piezas de la celda por distancias, y medir distancias.

Todo se mide sobre la caja que envuelve la pieza YA GIRADA (en el mundo), y
el "punto de apoyo" de una pieza es el centro de la base de esa caja: es lo
que uno mide con un metro en el taller ("la mesa está a 800 mm del robot",
"el posicionador va a 200 mm de la mesa").

Formas de ubicar (cada una devuelve x, y, z para `LayoutObject`, que es
dónde queda el ORIGEN del CAD):

- `offset_from`: el apoyo a ΔX, ΔY, ΔZ del apoyo de otra pieza o de la base
  del robot.
- `next_to`: al lado de otra pieza, con una separación entre caras, sobre
  +X, -X, +Y o -Y, centrada en el otro eje y en el mismo piso.
- `on_top`: apoyada encima de otra pieza (centrada, más un corrimiento).
- `from_robot`: a una distancia del eje de J1, en un ángulo (0° = adelante
  del robot, +X; 90° = a su izquierda, +Y), medida hasta el centro o hasta la
  cara más cercana.

Medir: `gaps` (separación entre cajas por eje) y, con python-fcl,
`clearance` (distancia mínima real entre superficies).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from sim.meshes import Mesh
from sim.scene import LayoutObject

Vec = tuple[float, float, float]
SIDES = ("+X", "-X", "+Y", "-Y")


@dataclass(frozen=True)
class Box:
    lo: Vec
    hi: Vec

    @property
    def center(self) -> Vec:
        return tuple((a + b) / 2 for a, b in zip(self.lo, self.hi))

    @property
    def base(self) -> Vec:
        """Centro de la base (el punto de apoyo)."""
        c = self.center
        return (c[0], c[1], self.lo[2])

    @property
    def size(self) -> Vec:
        return tuple(b - a for a, b in zip(self.lo, self.hi))


_VERTEX_CACHE: dict[int, list[Vec]] = {}


def _vertices(mesh: Mesh) -> list[Vec]:
    key = id(mesh)
    cached = _VERTEX_CACHE.get(key)
    if cached is None or len(cached) == 0:
        cached = list({p for tri in mesh.triangles for p in tri})
        _VERTEX_CACHE[key] = cached
    return cached


def world_box(obj: LayoutObject, mesh: Mesh) -> Box:
    """Caja del mundo de la pieza ubicada (con su giro)."""
    c, s = math.cos(math.radians(obj.rz)), math.sin(math.radians(obj.rz))
    xs, ys, zs = [], [], []
    for x, y, z in _vertices(mesh):
        xs.append(c * x - s * y)
        ys.append(s * x + c * y)
        zs.append(z)
    if not xs:
        return Box((obj.x, obj.y, obj.z), (obj.x, obj.y, obj.z))
    return Box((min(xs) + obj.x, min(ys) + obj.y, min(zs) + obj.z),
               (max(xs) + obj.x, max(ys) + obj.y, max(zs) + obj.z))


def _move_base_to(obj: LayoutObject, mesh: Mesh, target: Vec) -> Vec:
    """x, y, z del origen para que el apoyo de la pieza quede en `target`."""
    base = world_box(obj, mesh).base
    return (round(obj.x + target[0] - base[0], 1), round(obj.y + target[1] - base[1], 1),
            round(obj.z + target[2] - base[2], 1))


ROBOT_BASE = Box((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))  # el origen: eje de J1, en el piso


def offset_from(obj: LayoutObject, mesh: Mesh, ref: Box, dx: float, dy: float, dz: float) -> Vec:
    rb = ref.base
    return _move_base_to(obj, mesh, (rb[0] + dx, rb[1] + dy, rb[2] + dz))


def next_to(obj: LayoutObject, mesh: Mesh, ref: Box, side: str, gap: float) -> Vec:
    if side not in SIDES:
        raise ValueError(f"lado {side!r}: tiene que ser uno de {', '.join(SIDES)}")
    mine = world_box(obj, mesh)
    half = [s / 2 for s in mine.size]
    cx, cy, _ = ref.center
    if side == "+X":
        target = (ref.hi[0] + gap + half[0], cy)
    elif side == "-X":
        target = (ref.lo[0] - gap - half[0], cy)
    elif side == "+Y":
        target = (cx, ref.hi[1] + gap + half[1])
    else:
        target = (cx, ref.lo[1] - gap - half[1])
    return _move_base_to(obj, mesh, (target[0], target[1], ref.lo[2]))


def on_top(obj: LayoutObject, mesh: Mesh, ref: Box, dx: float = 0.0, dy: float = 0.0) -> Vec:
    cx, cy, _ = ref.center
    return _move_base_to(obj, mesh, (cx + dx, cy + dy, ref.hi[2]))


def from_robot(obj: LayoutObject, mesh: Mesh, distance: float, angle_deg: float,
               to_face: bool = True) -> Vec:
    """A `distance` mm del eje de J1, en la dirección `angle_deg`. Con
    `to_face`, la distancia es hasta la cara de la pieza que mira al robot
    (lo que queda libre); si no, hasta su centro."""
    ux, uy = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    r = distance
    if to_face:
        mine = world_box(obj, mesh)
        hx, hy = mine.size[0] / 2, mine.size[1] / 2
        # Cuánto hay del centro de la caja a su borde en esa dirección.
        r += min(hx / abs(ux) if abs(ux) > 1e-9 else math.inf,
                 hy / abs(uy) if abs(uy) > 1e-9 else math.inf)
    return _move_base_to(obj, mesh, (r * ux, r * uy, 0.0))


# --- medir --------------------------------------------------------------------------


def gaps(a: Box, b: Box) -> Vec:
    """Separación entre las cajas por eje (negativo = se superponen en ese eje)."""
    return tuple(max(b.lo[i] - a.hi[i], a.lo[i] - b.hi[i]) for i in range(3))


def axis_distance(box: Box) -> float:
    """Distancia horizontal del eje de J1 al punto más cercano de la caja."""
    dx = max(box.lo[0], 0.0, -box.hi[0]) if not box.lo[0] <= 0 <= box.hi[0] else 0.0
    dy = max(box.lo[1], 0.0, -box.hi[1]) if not box.lo[1] <= 0 <= box.hi[1] else 0.0
    return math.hypot(dx, dy)


def clearance(obj_a: LayoutObject, mesh_a: Mesh, obj_b: LayoutObject, mesh_b: Mesh) -> float | None:
    """Distancia mínima entre las superficies de dos piezas (0 = se tocan).
    None si falta python-fcl."""
    from sim import collision

    if not collision.available():
        return None
    a = collision._Solid(0, collision.Obstacle("a", mesh_a, object_matrix(obj_a)))
    b = collision._Solid(1, collision.Obstacle("b", mesh_b, object_matrix(obj_b)))
    d = collision.fcl.distance(a.object, b.object, collision.fcl.DistanceRequest(),
                               collision.fcl.DistanceResult())
    return max(0.0, float(d))


def object_matrix(obj: LayoutObject):
    c, s = math.cos(math.radians(obj.rz)), math.sin(math.radians(obj.rz))
    return [[c, -s, 0.0, obj.x], [s, c, 0.0, obj.y], [0.0, 0.0, 1.0, obj.z], [0.0, 0.0, 0.0, 1.0]]
