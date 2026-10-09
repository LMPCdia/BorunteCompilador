"""
Choques del robot contra la celda, con python-fcl.

Qué se revisa, sobre la trayectoria que ya simuló `PadSimulator`:

- cada parte del robot (eslabones J1..J6 y la herramienta montada en la brida)
  contra cada pieza del layout;
- el robot contra el piso (el plano de apoyo de la base, z = 0);
- el robot contra sí mismo (eslabones a tres o más ejes de distancia, y la
  herramienta contra el brazo).

Las piezas son SÓLIDOS: fcl solo ve superficies (un eslabón metido entero
adentro de una mesa no "toca" ningún triángulo), así que además se prueba si
la parte quedó adentro cada vez que el robot aparece de golpe en una pose (el
primer punto y después de cada salto). Entre medio el movimiento es continuo y
para meterse adentro tiene que cruzar la superficie, que sí se detecta.

Entre dos muestras no se mira "a ojo": avance conservador. Si los ejes cambian
Δq entre dos muestras, ningún punto de una parte se mueve más que
Σ |Δq_i|·R_i (R_i: cuánto se aleja la parte del eje i, una cota que no depende
de la pose). Si la distancia en los extremos alcanza para cubrir eso, el tramo
está libre; si no, se parte al medio y se vuelve a medir, hasta RESOLUTION_MM.
Un choque no se puede escapar entre dos muestras, por más rápido que gire J1.

Lo que NO es exacto (hipótesis, decirlo siempre al mostrar resultados):
- la forma del robot, mientras el modelo no traiga las mallas del fabricante
  (cilindros aproximados, ver `sim/scene.py`);
- la trayectoria entre puntos (`docs/SIMULATOR.md`): el controlador real
  puede redondear esquinas o interpolar distinto;
- la herramienta, si no se carga su modelo 3D.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

from sim.kinematics import Matrix, RobotModel, mat_mul, pose_matrix
from sim.meshes import Mesh
from sim.pad_sim import Cancelled, Issue, SimResult  # noqa: F401 — Cancelled: lo tira progress

try:  # La dependencia es opcional: sin ella el simulador anda igual, sin choques.
    import fcl
    import numpy as np
except ImportError:  # pragma: no cover — depende de la instalación
    fcl = None
    np = None
if fcl is not None and not hasattr(fcl, "distance"):  # pragma: no cover
    # Si la extensión compilada no carga (p. ej. le falta una DLL), el paquete
    # se importa igual pero vacío: imprime el error y sigue.
    fcl = None

RESOLUTION_MM = 3.0          # hasta dónde se parte un tramo dudoso
LOWER_BOUND_SLACK_MM = 150.0  # más lejos que margen + esto: no hace falta fcl
MAX_ISSUES = 60              # más que esto en "Problemas" no ayuda a nadie
PROGRESS_EVERY = 200         # evaluaciones entre llamadas a `progress`
SHOW_S = 0.15                # cuánto se ve pintada una pieza alrededor del choque

PART_NAMES = ["base", "columna (J1)", "brazo (J2)", "codo (J3)", "antebrazo (J4)",
              "muñeca (J5)", "brida (J6)"]
TOOL_PART = "herramienta"
FLOOR = "el piso"


def available() -> bool:
    return fcl is not None


class CollisionUnavailable(RuntimeError):
    pass


# --- resultado ------------------------------------------------------------------------


@dataclass
class Contact:
    """Lo peor que pasó entre una parte del robot y un obstáculo en un tramo."""
    segment: int          # índice en `SimResult.segments`
    where: str
    kind: str             # "MOVEJ" | "MOVEL" | ...
    obstacle: str         # nombre de la pieza, FLOOR, o una parte del robot
    obstacle_index: int | None  # índice en la lista de piezas (None: piso o el robot)
    parts: list[str]
    distance_mm: float    # mínima medida (0 = se tocan o se meten)
    time_s: float         # cuándo (el primer choque, o la distancia mínima)
    fraction: float       # cuándo, como fracción del tramo
    start_s: float        # intervalo en el que está en falta (para pintar en vivo)
    end_s: float
    inside: bool = False  # la parte quedó adentro de la pieza (sólido)

    @property
    def colliding(self) -> bool:
        return self.distance_mm <= 0.0


@dataclass
class CollisionReport:
    contacts: list[Contact] = field(default_factory=list)
    issues: list[Issue] = field(default_factory=list)
    evaluations: int = 0      # poses medidas (para los tests y el resumen)
    approximate_robot: bool = True

    @property
    def collisions(self) -> int:
        return sum(c.colliding for c in self.contacts)

    @property
    def near_misses(self) -> int:
        return sum(not c.colliding for c in self.contacts)

    def state_at(self, t: float) -> dict[int, str]:
        """Pieza -> "choque" | "cerca" en el instante t (para pintarlas)."""
        out: dict[int, str] = {}
        for c in self.contacts:
            # Un roce de un instante tiene que verse igual al animar (33 ms por cuadro).
            if c.obstacle_index is None or not (c.start_s - SHOW_S <= t <= c.end_s + SHOW_S):
                continue
            if c.colliding or out.get(c.obstacle_index) != "choque":
                out[c.obstacle_index] = "choque" if c.colliding else "cerca"
        return out


# --- geometría ---------------------------------------------------------------------------


@dataclass
class Obstacle:
    name: str
    mesh: Mesh           # en coordenadas del CAD
    matrix: Matrix       # dónde está en el mundo
    workpiece: bool = False  # la herramienta trabaja sobre ella: solo cuenta tocarla


def _bvh(vertices, faces):
    model = fcl.BVHModel()
    model.beginModel(len(vertices), len(faces))
    model.addSubModel(vertices, faces)
    model.endModel()
    return model


def _arrays(mesh: Mesh):
    """Vértices únicos e índices de triángulos (las mallas vienen sin índices)."""
    tris = np.asarray(mesh.triangles, dtype=float).reshape(-1, 3)
    vertices, inverse = np.unique(np.round(tris, 6), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3).astype(np.int32)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
                  & (faces[:, 0] != faces[:, 2])]
    return vertices, faces


def _transform(m: Matrix):
    return fcl.Transform(np.array([row[:3] for row in m[:3]], dtype=float),
                         np.array([m[0][3], m[1][3], m[2][3]], dtype=float))


def _apply(m: Matrix, p) -> tuple[float, float, float]:
    return tuple(m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3))


class _Part:
    def __init__(self, name: str, frame: int | None, mesh: Mesh, level: int) -> None:
        self.name = name
        self.frame = frame          # índice en joint_frames (None: no se mueve)
        self.level = level          # 0 = base ... 6 = brida (la herramienta también 6)
        self.vertices, faces = _arrays(mesh)
        self.object = fcl.CollisionObject(_bvh(self.vertices, faces))
        lo, hi = self.vertices.min(axis=0), self.vertices.max(axis=0)
        self.center = tuple(float(x) for x in (lo + hi) / 2)        # en la posición cero
        self.radius = float(np.linalg.norm(self.vertices - np.array(self.center), axis=1).max())
        self.reach: list[float] = []  # R_i: cota de distancia al eje i, para i <= frame


class _Solid:
    """Pieza del layout como sólido: fcl para la superficie, rayos para el adentro."""

    def __init__(self, index: int, obstacle: Obstacle) -> None:
        self.index = index
        self.name = obstacle.name
        self.workpiece = obstacle.workpiece
        vertices, faces = _arrays(obstacle.mesh)
        self.object = fcl.CollisionObject(_bvh(vertices, faces), _transform(obstacle.matrix))
        world = vertices @ np.array([r[:3] for r in obstacle.matrix[:3]]).T + np.array(
            [obstacle.matrix[i][3] for i in range(3)])
        self.lo, self.hi = world.min(axis=0), world.max(axis=0)
        self.triangles = world[faces]                       # (n, 3, 3), en el mundo

    def lower_bound(self, center, radius: float) -> float:
        c = np.asarray(center)
        gap = np.maximum(np.maximum(self.lo - c, c - self.hi), 0.0)
        return float(np.linalg.norm(gap)) - radius

    def contains(self, point) -> bool:
        """Paridad de cortes de un rayo (malla cerrada). Dirección "rara" para
        no pasar justo por una arista."""
        p = np.asarray(point, dtype=float)
        if np.any(p < self.lo) or np.any(p > self.hi):
            return False
        d = np.array([0.5773, 0.5791, 0.5755])
        v0, e1, e2 = (self.triangles[:, 0], self.triangles[:, 1] - self.triangles[:, 0],
                      self.triangles[:, 2] - self.triangles[:, 0])
        h = np.cross(d, e2)
        a = np.einsum("ij,ij->i", e1, h)
        ok = np.abs(a) > 1e-12
        f = np.zeros_like(a)
        f[ok] = 1.0 / a[ok]
        s = p - v0
        u = f * np.einsum("ij,ij->i", s, h)
        q = np.cross(s, e1)
        v = f * (q @ d)
        t = f * np.einsum("ij,ij->i", e2, q)
        hits = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-9)
        return int(hits.sum()) % 2 == 1


def tool_in_flange_zero(model: RobotModel, mesh: Mesh, mount: list[float]) -> Mesh:
    """La herramienta (en coordenadas de la brida) llevada a la posición cero
    del robot, que es como se dibujan y se chocan los eslabones."""
    m = mat_mul(model.fk([0.0] * 6), pose_matrix(*mount))
    out = Mesh()
    for tri in mesh.triangles:
        out.add(*(_apply(m, p) for p in tri))
    return out


# --- el chequeo -------------------------------------------------------------------------


class CollisionChecker:
    def __init__(self, model: RobotModel, link_meshes: list[Mesh], obstacles: list[Obstacle],
                 tool_mesh: Mesh | None = None, tool_mount: list[float] | None = None,
                 margin_mm: float = 20.0, floor: bool = True, self_collision: bool = True,
                 approximate_robot: bool = True, base: Matrix | None = None) -> None:
        """`link_meshes`: base + J1..J6 en la posición cero (como los dibuja el
        visor, sin el eje de dibujo de la brida). `tool_mesh`: la herramienta
        física en coordenadas de la brida, montada con `tool_mount` (X, Y, Z,
        U, V, W respecto de la brida). `base`: dónde está parado el robot en la
        celda (`Layout.base_matrix()`); las piezas y el piso están en
        coordenadas de la celda."""
        if fcl is None:
            raise CollisionUnavailable("Falta python-fcl: no se pueden revisar choques")
        if len(link_meshes) != 7:
            raise ValueError("hacen falta 7 mallas: base y J1..J6")
        self.model = model
        self.margin = max(0.0, float(margin_mm))
        self.base = base if base is not None else [[float(i == j) for j in range(4)]
                                                    for i in range(4)]
        self.floor = floor
        self.approximate_robot = approximate_robot
        self.parts = [_Part(PART_NAMES[i], None if i == 0 else i - 1, m, i)
                      for i, m in enumerate(link_meshes) if len(m)]
        if tool_mesh is not None and len(tool_mesh):
            self.parts.append(_Part(TOOL_PART, 5, tool_in_flange_zero(
                model, tool_mesh, tool_mount or [0.0] * 6), 6))
        self.solids = [_Solid(i, o) for i, o in enumerate(obstacles) if len(o.mesh)]
        self._reaches()

        # Pares a revisar: (parte, "obstáculo", margen). La base no: lo que la
        # toca es el pedestal o la mesa donde está atornillada.
        self.pairs: list[tuple[_Part, object, float]] = []
        for part in self.parts:
            if part.frame is None:
                continue
            for solid in self.solids:
                margin = 0.0 if (part.name == TOOL_PART and solid.workpiece) else self.margin
                self.pairs.append((part, solid, margin))
            if floor and part.level >= 2:
                self.pairs.append((part, FLOOR, self.margin))
        if self_collision:
            for i, a in enumerate(self.parts):
                for b in self.parts[i + 1:]:
                    lo, hi = sorted((a, b), key=lambda p: p.level)
                    gap = 2 if hi.name == TOOL_PART else 3  # la herramienta llega al antebrazo
                    if hi.level - lo.level >= gap:
                        self.pairs.append((hi, lo, 0.0))  # consigo mismo: solo si se toca

    def _reaches(self) -> None:
        """R_i de cada parte: |p - c_i| <= Σ |c_(j+1) - c_j| + |s - c_k| + r,
        con c_i un punto del eje i. Todo eso es rígido: vale en cualquier pose."""
        points = [p for _axis, p in self.model._screws()]
        for part in self.parts:
            if part.frame is None:
                continue
            k = part.frame
            own = math.dist(part.center, points[k]) + part.radius
            part.reach = []
            for i in range(k + 1):
                chain = sum(math.dist(points[j], points[j + 1]) for j in range(i, k))
                part.reach.append(chain + own)

    # -- medir ------------------------------------------------------------------------------

    def _pose(self, q: list[float]):
        """Por parte: (matriz a la celda, centro en la celda)."""
        frames = self.model.joint_frames(q)
        placed = {}
        for part in self.parts:
            m = self.base if part.frame is None else mat_mul(self.base, frames[part.frame])
            placed[id(part)] = (m, _apply(m, part.center))
        return placed

    def _set(self, part: _Part, placed, done: set) -> None:
        if id(part) in done:
            return
        m = placed[id(part)][0]
        if m is not None:
            part.object.setTransform(_transform(m))
        done.add(id(part))

    def _distances(self, q: list[float], pairs) -> dict[int, tuple[float, bool]]:
        """Por par: (distancia, exacta). Una cota inferior alcanza si está lejos."""
        self.evaluations += 1
        if self._progress is not None and self.evaluations % PROGRESS_EVERY == 0:
            self._progress(self.evaluations)  # puede tirar Cancelled
        placed = self._pose(q)
        done: set = set()
        out = {}
        for n in pairs:
            part, other, margin = self.pairs[n]
            m, center = placed[id(part)]
            far = margin + LOWER_BOUND_SLACK_MM
            if other == FLOOR:
                lb = center[2] - part.radius
                if lb > far:
                    out[n] = (lb, False)
                    continue
                rot = np.array([m[2][0], m[2][1], m[2][2]])
                out[n] = (float((part.vertices @ rot).min() + m[2][3]), True)
                continue
            if isinstance(other, _Solid):
                lb = other.lower_bound(center, part.radius)
            else:
                lb = math.dist(center, placed[id(other)][1]) - part.radius - other.radius
            if lb > far:
                out[n] = (lb, False)
                continue
            self._set(part, placed, done)
            if isinstance(other, _Part):
                self._set(other, placed, done)
            d = fcl.distance(part.object, other.object, fcl.DistanceRequest(),
                             fcl.DistanceResult())
            out[n] = (max(0.0, float(d)), True)
        return out

    def _bound(self, part: _Part, qa, qb) -> float:
        return sum(abs(math.radians(b - a)) * r for a, b, r in zip(qa, qb, part.reach))

    def distances_at(self, q: list[float]) -> list[tuple[str, str, float]]:
        """(parte, obstáculo, distancia exacta) de cada par, para una pose."""
        self.evaluations, self._progress = 0, None
        out = []
        placed = self._pose(q)
        done: set = set()
        for n, (part, other, _margin) in enumerate(self.pairs):
            if other == FLOOR:
                m = placed[id(part)][0]
                rot = np.array([m[2][0], m[2][1], m[2][2]])
                d = float((part.vertices @ rot).min() + m[2][3])
                out.append((part.name, FLOOR, d))
                continue
            self._set(part, placed, done)
            if isinstance(other, _Part):
                self._set(other, placed, done)
            d = fcl.distance(part.object, other.object, fcl.DistanceRequest(),
                             fcl.DistanceResult())
            out.append((part.name, other.name, max(0.0, float(d))))
        return out

    def robot_distance(self, q: list[float], obstacle: Obstacle) -> tuple[str, float]:
        """(parte más cercana, distancia mínima) del robot en la pose `q` a
        una pieza cualquiera (para medir al ubicar piezas)."""
        solid = _Solid(-1, obstacle)
        placed = self._pose(q)
        done: set = set()
        best = ("", math.inf)
        for part in self.parts:
            self._set(part, placed, done)
            d = fcl.distance(part.object, solid.object, fcl.DistanceRequest(), fcl.DistanceResult())
            if d < best[1]:
                best = (part.name, max(0.0, float(d)))
        return best

    # -- recorrer la simulación -----------------------------------------------------------------

    def check(self, result: SimResult,
              progress: Callable[[int], None] | None = None) -> CollisionReport:
        self.evaluations, self._progress = 0, progress
        self._events: dict[tuple[int, int], dict] = {}
        all_pairs = list(range(len(self.pairs)))
        t0 = 0.0
        previous_end: list[float] | None = None
        for s, seg in enumerate(result.segments):
            duration = max(0.0, seg.duration_s)
            if seg.kind in ("MOVEJ", "MOVEL") and seg.samples:
                times = seg.times if len(seg.times) == len(seg.samples) else [
                    duration * i / max(1, len(seg.samples) - 1) for i in range(len(seg.samples))]
                self._segment = (s, seg, t0, duration)
                first = seg.samples[0]
                d_prev = self._distances(first, all_pairs)
                self._observe(first, t0 + times[0], d_prev, all_pairs)
                if previous_end is None or max(abs(a - b) for a, b in zip(first, previous_end)) > 1e-6:
                    self._inside(first, t0 + times[0], d_prev)
                for i in range(1, len(seg.samples)):
                    qa, qb = seg.samples[i - 1], seg.samples[i]
                    d_next = self._distances(qb, all_pairs)
                    self._observe(qb, t0 + times[i], d_next, all_pairs)
                    self._interval(qa, qb, t0 + times[i - 1], t0 + times[i], d_prev, d_next,
                                   all_pairs)
                    d_prev = d_next
                previous_end = seg.samples[-1]
            elif seg.samples:
                previous_end = None if seg.kind == "SALTO" else seg.samples[-1]
            t0 += duration
        return self._report(result)

    def _interval(self, qa, qb, ta, tb, da, db, pairs) -> None:
        need = []
        for n in pairs:
            part, other, margin = self.pairs[n]
            bound = self._bound(part, qa, qb)
            if isinstance(other, _Part):
                bound += self._bound(other, qa, qb)
            a, b = da[n][0], db[n][0]
            lower = (a + b - bound) / 2  # nada en el tramo puede estar más cerca que esto
            if lower > margin:
                continue
            if a <= 0 and b <= 0:
                continue                 # choca de punta a punta: ya está anotado
            if bound < RESOLUTION_MM:
                continue
            if lower > 0 and min(a, b) < margin:
                continue                 # no llega a chocar y la cercanía ya se anotó
            need.append(n)
        if not need:
            return
        qm = [(a + b) / 2 for a, b in zip(qa, qb)]
        tm = (ta + tb) / 2
        dm = self._distances(qm, need)
        self._observe(qm, tm, dm, need)
        self._interval(qa, qm, ta, tm, da, dm, need)
        self._interval(qm, qb, tm, tb, dm, db, need)

    def _observe(self, q, t: float, distances, pairs) -> None:
        s, seg, t0, duration = self._segment
        for n in pairs:
            d, exact = distances[n]
            part, other, margin = self.pairs[n]
            if not exact or (d > 0 and d >= margin):
                continue
            key = (s, self._obstacle_key(other))
            ev = self._events.get(key)
            if ev is None:
                ev = self._events[key] = {"d": d, "t": t, "parts": [], "start": t, "end": t,
                                          "other": other, "inside": False}
            ev["start"], ev["end"] = min(ev["start"], t), max(ev["end"], t)
            if part.name not in ev["parts"]:
                ev["parts"].append(part.name)
            # Choque: el primero. Si no, la distancia mínima (y si se mantiene
            # pareja, como en una recta paralela a la mesa, el primer momento).
            if d <= 0:
                if ev["d"] > 0 or t < ev["t"]:
                    ev["d"], ev["t"] = d, t
            elif d < ev["d"] - 0.5 or (d < ev["d"] and t < ev["t"]):
                ev["d"], ev["t"] = d, t
            elif d < ev["d"]:
                ev["d"] = d

    def _inside(self, q, t: float, distances) -> None:
        """¿Alguna parte apareció metida ENTERA adentro de una pieza?"""
        placed = self._pose(q)
        for n, (part, other, _margin) in enumerate(self.pairs):
            if not isinstance(other, _Solid) or distances[n][0] <= 0:
                continue
            m, center = placed[id(part)]
            if other.lower_bound(center, part.radius) > 0:
                continue
            if other.contains(_apply(m, part.vertices[0])):
                self._observe(q, t, {n: (0.0, True)}, [n])
                self._events[(self._segment[0], self._obstacle_key(other))]["inside"] = True

    @staticmethod
    def _obstacle_key(other) -> int:
        if other == FLOOR:
            return -1
        if isinstance(other, _Solid):
            return other.index
        return 1000 + other.level

    def _report(self, result: SimResult) -> CollisionReport:
        report = CollisionReport(evaluations=self.evaluations,
                                 approximate_robot=self.approximate_robot)
        for (s, _key), ev in sorted(self._events.items(), key=lambda kv: kv[1]["t"]):
            seg = result.segments[s]
            other = ev["other"]
            t0 = sum(max(0.0, x.duration_s) for x in result.segments[:s])
            duration = max(0.0, seg.duration_s)
            fraction = 0.0 if duration <= 0 else min(1.0, max(0.0, (ev["t"] - t0) / duration))
            if other == FLOOR:
                name, index = FLOOR, None
            elif isinstance(other, _Solid):
                name, index = other.name, other.index
            else:
                name, index = other.name, None
            report.contacts.append(Contact(
                segment=s, where=seg.where, kind=seg.kind, obstacle=name, obstacle_index=index,
                parts=list(ev["parts"]), distance_mm=ev["d"], time_s=ev["t"], fraction=fraction,
                start_s=ev["start"], end_s=ev["end"], inside=ev["inside"]))
        for c in report.contacts[:MAX_ISSUES]:
            report.issues.append(Issue("error" if c.colliding else "aviso", c.where,
                                       self._message(c), c.time_s))
        rest = len(report.contacts) - MAX_ISSUES
        if rest > 0:
            report.issues.append(Issue("aviso", "programa",
                                       f"… y {rest} choque(s)/cercanía(s) más sin listar."))
        if report.contacts and self.approximate_robot:
            report.issues.append(Issue(
                "info", "colisiones",
                "El robot está dibujado con cilindros aproximados (faltan las mallas del "
                "fabricante): los choques son estimaciones. Revisar en el robot real a "
                "velocidad baja."))
        return report

    def _message(self, c: Contact) -> str:
        parts = " y ".join(c.parts)
        when = f"al {c.fraction:.0%} del {c.kind}"
        robot_self = c.obstacle_index is None and c.obstacle != FLOOR
        if c.inside:
            return f"Choque: {parts} queda adentro de «{c.obstacle}» {when}"
        if c.colliding:
            if robot_self:
                return f"Choque del robot consigo mismo: {parts} contra {c.obstacle} {when}"
            target = c.obstacle if c.obstacle == FLOOR else f"«{c.obstacle}»"
            return f"Choque: {parts} contra {target} {when}"
        target = c.obstacle if c.obstacle == FLOOR else f"«{c.obstacle}»"
        return (f"{parts[:1].upper()}{parts[1:]} pasa a {c.distance_mm:.0f} mm de {target} "
                f"{when} (margen {self.margin:g} mm)")
