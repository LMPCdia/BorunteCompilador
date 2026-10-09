"""
Lo que el visor 3D necesita y se puede probar sin placa de video:

- `robot_link_meshes`: un `Mesh` por eslabón (base + J1..J6) en la posición
  cero, en coordenadas del mundo. Si el JSON del modelo trae `"meshes"` (un
  archivo por eslabón, exportado del CAD del fabricante en la posición cero),
  se usan esos; si no, piezas simples a partir de las cotas.
- `Layout`: los objetos importados para armar la celda, guardables en JSON.
- `Timeline`: la trayectoria simulada como función del tiempo.

Con producto de exponenciales, cada eslabón dibujado en la posición cero se
ubica con `model.joint_frames(q)[i]`: no hace falta ningún otro ajuste.
"""

from __future__ import annotations

import bisect
import json
import math
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path

from sim.kinematics import RobotModel, model_path
from sim.meshes import Mesh, box, cylinder, load_mesh
from sim.pad_sim import SimResult

LINK_NAMES = ["base", "J1", "J2", "J3", "J4", "J5", "J6"]


def robot_link_meshes(model: RobotModel, tool_axis: bool = True) -> list[Mesh]:
    """`tool_axis`: dibujar el eje de la herramienta en la brida (una ayuda
    visual: para los choques no cuenta)."""
    files = _mesh_files(model)
    if files:
        return [load_mesh(f) for f in files]
    return _simple_links(model, tool_axis)


def has_real_meshes(model: RobotModel) -> bool:
    """¿El modelo trae las mallas del fabricante? Si no, el robot son cilindros."""
    return _mesh_files(model) is not None


def _mesh_files(model: RobotModel) -> list[Path] | None:
    path = model_path(model.name)
    data = json.loads(path.read_text(encoding="utf-8"))
    names = data.get("meshes")
    if not names:
        return None
    files = [path.parent / model.name / n for n in names]
    return files if all(f.exists() for f in files) else None


def _simple_links(m: RobotModel, tool_axis: bool = True) -> list[Mesh]:
    z2, z3 = m.d1, m.d1 + m.a2
    zw, xw = z3 + m.a3, m.a1 + m.d4
    x4 = m.a1 + m.d4 * 0.35  # dónde empieza la parte del antebrazo que gira con J4
    base_h = m.d1 * 0.55
    flange = cylinder((xw, 0, zw), (xw + m.d6, 0, zw), 38)
    if tool_axis:
        flange.extend(cylinder((xw + m.d6, 0, zw), (xw + m.d6 + 60, 0, zw), 6))
    return [
        cylinder((0, 0, 0), (0, 0, base_h), 160).extend(box((0, 0, 15), (350, 350, 30))),
        cylinder((0, 0, base_h), (0, 0, z2), 130)
        .extend(cylinder((m.a1, -110, z2), (m.a1, 110, z2), 110)),
        cylinder((m.a1, 0, z2), (m.a1, 0, z3), 75)
        .extend(cylinder((m.a1, -95, z2), (m.a1, 95, z2), 95)),
        cylinder((m.a1, -85, z3), (m.a1, 85, z3), 85)
        .extend(cylinder((m.a1, 0, z3), (m.a1, 0, zw), 70))
        .extend(cylinder((m.a1, 0, zw), (x4, 0, zw), 70)),
        cylinder((x4, 0, zw), (xw - 40, 0, zw), 55),
        cylinder((xw, -55, zw), (xw, 55, zw), 50),
        flange,
    ]


# --- layout -------------------------------------------------------------------------


@dataclass
class LayoutObject:
    name: str
    path: str                 # archivo STEP/STL/OBJ
    x: float = 0.0            # mm, respecto de la base del robot
    y: float = 0.0
    z: float = 0.0
    rz: float = 0.0           # grados alrededor del eje vertical
    color: str = "#9aa4ad"
    # La herramienta trabaja sobre esta pieza (p. ej. la que se suelda): para la
    # herramienta solo cuenta tocarla, no acercarse. El brazo usa el margen igual.
    workpiece: bool = False


@dataclass
class Layout:
    """La celda: modelo de robot, piezas importadas, las herramientas y
    sistemas de coordenadas del pad (número -> X, Y, Z, U, V, W), y lo que hace
    falta para buscar choques: la herramienta física montada en la brida (su
    modelo 3D en coordenadas de la brida, y cómo está montada) y el margen."""

    model: str = "BRTIRUS1820A"
    objects: list[LayoutObject] = field(default_factory=list)
    tools: dict[int, list[float]] = field(default_factory=dict)
    frames: dict[int, list[float]] = field(default_factory=dict)
    tool_mesh: str = ""                       # STEP/STL/OBJ de la antorcha, pinza, etc.
    tool_mount: list[float] = field(default_factory=lambda: [0.0] * 6)
    margin_mm: float = 20.0
    collisions: bool = True                   # buscar choques al simular

    def save(self, path: str | Path) -> None:
        path = Path(path)
        data = {
            "model": self.model,
            "collisions": self.collisions,
            "margin_mm": self.margin_mm,
            "tool_mesh": _relative(self.tool_mesh, path) if self.tool_mesh else "",
            "tool_mount": list(self.tool_mount),
            # Claves como texto: JSON no tiene claves numéricas.
            "tools": {str(k): list(v) for k, v in sorted(self.tools.items())},
            "frames": {str(k): list(v) for k, v in sorted(self.frames.items())},
            "objects": [],
        }
        for obj in self.objects:
            item = asdict(obj)
            item["path"] = _relative(obj.path, path)
            data["objects"].append(item)
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "Layout":
        path = Path(path)
        data = json.loads(path.read_text(encoding="utf-8"))
        objects = []
        for item in data.get("objects", []):
            obj = LayoutObject(**item)
            if not Path(obj.path).is_absolute():
                obj.path = str(path.parent / obj.path)
            objects.append(obj)
        def poses(key: str) -> dict[int, list[float]]:
            out = {}
            for k, v in data.get(key, {}).items():
                values = [float(x) for x in v]
                if len(values) != 6 or not all(math.isfinite(x) for x in values):
                    raise ValueError(f"{key} {k}: hacen falta 6 números (X, Y, Z, U, V, W)")
                out[int(k)] = values
            return out

        mount = [float(x) for x in data.get("tool_mount", [0.0] * 6)]
        if len(mount) != 6 or not all(math.isfinite(x) for x in mount):
            raise ValueError("tool_mount: hacen falta 6 números (X, Y, Z, U, V, W)")
        margin = float(data.get("margin_mm", 20.0))
        if not (math.isfinite(margin) and 0 <= margin <= 1000):
            raise ValueError("margin_mm: tiene que ser un número entre 0 y 1000")
        tool_mesh = data.get("tool_mesh") or ""
        if tool_mesh and not Path(tool_mesh).is_absolute():
            tool_mesh = str(path.parent / tool_mesh)
        return cls(model=data.get("model", "BRTIRUS1820A"), objects=objects,
                   tools=poses("tools"), frames=poses("frames"), tool_mesh=tool_mesh,
                   tool_mount=mount, margin_mm=margin,
                   collisions=bool(data.get("collisions", True)))


def _relative(file: str, layout_path: Path) -> str:
    """Ruta relativa al layout, para poder mover la carpeta entera. relpath y no
    relative_to: también sirve con piezas en una carpeta hermana
    ("../piezas/mesa.stl"). Entre unidades de Windows no se puede: queda absoluta."""
    try:
        return os.path.relpath(Path(file).resolve(), layout_path.resolve().parent)
    except ValueError:
        return file


# --- línea de tiempo -----------------------------------------------------------------


class Timeline:
    """Ángulos de los ejes en función del tiempo, a partir de una simulación."""

    def __init__(self, result: SimResult, start_deg: list[float] | None = None) -> None:
        first = next((s.samples[0] for s in result.segments if s.samples), None)
        start = start_deg or result.start_deg or first or [0.0] * 6
        self.times: list[float] = [0.0]
        self.poses: list[list[float]] = [list(start)]
        self.labels: list[str] = [""]
        t = 0.0
        for seg in result.segments:
            label = f"{seg.where} {seg.name}".strip()
            if seg.kind == "WAIT" or len(seg.samples) < 2:
                # Quieto todo el tramo: primero ubicarse, después esperar.
                self._append(t, seg.samples[-1], label)
                t += max(0.0, seg.duration_s)
                self._append(t, seg.samples[-1], label)
                continue
            # Cada muestra en su tiempo real (cerca de una singularidad los
            # tramos chicos en el espacio pueden ser largos en el tiempo).
            times = seg.times if len(seg.times) == len(seg.samples) else [
                seg.duration_s * i / (len(seg.samples) - 1) for i in range(len(seg.samples))]
            for q, dt in zip(seg.samples, times):
                self._append(t + dt, q, label)
            t += seg.duration_s

    def _append(self, t: float, q: list[float], where: str) -> None:
        self.times.append(t)
        self.poses.append(list(q))
        self.labels.append(where)

    @property
    def duration(self) -> float:
        return self.times[-1]

    def at(self, t: float) -> tuple[list[float], str]:
        """Ángulos interpolados en `t` y la instrucción que se está ejecutando."""
        if t <= 0:
            return list(self.poses[0]), self.labels[min(1, len(self.labels) - 1)]
        if t >= self.duration:
            return list(self.poses[-1]), self.labels[-1]
        i = bisect.bisect_right(self.times, t)
        t0, t1 = self.times[i - 1], self.times[i]
        f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        q = [a + (b - a) * f for a, b in zip(self.poses[i - 1], self.poses[i])]
        return q, self.labels[i]
