"""
Modelo de robot a partir del ensamble STEP del fabricante.

El ensamble viene en una pose cualquiera (el del 1510A tiene J1 en -1.3°,
J2 en 3.5°, J4 en 72°...) y en coordenadas del CAD (Y para arriba). El
simulador necesita cada eslabón en la POSICIÓN CERO del producto de
exponenciales (ver `sim/kinematics.py`): brazo vertical, antebrazo
horizontal hacia +X, brida mirando a +X, Z para arriba, origen en el eje de
J1 a la altura del apoyo de la base.

Cómo:
1. Se ubican los 6 ejes en el ensamble, cada uno con un cilindro de la pieza
   que gira con él (una "receta" por familia de robots: qué parte es cada
   eslabón y en qué cilindro de la parte está cada eje).
2. Se "desgira" eje por eje, de J1 a J6: cada giro mueve todos los
   eslabones y ejes de ahí en adelante, hasta que se cumple la posición cero.
   Así es exactamente la inversa del producto de exponenciales.
3. De la posición cero salen las cotas (d1, a1, a2, a3, d4, d6).

Lo medido del CAD es geometría del fabricante. Lo que NO sale del CAD y es
hipótesis: el cero de J6 (la brida es casi simétrica: se toma el giro que
deja los ejes de la pieza alineados), los sentidos de giro y los rangos.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

from sim.kinematics import Matrix, identity, mat_mul, rot_axis
from sim.meshes import Mesh, write_stl
from sim.step_assembly import Part, _apply

Vec = list[float]

# Familia BRTIRUS (Borunte): el código de cada parte termina en A000..F000 y la
# brida no tiene código. Los ejes están en coordenadas de cada parte; salen de
# sus cilindros más grandes (rodamientos), medidos en el 1510A. Para otro
# modelo de la familia hay que verificarlos (ver docs/SIMULATOR.md).
BORUNTE_RECIPE = {
    "links": {"base": "A000", "J1": "B000", "J2": "C000", "J3": "D000", "J4": "E000",
              "J5": "F000", "J6": None},
    # eje: (parte, dirección local, punto local)
    "axes": {
        "J1": ("A000", (0, 1, 0), (0, 0, 0)),
        "J2": ("B000", (0, 0, 1), (170, -179, 0)),
        "J3": ("D000", (0, 0, 1), (42, -43, 0)),
        "J4": ("E000", (1, 0, 0), (0, 5, -5)),
        "J5": ("F000", (1, 0, 0), (0, 0, 0)),
        "J6": ("F000", (0, 0, 1), (0, 0, 0)),
    },
}


# --- vectores ----------------------------------------------------------------------


def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _unit(v):
    n = math.sqrt(_dot(v, v)) or 1.0
    return [x / n for x in v]


def _perp(v, axis):
    return _sub(v, [axis[i] * _dot(v, axis) for i in range(3)])


def _angle_to(v, target, axis) -> float:
    """Giro alrededor de `axis` que lleva la dirección de `v` a la de `target`."""
    v, t = _unit(_perp(v, axis)), _unit(_perp(target, axis))
    return math.atan2(_dot(axis, _cross(v, t)), _dot(v, t))


def _rotation(axis: Vec, point: Vec, angle: float) -> Matrix:
    r = rot_axis(tuple(axis), angle)
    t = [point[i] - sum(r[i][k] * point[k] for k in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def _dir(m: Matrix, d) -> Vec:
    return [sum(m[i][k] * d[k] for k in range(3)) for i in range(3)]


# --- conversión ----------------------------------------------------------------------


@dataclass
class ImportedRobot:
    links: list[Mesh]                 # base, J1..J6 en la posición cero
    geometry: dict[str, float]        # d1, a1, a2, a3, d4, d6 (mm)
    cad_pose_deg: list[float]         # en qué pose estaba el ensamble (ángulos geométricos)
    notes: list[str] = field(default_factory=list)

    @property
    def reach_mm(self) -> float:
        g = self.geometry
        return g["a1"] + g["a2"] + math.hypot(g["d4"], g["a3"])


def _find(parts: list[Part], code: str | None) -> Part:
    if code is None:
        rest = [p for p in parts if not p.name.startswith("PBR")]
        if len(rest) != 1:
            raise ValueError(f"no se pudo identificar la brida entre {[p.name for p in parts]}")
        return rest[0]
    found = [p for p in parts if code in p.name]
    if len(found) != 1:
        raise ValueError(f"se esperaba una parte con «{code}» y hay {len(found)}")
    return found[0]


def import_robot(parts: list[Part], recipe: dict = BORUNTE_RECIPE,
                 up_axis: str = "Y") -> ImportedRobot:
    """`parts`: salida de `sim.step_assembly.split_assembly` (coordenadas del ensamble)."""
    names = ["base", "J1", "J2", "J3", "J4", "J5", "J6"]
    by_link = {k: _find(parts, recipe["links"][k]) for k in names}
    by_code = {p.name: p for p in parts}

    # CAD -> robot: Z para arriba (si el CAD tiene Y arriba, giro de 90° en X).
    if up_axis == "Y":
        to_robot = [[1.0, 0, 0, 0], [0, 0, -1.0, 0], [0, 1.0, 0, 0], [0, 0, 0, 1.0]]
    elif up_axis == "Z":
        to_robot = identity()
    else:
        raise ValueError("up_axis tiene que ser Y o Z")

    # Cada eslabón como (malla, matriz acumulada); se mueven las matrices y al
    # final se aplica una vez a cada malla.
    moves = {k: to_robot for k in names}
    axes = []
    for j in ["J1", "J2", "J3", "J4", "J5", "J6"]:
        code, d, p = recipe["axes"][j]
        part = next(pp for name, pp in by_code.items() if code in name)
        axes.append((_unit(_dir(to_robot, _dir(part.matrix, d))),
                     list(_apply(to_robot, _apply(part.matrix, p)))))

    # Apoyo de la base = z mínima de la base: el origen va ahí, sobre el eje J1.
    zmin = min(_apply(to_robot, v)[2] for tri in by_link["base"].mesh.triangles for v in tri)
    a1d, a1p = axes[0]
    shift = [[1.0, 0, 0, -a1p[0]], [0, 1.0, 0, -a1p[1]], [0, 0, 1.0, -zmin], [0, 0, 0, 1.0]]
    moves = {k: mat_mul(shift, m) for k, m in moves.items()}
    axes = [(d, list(_apply(shift, p))) for d, p in axes]
    if abs(axes[0][0][2]) < 0.999:
        raise ValueError("el eje de J1 no es vertical: revisar up_axis")
    axes[0] = ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0])

    # Condiciones de la posición cero, eje por eje.
    def target(i: int):
        if i == 0:   # J2 a lo largo de Y
            return axes[1][0], [0.0, math.copysign(1.0, axes[1][0][1] or 1.0), 0.0]
        if i == 1:   # J3 justo arriba de J2
            return _sub(axes[2][1], axes[1][1]), [0.0, 0.0, 1.0]
        if i == 2:   # J4 horizontal hacia +X
            return _forward(axes[3][0]), [1.0, 0.0, 0.0]
        if i == 3:   # J5 a lo largo de Y
            return axes[4][0], [0.0, math.copysign(1.0, axes[4][0][1] or 1.0), 0.0]
        if i == 4:   # brida hacia +X
            return _forward(axes[5][0]), [1.0, 0.0, 0.0]
        return None

    canonical = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]
    cad_pose = []
    for i in range(6):
        axis_dir, axis_point = axes[i]
        # Con los ejes anteriores ya desgirados, este quedó en su dirección de
        # la posición cero (o al revés: el cilindro no dice el sentido). Con
        # el sentido del modelo, la pose del CAD sale como la del simulador.
        if _dot(axis_dir, canonical[i]) < 0:
            axis_dir = [-x for x in axis_dir]
        if i < 5:
            v, t = target(i)
            angle = _angle_to(v, t, axis_dir)
        else:
            angle = _flange_zero(moves["J6"], by_link["J6"].matrix, axis_dir)
        undo = _rotation(axis_dir, axis_point, angle)
        for k in names[i + 1:]:
            moves[k] = mat_mul(undo, moves[k])
        axes = axes[:i + 1] + [(_unit(_dir(undo, d)), list(_apply(undo, p))) for d, p in axes[i + 1:]]
        cad_pose.append(-math.degrees(angle))

    notes = []
    # El brazo tiene que quedar en el plano XZ. Un ensamble con uniones
    # "concéntricas" deja deslizar una parte a lo largo del eje: se corrige y
    # se avisa (ver docs/SIMULATOR.md).
    lateral = axes[3][1][1]
    if abs(lateral) > 0.5:
        fix = [[1.0, 0, 0, 0], [0, 1.0, 0, -lateral], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
        for k in names[3:]:
            moves[k] = mat_mul(fix, moves[k])
        axes = axes[:3] + [(d, list(_apply(fix, p))) for d, p in axes[3:]]
        notes.append(f"El antebrazo estaba corrido {lateral:+.1f} mm a lo largo del eje de J3 "
                     f"en el ensamble: se centró en el plano del brazo.")

    links = []
    for k in names:
        mesh = Mesh()
        for tri in by_link[k].mesh.triangles:
            mesh.add(*(_apply(moves[k], v) for v in tri))
        links.append(mesh)

    p2, p3 = axes[1][1], axes[2][1]
    wrist = axes[4][1]
    flange_x = max(v[0] for tri in links[6].triangles for v in tri)
    geometry = {
        "d1": p2[2],
        "a1": p2[0],
        "a2": p3[2] - p2[2],
        "a3": axes[3][1][2] - p3[2],
        "d4": wrist[0] - p3[0],
        "d6": flange_x - wrist[0],
    }
    geometry = {k: round(float(v), 1) for k, v in geometry.items()}
    return ImportedRobot(links, geometry, [round(float(a), 2) for a in cad_pose], notes)


def _forward(d: Vec) -> Vec:
    """Una dirección de eje con el sentido que apunta hacia adelante (+X)."""
    return d if d[0] >= 0 else [-x for x in d]


def _flange_zero(move: Matrix, part_matrix: Matrix, axis: Vec) -> float:
    """Cero de J6: la brida es casi simétrica, así que no hay una marca en el
    CAD. Se toma el giro que deja los ejes propios de la pieza alineados con
    los del robot (múltiplo de 90°). HIPÓTESIS: confirmar con el pad."""
    m = mat_mul(move, part_matrix)
    best = None
    for k in range(3):
        local = [0.0, 0.0, 0.0]
        local[k] = 1.0
        d = _dir(m, local)
        if abs(_dot(d, axis)) > 0.9:
            continue
        angle = _angle_to(d, [0.0, 0.0, 1.0], axis)
        snapped = angle - round(angle / (math.pi / 2)) * (math.pi / 2)
        if best is None or abs(snapped) < abs(best):
            best = snapped
    return best or 0.0


def write_model(robot: ImportedRobot, name: str, folder: Path, joints: list[dict],
                notes: str) -> Path:
    """`<folder>/<name>.json` + `<folder>/<name>/{base,j1..j6}.stl`."""
    files = ["base.stl"] + [f"j{i}.stl" for i in range(1, 7)]
    (folder / name).mkdir(parents=True, exist_ok=True)
    for mesh, file in zip(robot.links, files):
        write_stl(mesh, folder / name / file)
    data = {
        "name": name,
        "notes": notes,
        "reach_mm": round(robot.reach_mm, 1),
        "geometry_mm": robot.geometry,
        "joints": joints,
        "meshes": files,
    }
    path = folder / f"{name}.json"
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


# Grilla para simplificar cada eslabón (mm): más fina en los chicos (muñeca y
# brida) para que no se deformen.
SIMPLIFY_MM = [6.0, 6.0, 6.0, 6.0, 6.0, 3.0, 2.0]


def import_robot_step(step: str | Path, name: str, out_dir: str | Path, joints_from: str,
                      progress=None) -> tuple[Path, ImportedRobot]:
    """Todo el camino: ensamble STEP -> `<out_dir>/<name>.json` + mallas.
    Tira ValueError, OSError o MeshError con un mensaje para el usuario."""
    from sim.kinematics import RobotModel, model_path
    from sim.meshes import simplify
    from sim.step_assembly import split_assembly

    if not re.fullmatch(r"[A-Za-z0-9_.-]+", name):
        raise ValueError(f"nombre de modelo inválido: {name!r}")
    source = json.loads(model_path(joints_from).read_text(encoding="utf-8"))
    RobotModel.load(joints_from)
    parts = split_assembly(step, progress=progress)
    robot = import_robot(parts)
    robot.links = [simplify(m, c) for m, c in zip(robot.links, SIMPLIFY_MM)]
    skipped = sum(p.mesh.skipped_faces for p in parts)
    notes = (f"Cotas y mallas medidas del ensamble STEP del fabricante ({Path(step).name}); "
             f"pose del ensamble {robot.cad_pose_deg}. {' '.join(robot.notes)} "
             f"Rangos, velocidades y sentidos de giro copiados de {joints_from}: HIPÓTESIS, "
             f"reemplazar por la tabla del robot. Cero de J6: hipótesis (la brida es simétrica).")
    if skipped:
        notes += f" {skipped} cara(s) del CAD no se pudieron mallar y faltan."
    path = write_model(robot, name, Path(out_dir), source["joints"], notes)
    return path, robot


def main(argv: list[str] | None = None) -> int:
    """python -m sim.robot_import ENSAMBLE.step --name BRTIRUSxxxxA --joints-from BRTIRUS1510A"""
    import argparse
    import sys

    from sim.kinematics import USER_MODELS_DIR
    from sim.meshes import MeshError

    parser = argparse.ArgumentParser(prog="python -m sim.robot_import",
                                     description="Modelo de robot desde el ensamble STEP del fabricante")
    parser.add_argument("step", help="ensamble STEP del robot (familia BRTIRUS)")
    parser.add_argument("--name", required=True, help="nombre del modelo, p. ej. BRTIRUS1510A")
    parser.add_argument("--joints-from", required=True, metavar="MODELO",
                        help="modelo del que se copian rangos, velocidades y sentidos de giro "
                             "(HIPÓTESIS hasta tener la tabla del robot: editar el JSON)")
    parser.add_argument("--out", default=str(USER_MODELS_DIR),
                        help=f"carpeta de modelos (por defecto {USER_MODELS_DIR})")
    args = parser.parse_args(argv)

    try:
        print(f"Leyendo {args.step} (puede tardar unos minutos)…")
        path, robot = import_robot_step(args.step, args.name, args.out, args.joints_from,
                                        progress=lambda i, n, name: print(f"  {i + 1}/{n} {name}"))
    except (OSError, ValueError, MeshError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2
    g = robot.geometry
    print(f"Listo: {path}")
    print(f"  d1={g['d1']} a1={g['a1']} a2={g['a2']} a3={g['a3']} d4={g['d4']} d6={g['d6']} "
          f"alcance={robot.reach_mm:.1f} mm")
    for note in robot.notes:
        print(f"  {note}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
