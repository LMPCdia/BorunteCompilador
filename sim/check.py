"""
Simula un programa contra un modelo de robot y lista los problemas.

    python -m sim.check programa.krlb
    python -m sim.check HCBackupRobot_20260814213843.zip --model BRTIRUS1820A
    python -m sim.check programa.krlb --input X012=1 --input X013=0
    python -m sim.check programa.krlb --layout celda.layout.json   # + choques

Acepta el fuente del DSL (lo compila para el pad) o un respaldo ya exportado.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from compiler.codegen import CompileError
from compiler.pad_codegen import PadOptions, compile_to_pad_report, io_point
from pad.backup import PadBackup
from sim import collision
from sim.kinematics import RobotModel, rot_axis
from sim.meshes import MeshError, load_mesh
from sim.pad_sim import SimResult, simulate
from sim.scene import Layout, has_real_meshes, robot_link_meshes

DEFAULT_MODEL = "BRTIRUS1510A"  # el robot de la celda


def format_result(result: SimResult) -> list[str]:
    lines = [f"Modelo: {result.model}"]
    for seg in result.segments:
        name = f"  ({seg.name})" if seg.name else ""
        lines.append(f"  {seg.where:<22} {seg.kind:<6} {seg.duration_s:6.2f} s{name}")
    lines.append(f"Tiempo de ciclo estimado: {result.total_time_s:.1f} s "
                 f"(sin aceleraciones ni suavizado)")
    if result.issues:
        lines.append("Problemas:")
        lines += [f"  [{i.severity}] {i.where}: {i.message}" for i in result.issues]
    else:
        lines.append("Sin problemas detectados.")
    return lines


def check_collisions(result: SimResult, model: RobotModel, layout: Layout) -> list[str]:
    """Busca choques con la celda; devuelve avisos sobre lo que no se pudo revisar."""
    notes = []
    if not collision.available():
        return ["Choques sin revisar: falta python-fcl."]
    obstacles = []
    for obj in layout.objects:
        try:
            mesh = load_mesh(obj.path)
        except (MeshError, OSError) as e:
            notes.append(f"Choques: «{obj.name}» no se revisa ({e})")
            continue
        r = rot_axis((0, 0, 1), math.radians(obj.rz))
        matrix = [r[0] + [obj.x], r[1] + [obj.y], r[2] + [obj.z], [0.0, 0.0, 0.0, 1.0]]
        obstacles.append(collision.Obstacle(obj.name, mesh, matrix, obj.workpiece))
    tool = None
    if layout.tool_mesh:
        try:
            tool = load_mesh(layout.tool_mesh)
        except (MeshError, OSError) as e:
            notes.append(f"Choques: la herramienta no se revisa ({e})")
    checker = collision.CollisionChecker(
        model, robot_link_meshes(model, tool_axis=False), obstacles, tool_mesh=tool,
        tool_mount=layout.tool_mount, margin_mm=layout.margin_mm,
        approximate_robot=not has_real_meshes(model), base=layout.base_matrix())
    result.issues.extend(checker.check(result).issues)
    return notes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m sim.check", description=__doc__.split("\n")[1])
    parser.add_argument("archivo", help=".krlb o HCBackupRobot_*.zip")
    parser.add_argument("--model", choices=RobotModel.available(),
                        help="por defecto, el de la celda (--layout) o el BRTIRUS1510A")
    parser.add_argument("--input", action="append", default=[], metavar="X012=1",
                        help="estado de una entrada (se puede repetir)")
    parser.add_argument("--layout", metavar="CELDA.layout.json",
                        help="celda guardada en la app: herramientas, coordenadas y piezas "
                             "(busca choques)")
    args = parser.parse_args(argv)

    path = Path(args.archivo)
    try:
        if path.suffix.lower() == ".zip":
            backup = PadBackup.read(path)
        else:
            # Simular no tiene riesgo: se permite lo "sin confirmar".
            backup, warnings = compile_to_pad_report(
                path.read_text(encoding="utf-8"), PadOptions(allow_unverified=True))
            for warning in warnings:
                print(f"Aviso: {warning}")
        inputs = {}
        for item in args.input:
            name, _, value = item.partition("=")
            inputs[io_point(name.strip().upper(), "X")] = value.strip().upper() not in ("0", "OFF", "")
    except CompileError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2
    except (OSError, ValueError) as e:
        print(f"Error: no se pudo leer {path}: {e}", file=sys.stderr)
        return 2

    layout = None
    if args.layout:
        try:
            layout = Layout.load(args.layout)
        except (OSError, ValueError, TypeError) as e:
            print(f"Error: no se pudo leer la celda {args.layout}: {e}", file=sys.stderr)
            return 2
    name = args.model or (layout.model if layout is not None else DEFAULT_MODEL)
    if name not in RobotModel.available():
        print(f"Error: el robot {name} no está instalado", file=sys.stderr)
        return 2
    model = RobotModel.load(name)
    if layout is None:
        result = simulate(backup, model, inputs)
    else:
        result = simulate(backup, model, inputs, tools=layout.tools, frames=layout.frames)
        if layout.collisions:
            for note in check_collisions(result, model, layout):
                print(note)
    print("\n".join(format_result(result)))
    return 0 if result.ok else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
