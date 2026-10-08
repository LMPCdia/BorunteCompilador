"""
Simula un programa contra un modelo de robot y lista los problemas.

    python -m sim.check programa.krlb
    python -m sim.check HCBackupRobot_20260814213843.zip --model BRTIRUS1820A
    python -m sim.check programa.krlb --input X012=1 --input X013=0

Acepta el fuente del DSL (lo compila para el pad) o un respaldo ya exportado.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from compiler.pad_codegen import compile_to_pad, io_point
from pad.backup import PadBackup
from sim.kinematics import RobotModel
from sim.pad_sim import SimResult, simulate


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


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m sim.check", description=__doc__.split("\n")[1])
    parser.add_argument("archivo", help=".krlb o HCBackupRobot_*.zip")
    parser.add_argument("--model", default="BRTIRUS1820A", choices=RobotModel.available())
    parser.add_argument("--input", action="append", default=[], metavar="X012=1",
                        help="estado de una entrada (se puede repetir)")
    args = parser.parse_args(argv)

    path = Path(args.archivo)
    if path.suffix.lower() == ".zip":
        backup = PadBackup.read(path)
    else:
        backup = compile_to_pad(path.read_text(encoding="utf-8"))
    inputs = {}
    for item in args.input:
        name, _, value = item.partition("=")
        inputs[io_point(name, "X")] = value.strip() not in ("0", "OFF", "off", "")

    result = simulate(backup, RobotModel.load(args.model), inputs)
    print("\n".join(format_result(result)))
    return 0 if result.ok else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
