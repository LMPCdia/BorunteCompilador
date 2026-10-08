"""
Simulador cinemático de un respaldo del pad: ejecuta las MISMAS acciones que
se exportan (`pad/backup.py`) sobre un modelo de robot (`sim/kinematics.py`)
y devuelve la trayectoria de los ejes más una lista de problemas.

No simula dinámica ni el suavizado del controlador: es para ver forma,
alcance, límites y orden, con un tiempo de ciclo ESTIMADO. Supuestos (ver
docs/SIMULATOR.md):

- MOVEJ interpola en ejes; cada eje va a `speed`% de su velocidad máxima y
  todos llegan juntos (el más lento manda).
- MOVEL interpola en línea recta en el espacio cartesiano, con la misma regla
  de velocidad aplicada a cada tramo (no conocemos la velocidad lineal máxima).
- Los MOVEL con herramienta o coordenadas distintas de 0 no se pueden
  simular: hace falta cargar esos marcos del pad. Se reportan y se saltean.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from pad.backup import Action, PadBackup
from sim.kinematics import Matrix, RobotModel, mat_mul, pose_error, pose_matrix, rot_axis

MOVEJ, MOVEL, WAIT, SET_OUT = 4, 10, 100, 200
SET_COORD, SET_TOOL = 800, 801
IF_INPUT_GOTO, CALL, RETURN = 10001, 20000, 20001
COMMENT, LABEL, END = 50000, 59999, 60000

# Saltos de un eje entre dos muestras de un MOVEL que delatan una
# singularidad (la muñeca "da la vuelta").
JUMP_THRESHOLD_DEG = 15.0
LINEAR_STEP_MM = 10.0
MAX_ACTIONS = 100_000


@dataclass
class Issue:
    severity: str  # "error" | "aviso"
    where: str     # "MAIN[12]" = programa y insertedIndex
    message: str


@dataclass
class Segment:
    kind: str                      # "MOVEJ" | "MOVEL" | "WAIT"
    where: str
    samples: list[list[float]]     # ángulos de los ejes, en grados
    duration_s: float
    name: str = ""


@dataclass
class SimResult:
    model: str
    segments: list[Segment] = field(default_factory=list)
    issues: list[Issue] = field(default_factory=list)
    outputs: list[tuple[float, int, bool]] = field(default_factory=list)  # (t, salida, estado)

    @property
    def total_time_s(self) -> float:
        return sum(s.duration_s for s in self.segments)

    @property
    def ok(self) -> bool:
        return not any(i.severity == "error" for i in self.issues)


def _interp_rotation(r0: Matrix, r1: Matrix, t: float) -> list[list[float]]:
    """Rotación a fracción `t` del camino más corto de r0 a r1."""
    err = pose_error(r0, r1)[3:]
    angle = math.sqrt(sum(e * e for e in err))
    if angle < 1e-12:
        return [row[:3] for row in r0[:3]]
    axis = tuple(e / angle for e in err)
    dr = rot_axis(axis, angle * t)
    return [[sum(dr[i][k] * r0[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


class PadSimulator:
    def __init__(self, model: RobotModel, inputs: dict[int, bool] | None = None,
                 start_deg: list[float] | None = None) -> None:
        self.model = model
        self.inputs = inputs or {}
        self.q = list(start_deg or [0.0] * 6)

    def run(self, backup: PadBackup) -> SimResult:
        result = SimResult(model=self.model.name)
        modules = {m.id: m for m in backup.act.modules}
        self._run_program("MAIN", backup.act.main, modules, result, depth=0)
        return result

    # -- intérprete -----------------------------------------------------------------

    def _run_program(self, name: str, actions: list[Action], modules, result: SimResult,
                     depth: int) -> int | None:
        """Ejecuta un programa. Devuelve el `flag` al que hay que saltar al
        volver (CALL con salto), o None."""
        if depth > 20:
            result.issues.append(Issue("error", name, "Más de 20 llamadas anidadas"))
            return None
        labels = {a["flag"]: i for i, a in enumerate(actions) if a["action"] == LABEL}
        pc, executed = 0, 0
        while pc < len(actions):
            executed += 1
            if executed > MAX_ACTIONS:
                result.issues.append(Issue("error", name, "Posible bucle infinito"))
                return None
            action = actions[pc]
            code = action["action"]
            where = f"{name}[{action.get('insertedIndex', '?')}]"
            pc += 1
            if code in (MOVEJ, MOVEL):
                self._move(action, where, result)
            elif code == WAIT:
                result.segments.append(
                    Segment("WAIT", where, [list(self.q)], float(action.get("limit", 0))))
            elif code == SET_OUT:
                result.outputs.append((result.total_time_s, action["point"], bool(action["pointStatus"])))
            elif code == IF_INPUT_GOTO:
                state = self.inputs.get(action["point"], False)
                if state == bool(action["pointStatus"]):
                    if action["flag"] not in labels:
                        result.issues.append(Issue("error", where, f"Etiqueta {action['flag']} inexistente"))
                        return None
                    pc = labels[action["flag"]]
            elif code == CALL:
                module = modules.get(int(action["module"]))
                if module is None:
                    result.issues.append(Issue("error", where, f"Módulo {action['module']} inexistente"))
                    continue
                self._run_program(module.name, module.actions, modules, result, depth + 1)
                # Hipótesis (docs/PAD_FORMAT.md): flag != -1 = saltar al volver.
                if action["flag"] != -1:
                    flag = int(action["flag"])
                    if flag in labels:
                        pc = labels[flag]
            elif code in (RETURN, END):
                return None
            elif code in (COMMENT, LABEL, SET_COORD, SET_TOOL):
                pass
            else:
                result.issues.append(Issue("aviso", where, f"Acción {code} desconocida: no se simula"))
        return None

    # -- movimientos ------------------------------------------------------------------

    def _move(self, action: Action, where: str, result: SimResult) -> None:
        pos = action["points"][0]["pos"]
        values = [float(pos[f"m{i}"]) for i in range(6)]
        speed = float(action.get("speed", 100)) / 100.0
        name = action.get("customName", "")

        if action["action"] == MOVEJ:
            target = values
            bad = self.model.out_of_limits(target)
            if bad:
                ejes = ", ".join(f"J{i + 1}={target[i]:.1f}°" for i in bad)
                result.issues.append(Issue("error", where, f"MOVEJ fuera de rango: {ejes}"))
            samples = self._joint_path(self.q, target)
            result.segments.append(Segment("MOVEJ", where, samples, self._duration(samples, speed), name))
            self.q = target
            return

        if action.get("toolCoord", 0) != 0:
            result.issues.append(Issue(
                "aviso", where,
                "MOVEL con herramienta/coordenadas del pad: hace falta cargar esos marcos "
                "para simularlo. Se saltea."))
            return
        target_m = pose_matrix(*values)
        samples = self._linear_path(target_m, where, result)
        if samples is None:
            return
        result.segments.append(Segment("MOVEL", where, samples, self._duration(samples, speed), name))
        self.q = samples[-1]

    def _joint_path(self, q0: list[float], q1: list[float]) -> list[list[float]]:
        n = max(2, int(max(abs(b - a) for a, b in zip(q0, q1)) / 2) + 1)
        return [[a + (b - a) * i / (n - 1) for a, b in zip(q0, q1)] for i in range(n)]

    def _linear_path(self, target: Matrix, where: str, result: SimResult) -> list[list[float]] | None:
        start = self.model.fk(self.q)
        dist = math.dist([start[i][3] for i in range(3)], [target[i][3] for i in range(3)])
        rot = math.sqrt(sum(e * e for e in pose_error(start, target)[3:]))
        n = max(2, int(max(dist / LINEAR_STEP_MM, math.degrees(rot) / 2)) + 1)
        q, samples = list(self.q), [list(self.q)]
        for i in range(1, n):
            t = i / (n - 1)
            r = _interp_rotation(start, target, t)
            p = [start[k][3] + (target[k][3] - start[k][3]) * t for k in range(3)]
            m = [r[0] + [p[0]], r[1] + [p[1]], r[2] + [p[2]], [0.0, 0.0, 0.0, 1.0]]
            nq = self.model.ik(m, q)
            if nq is None:
                result.issues.append(Issue(
                    "error", where,
                    f"MOVEL inalcanzable al {t:.0%} del recorrido (fuera de alcance o "
                    f"singularidad)"))
                return None
            jump = max(abs(a - b) for a, b in zip(nq, q))
            if jump > JUMP_THRESHOLD_DEG:
                result.issues.append(Issue(
                    "aviso", where,
                    f"Salto de {jump:.0f}° en un eje al {t:.0%} del MOVEL: probable "
                    f"singularidad de muñeca"))
            bad = self.model.out_of_limits(nq)
            if bad:
                result.issues.append(Issue(
                    "error", where,
                    "MOVEL fuera de rango en " + ", ".join(f"J{b + 1}" for b in bad)
                    + f" al {t:.0%} del recorrido"))
                return None
            q = nq
            samples.append(q)
        return samples

    def _duration(self, samples: list[list[float]], speed: float) -> float:
        speeds = [j.max_speed_dps * max(speed, 1e-3) for j in self.model.joints]
        return sum(
            max(abs(b - a) / v for a, b, v in zip(s0, s1, speeds))
            for s0, s1 in zip(samples, samples[1:])
        )


def simulate(backup: PadBackup, model: RobotModel, inputs: dict[int, bool] | None = None,
             start_deg: list[float] | None = None) -> SimResult:
    return PadSimulator(model, inputs, start_deg).run(backup)
