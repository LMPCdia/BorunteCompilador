"""
Curvas de movimiento de una simulación: posición, velocidad y aceleración
de cada eje, y velocidad y aceleración lineal de la punta de la herramienta.

Se muestrea la línea de tiempo (`sim.scene.Timeline`) a paso fijo y se
deriva numéricamente, con un suavizado corto (`SMOOTH_S`) para que las
esquinas de la interpolación lineal entre muestras no se vean como picos de
aceleración que el robot no tiene.

OJO con lo que significan los números (ver docs/SIMULATOR.md):
- velocidades: las del modelo (velocidad máxima de cada eje × SPEED %);
- aceleraciones: salen del perfil de velocidad SUPUESTO del simulador
  (`accel_s`). Con `accel_s` = 0 no hay perfil y las aceleraciones no
  significan nada (se marcan como no disponibles).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from sim.kinematics import Matrix, RobotModel, identity, mat_mul
from sim.scene import Timeline

MAX_POINTS = 6000      # puntos por curva: más no se ven y hacen lento el gráfico
MIN_DT = 0.005         # s
SMOOTH_S = 0.04        # ventana de la derivada central


@dataclass
class MotionCurves:
    t: list[float] = field(default_factory=list)
    angle: list[list[float]] = field(default_factory=list)     # [eje][i] grados
    speed: list[list[float]] = field(default_factory=list)     # °/s
    accel: list[list[float]] = field(default_factory=list)     # °/s²
    tcp_speed: list[float] = field(default_factory=list)       # mm/s
    tcp_accel: list[float] = field(default_factory=list)       # mm/s² (módulo)
    accelerations_valid: bool = False

    def peaks(self, model: RobotModel) -> list[dict]:
        """Máximos por eje, comparados con el límite del modelo."""
        out = []
        for j, joint in enumerate(model.joints):
            vmax = max((abs(v) for v in self.speed[j]), default=0.0)
            out.append({
                "eje": f"J{j + 1}",
                "min_deg": min(self.angle[j], default=0.0),
                "max_deg": max(self.angle[j], default=0.0),
                "v_max": vmax,
                "v_pct": 100.0 * vmax / joint.max_speed_dps if joint.max_speed_dps else 0.0,
                "a_max": max((abs(a) for a in self.accel[j]), default=0.0),
            })
        return out


def analyze(timeline: Timeline, model: RobotModel, tools: dict[int, Matrix] | None = None,
            accel_s: float | None = 0.0) -> MotionCurves:
    """`tools`: matrices de las herramientas (número -> brida->punta)."""
    tools = tools or {}
    duration = timeline.duration
    # None = aceleraciones del modelo (datasheet).
    curves = MotionCurves(accelerations_valid=(accel_s is None and model.has_accelerations)
                          or (accel_s is not None and accel_s > 0))
    if duration <= 0:
        return curves
    dt = max(MIN_DT, duration / MAX_POINTS)
    n = int(duration / dt) + 1
    ts = [min(duration, i * dt) for i in range(n)]
    if ts[-1] < duration:
        ts.append(duration)
    qs = [timeline.at(t)[0] for t in ts]
    tips = []
    for t, q in zip(ts, qs):
        m = mat_mul(model.fk(q), tools.get(timeline.tool_at(t), identity()))
        tips.append((m[0][3], m[1][3], m[2][3]))

    k = max(1, round(SMOOTH_S / dt / 2))       # medio ancho de la ventana, en muestras
    # Tramos continuos: la derivada no cruza un salto del simulador.
    piece, jumps, j = [], timeline.jumps(), 0
    for t in ts:
        while j < len(jumps) and jumps[j] <= t:
            j += 1
        piece.append(j)
    curves.t = ts
    curves.angle = [[q[i] for q in qs] for i in range(6)]
    curves.speed = [_derivative(ts, a, k, piece) for a in curves.angle]
    curves.accel = [_derivative(ts, v, k, piece) for v in curves.speed]
    vel = [_derivative(ts, [p[c] for p in tips], k, piece) for c in range(3)]
    curves.tcp_speed = [math.sqrt(vx * vx + vy * vy + vz * vz) for vx, vy, vz in zip(*vel)]
    acc = [_derivative(ts, v, k, piece) for v in vel]
    curves.tcp_accel = [math.sqrt(ax * ax + ay * ay + az * az) for ax, ay, az in zip(*acc)]
    return curves


def _derivative(ts: list[float], ys: list[float], k: int, piece: list[int]) -> list[float]:
    """Derivada central sobre ±k muestras, sin salirse del tramo continuo
    (`piece`); en los bordes la ventana se achica."""
    n = len(ys)
    out = []
    for i in range(n):
        a, b = max(0, i - k), min(n - 1, i + k)
        while piece[a] != piece[i]:
            a += 1
        while piece[b] != piece[i]:
            b -= 1
        span = ts[b] - ts[a]
        out.append(0.0 if span <= 0 else (ys[b] - ys[a]) / span)
    return out
