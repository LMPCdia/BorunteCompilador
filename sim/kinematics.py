"""
Cinemática de los brazos Borunte de 6 ejes, en Python puro (sin numpy: no es
dependencia del proyecto).

Modelo geométrico (todos los modelos BRTIRUS que vimos tienen esta forma):

    J1 vertical en el origen; J2 desplazado `a1` hacia adelante y `d1` arriba;
    brazo `a2` hasta J3; codo con desplazamiento `a3` hacia arriba; antebrazo
    `d4` hasta el centro de la muñeca; muñeca esférica (J4, J5, J6) y brida a
    `d6` del centro de la muñeca.

En la posición cero el brazo está vertical y el antebrazo horizontal hacia +X,
como en el plano del fabricante. Se usa "producto de exponenciales": cada eje
es una rotación alrededor de una recta fija dada en la posición cero, lo que
deja el modelo legible y fácil de corregir si un eje gira al revés.

Qué es hipótesis (ver docs/SIMULATOR.md):
- que el cero de cada eje del pad sea esa posición del plano;
- el sentido de giro de cada eje (`sign` en el JSON del modelo);
- la convención de orientación U, V, W del pad (acá: ángulos fijos X-Y-Z,
  R = Rz(W) · Ry(V) · Rx(U)).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parent / "models"

Matrix = list[list[float]]  # 4x4 homogénea
Vec = tuple[float, float, float]


# --- álgebra mínima -----------------------------------------------------------


def mat_mul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def identity() -> Matrix:
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def rot_axis(axis: Vec, angle: float) -> list[list[float]]:
    """Matriz 3x3 de rotación alrededor de un eje unitario (Rodrigues)."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def screw(axis: Vec, point: Vec, angle: float) -> Matrix:
    """Rotación `angle` alrededor de la recta (axis, point), como 4x4."""
    r = rot_axis(axis, angle)
    px, py, pz = point
    t = [
        px - (r[0][0] * px + r[0][1] * py + r[0][2] * pz),
        py - (r[1][0] * px + r[1][1] * py + r[1][2] * pz),
        pz - (r[2][0] * px + r[2][1] * py + r[2][2] * pz),
    ]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def pose_matrix(x: float, y: float, z: float, u: float, v: float, w: float) -> Matrix:
    """X, Y, Z (mm) + U, V, W (grados) -> 4x4. Convención R = Rz(W)·Ry(V)·Rx(U)
    (hipótesis: no está confirmada para el pad)."""
    rx = rot_axis((1, 0, 0), math.radians(u))
    ry = rot_axis((0, 1, 0), math.radians(v))
    rz = rot_axis((0, 0, 1), math.radians(w))

    def m3(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    r = m3(rz, m3(ry, rx))
    return [r[0] + [x], r[1] + [y], r[2] + [z], [0.0, 0.0, 0.0, 1.0]]


def matrix_to_pose(m: Matrix) -> tuple[float, float, float, float, float, float]:
    """Inversa de `pose_matrix`."""
    r = m
    v = math.asin(max(-1.0, min(1.0, -r[2][0])))
    if abs(math.cos(v)) > 1e-9:
        u = math.atan2(r[2][1], r[2][2])
        w = math.atan2(r[1][0], r[0][0])
    else:  # gimbal lock: se fija U = 0
        u = 0.0
        w = math.atan2(-r[0][1], r[1][1])
    return (m[0][3], m[1][3], m[2][3], math.degrees(u), math.degrees(v), math.degrees(w))


def pose_error(current: Matrix, target: Matrix) -> list[float]:
    """Error (dx, dy, dz en mm; rx, ry, rz en rad) de `current` a `target`,
    en coordenadas del mundo."""
    dp = [target[i][3] - current[i][3] for i in range(3)]
    # R_err = R_t · R_cᵀ, y su vector de rotación.
    re = [[sum(target[i][k] * current[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    cos_a = max(-1.0, min(1.0, (re[0][0] + re[1][1] + re[2][2] - 1) / 2))
    angle = math.acos(cos_a)
    vec = [re[2][1] - re[1][2], re[0][2] - re[2][0], re[1][0] - re[0][1]]
    if angle < 1e-9:
        rot = [0.0, 0.0, 0.0]
    elif math.pi - angle < 1e-6:
        # cerca de 180°: el vector de arriba se anula; eje de la diagonal
        axis = [math.sqrt(max(0.0, (re[i][i] + 1) / 2)) for i in range(3)]
        rot = [a * angle for a in axis]
    else:
        k = angle / (2 * math.sin(angle))
        rot = [k * c for c in vec]
    return dp + rot


def solve_linear(a: list[list[float]], b: list[float]) -> list[float]:
    """Gauss con pivoteo parcial; `a` cuadrada."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        if abs(m[col][col]) < 1e-12:
            raise ZeroDivisionError("matriz singular")
        for r in range(n):
            if r != col:
                f = m[r][col] / m[col][col]
                for c in range(col, n + 1):
                    m[r][c] -= f * m[col][c]
    return [m[i][n] / m[i][i] for i in range(n)]


# --- modelo del robot ------------------------------------------------------------


@dataclass(frozen=True)
class Joint:
    min_deg: float
    max_deg: float
    max_speed_dps: float
    sign: int = 1  # sentido de giro respecto del modelo (hipótesis)


@dataclass(frozen=True)
class RobotModel:
    name: str
    d1: float
    a1: float
    a2: float
    a3: float
    d4: float
    d6: float
    joints: tuple[Joint, ...]
    reach_mm: float | None = None
    notes: str = ""

    @classmethod
    def load(cls, name: str) -> "RobotModel":
        data = json.loads((MODELS_DIR / f"{name}.json").read_text(encoding="utf-8"))
        g = data["geometry_mm"]
        joints = tuple(
            Joint(j["min_deg"], j["max_deg"], j["max_speed_dps"], j.get("sign", 1))
            for j in data["joints"]
        )
        return cls(name=data["name"], joints=joints, reach_mm=data.get("reach_mm"),
                   notes=data.get("notes", ""), **{k: g[k] for k in ("d1", "a1", "a2", "a3", "d4", "d6")})

    @staticmethod
    def available() -> list[str]:
        return sorted(p.stem for p in MODELS_DIR.glob("*.json"))

    # -- geometría en la posición cero ----------------------------------------------

    def _screws(self) -> list[tuple[Vec, Vec]]:
        z_elbow = self.d1 + self.a2
        z_wrist = z_elbow + self.a3
        x_wrist = self.a1 + self.d4
        return [
            ((0, 0, 1), (0, 0, 0)),                   # J1
            ((0, 1, 0), (self.a1, 0, self.d1)),       # J2
            ((0, 1, 0), (self.a1, 0, z_elbow)),       # J3
            ((1, 0, 0), (self.a1, 0, z_wrist)),       # J4
            ((0, 1, 0), (x_wrist, 0, z_wrist)),       # J5
            ((1, 0, 0), (x_wrist, 0, z_wrist)),       # J6
        ]

    def _home_flange(self) -> Matrix:
        # Brida mirando hacia +X: z de la herramienta = +X del mundo.
        z_wrist = self.d1 + self.a2 + self.a3
        x_flange = self.a1 + self.d4 + self.d6
        return [[0, 0, 1, x_flange], [0, 1, 0, 0], [-1, 0, 0, z_wrist], [0, 0, 0, 1]]

    # -- cinemática directa -------------------------------------------------------------

    def joint_frames(self, q_deg: list[float]) -> list[Matrix]:
        """Transformaciones acumuladas después de cada eje (para dibujar cada
        eslabón): la i-ésima mueve el eslabón que cuelga del eje i+1."""
        frames, t = [], identity()
        for (axis, point), joint, q in zip(self._screws(), self.joints, q_deg):
            t = mat_mul(t, screw(axis, point, math.radians(joint.sign * q)))
            frames.append(t)
        return frames

    def fk(self, q_deg: list[float]) -> Matrix:
        """Brida en el mundo para los ángulos del pad (grados)."""
        return mat_mul(self.joint_frames(q_deg)[-1], self._home_flange())

    def wrist_center(self, q_deg: list[float]) -> Vec:
        """El "P point" del plano del fabricante."""
        t = self.joint_frames(q_deg)[-1]
        z_wrist = self.d1 + self.a2 + self.a3
        p = (self.a1 + self.d4, 0.0, z_wrist)
        return tuple(sum(t[i][k] * p[k] for k in range(3)) + t[i][3] for i in range(3))

    # -- límites ----------------------------------------------------------------------------

    def out_of_limits(self, q_deg: list[float], tol: float = 1e-6) -> list[int]:
        """Índices (0-based) de los ejes fuera de rango."""
        return [i for i, (j, q) in enumerate(zip(self.joints, q_deg))
                if q < j.min_deg - tol or q > j.max_deg + tol]

    # -- cinemática inversa -------------------------------------------------------------------

    def ik(self, target: Matrix, seed_deg: list[float], max_iter: int = 200,
           tol_mm: float = 0.01, tol_rad: float = 1e-5) -> list[float] | None:
        """Ángulos que llevan la brida a `target`, buscando cerca de `seed_deg`
        (Levenberg-Marquardt). Devuelve None si no converge: punto fuera de
        alcance, o demasiado lejos de la semilla.

        Se busca cerca de la semilla a propósito: es lo que hace el robot al
        seguir una recta, y evita saltar a otra configuración del brazo. El
        amortiguamiento es adaptativo porque cerca de una singularidad (J5 ≈ 0,
        J4 y J6 alineados) un paso fijo oscila y no converge.
        """
        rot_weight = 200.0  # mm por radián, para mezclar unidades

        def weighted_error(q):
            err = pose_error(self.fk(q), target)
            return err, err[:3] + [x * rot_weight for x in err[3:]]

        q = list(seed_deg)
        err, e = weighted_error(q)
        cost = sum(x * x for x in e)
        lam = 1.0
        for _ in range(max_iter):
            if max(abs(x) for x in err[:3]) < tol_mm and max(abs(x) for x in err[3:]) < tol_rad:
                return q
            jac = self._jacobian(q, rot_weight)
            jt = [[jac[r][c] for r in range(6)] for c in range(6)]
            jtj = [[sum(jt[i][k] * jac[k][j] for k in range(6)) for j in range(6)] for i in range(6)]
            jte = [sum(jt[i][k] * e[k] for k in range(6)) for i in range(6)]
            while lam < 1e9:
                damped = [[jtj[i][j] + (lam if i == j else 0) for j in range(6)] for i in range(6)]
                try:
                    dq = solve_linear(damped, jte)
                except ZeroDivisionError:
                    lam *= 4
                    continue
                step = max(abs(d) for d in dq)
                if step > 10:  # como mucho 10° por iteración
                    dq = [d * 10 / step for d in dq]
                nq = [qi + di for qi, di in zip(q, dq)]
                nerr, ne = weighted_error(nq)
                ncost = sum(x * x for x in ne)
                if ncost < cost:
                    q, err, e, cost = nq, nerr, ne, ncost
                    lam = max(lam / 3, 1e-6)
                    break
                lam *= 4
            else:
                return None
        return None

    def _jacobian(self, q: list[float], rot_weight: float, h: float = 1e-4) -> list[list[float]]:
        base = self.fk(q)
        cols = []
        for i in range(6):
            dq = list(q)
            dq[i] += h
            e = pose_error(base, self.fk(dq))
            cols.append([x / h for x in e[:3]] + [x * rot_weight / h for x in e[3:]])
        return [[cols[c][r] for c in range(6)] for r in range(6)]
