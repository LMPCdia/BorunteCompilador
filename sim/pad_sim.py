"""
Simulador cinemático de un respaldo del pad: ejecuta las MISMAS acciones que
se exportan (`pad/backup.py`) sobre un modelo de robot (`sim/kinematics.py`)
y devuelve la trayectoria de los ejes más una lista de problemas.

No simula dinámica ni el suavizado del controlador: es para ver forma,
alcance, límites y orden, con un tiempo de ciclo ESTIMADO. Supuestos (ver
docs/SIMULATOR.md):

- MOVEJ interpola en ejes; cada eje va a `speed`% de su velocidad máxima y
  todos llegan juntos (el más lento manda).
- MOVEL lleva la PUNTA de la herramienta en línea recta (posición y
  orientación interpoladas), con la misma regla de velocidad por tramo.
- Herramientas y sistemas de coordenadas: los valores se cargan a mano (del
  pad) en `tools` y `frames`, como X, Y, Z, U, V, W. Un MOVEL apunta la punta
  de la herramienta t a la pose P medida en el sistema c:
      brida = frame[c] · P · tool[t]⁻¹
  (hipótesis: misma convención U/V/W que las poses). La herramienta 0 es la
  brida y el sistema 0 la base.
- La pose inicial NO se conoce: el robot "aparece" en el primer MOVEJ (sin
  tiempo ni trayectoria). Un MOVEL antes de cualquier MOVEJ no se puede
  evaluar.
- Si un movimiento falla o no se puede simular, el robot sigue desde el
  destino (si se encuentra una configuración que llegue); si no, los MOVEL
  siguientes quedan sin evaluar hasta el próximo MOVEJ, en vez de dar una
  cascada de errores falsos.
- Perfil de velocidad: con `accel_s` = 0 cada movimiento va a velocidad
  constante de punta a punta (sin aceleraciones). Con `accel_s` > 0 cada
  movimiento arranca y termina quieto con un perfil trapezoidal: tarda
  `accel_s` en llegar a la velocidad del tramo y otro tanto en frenar (el
  respaldo solo trae `speed` y `smooth`: es una hipótesis configurable). Con
  `accel_s` = None se usan las aceleraciones máximas de cada eje del modelo
  (datasheet): el tramo tarda en acelerar lo que necesite el eje más
  exigido. Sigue siendo un supuesto que el controlador acelere al máximo.
- Velocidad lineal: si el modelo trae `max_linear_speed_mms`, un MOVEL no
  lleva la punta más rápido que esa velocidad × SPEED % (hipótesis: que el %
  del MOVEL sea sobre la velocidad lineal máxima). Sin el dato, solo limitan
  los ejes. `smooth`
  (redondeo de esquinas, probablemente) no se simula: el robot real puede
  no detenerse entre movimientos.
- Las entradas son fijas y el pad no tiene variables: volver a una etiqueta
  ya visitada (en la misma llamada) es un programa cíclico que se repite para
  siempre. Se simula UN ciclo, que es el tiempo de ciclo, y se informa.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

from pad.backup import Action, PadBackup
from sim.kinematics import Matrix, RobotModel, identity, mat_mul, pose_error, pose_matrix, rot_axis

MOVEJ, MOVEL, WAIT, SET_OUT = 4, 10, 100, 200
SET_COORD, SET_TOOL = 800, 801
IF_INPUT_GOTO, CALL, RETURN = 10001, 20000, 20001
COMMENT, LABEL, END = 50000, 59999, 60000

# Saltos de un eje entre dos muestras de un MOVEL que delatan una
# singularidad (la muñeca "da la vuelta").
JUMP_THRESHOLD_DEG = 20.0
LINEAR_STEP_MM = 25.0
LINEAR_STEP_DEG = 5.0
# Presupuestos: un programa que no los respeta se corta con un aviso, nunca
# deja la interfaz colgada.
MAX_ACTIONS = 20_000
MAX_CALL_DEPTH = 20
PROGRESS_EVERY = 50  # acciones entre llamadas a `progress`

Pose6 = list[float] | tuple[float, ...]  # X, Y, Z (mm), U, V, W (grados)


def invert(m: Matrix) -> Matrix:
    """Inversa de una transformación rígida."""
    rt = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(rt[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [rt[0] + [t[0]], rt[1] + [t[1]], rt[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


@dataclass
class Issue:
    severity: str  # "error" | "aviso" | "info"
    where: str     # "MAIN[12]" = programa y insertedIndex
    message: str
    time_s: float | None = None  # instante de la simulación, para saltar ahí


@dataclass
class Segment:
    kind: str                      # "MOVEJ" | "MOVEL" | "WAIT"
    where: str
    samples: list[list[float]]     # ángulos de los ejes, en grados
    duration_s: float
    name: str = ""
    tool: int = 0                  # herramienta con la que se dibuja la punta
    times: list[float] = field(default_factory=list)  # tiempo de cada muestra, desde 0
    failed: bool = False           # tramo hasta donde se llegó antes de un error
    accel_s: float = 0.0           # perfil trapezoidal con el que se calcularon `times`


@dataclass
class SimResult:
    model: str
    segments: list[Segment] = field(default_factory=list)
    issues: list[Issue] = field(default_factory=list)
    outputs: list[tuple[float, int, bool]] = field(default_factory=list)  # (t, salida, estado)
    total_moves: int = 0            # MOVEJ + MOVEL que el programa ejecutó
    skipped_moves: int = 0          # de esos, cuántos no se pudieron simular
    complete: bool = True           # False si se cortó (bucle, presupuesto, cancelado)
    start_deg: list[float] | None = None
    missing_tools: list[int] = field(default_factory=list)   # herramientas sin cargar
    missing_frames: list[int] = field(default_factory=list)  # sistemas sin cargar
    cyclic: bool = False            # el programa vuelve a empezar: se simuló un ciclo
    # Por qué no se simularon los `skipped_moves`:
    failed_moves: int = 0           # el robot no llega (error)
    no_frames_moves: int = 0        # herramienta/coordenadas sin cargar
    unevaluated_moves: int = 0      # pose desconocida (antes del primer MOVEJ, etc.)

    @property
    def total_time_s(self) -> float:
        return sum(s.duration_s for s in self.segments)

    @property
    def ok(self) -> bool:
        return not any(i.severity == "error" for i in self.issues)


class Cancelled(Exception):
    """La interfaz pidió cortar la simulación."""


class _Stop(Exception):
    """Fin de la simulación antes de tiempo (presupuesto, error fatal)."""


class _Cycle(Exception):
    """El programa volvió a empezar: un ciclo completo ya está simulado."""


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
                 start_deg: list[float] | None = None,
                 tools: dict[int, Pose6] | None = None,
                 frames: dict[int, Pose6] | None = None,
                 progress: Callable[[int], None] | None = None,
                 accel_s: float | None = 0.0) -> None:
        self.model = model
        # None = las aceleraciones del modelo (si no tiene, sin perfil).
        self.accel_s = None if accel_s is None else max(0.0, float(accel_s))
        self.inputs = inputs or {}
        # None = no se sabe dónde está el robot (hasta el primer MOVEJ).
        self.q: list[float] | None = list(start_deg) if start_deg else None
        self.tools = {0: identity(), **{int(k): pose_matrix(*v) for k, v in (tools or {}).items()}}
        self.frames = {0: identity(), **{int(k): pose_matrix(*v) for k, v in (frames or {}).items()}}
        self.progress = progress
        self._actions = 0
        self._missing_tools: set[int] = set()
        self._missing_frames: set[int] = set()
        self._unknown: dict[int, list[str]] = {}
        # MOVEL sin evaluar por pose desconocida, agrupados por causa: (dónde, cuándo)
        self._unevaluated: dict[str, list[tuple[str, float]]] = {}
        self._lost_reason = "el programa no empieza con un MOVEJ (no se conoce la pose inicial)"
        self._elapsed = 0.0  # suma de duraciones (evita recalcularla en cada aviso)
        self._seen_jumps: set = set()
        self._last_known: list[float] = list(self.q) if self.q else [0.0] * 6  # para dibujar

    # -- API -------------------------------------------------------------------

    def run(self, backup: PadBackup) -> SimResult:
        result = SimResult(model=self.model.name)
        self._result = result
        modules = {m.id: m for m in backup.act.modules}
        try:
            self._run_program("MAIN", backup.act.main, modules, depth=0, stack=())
        except _Cycle:
            pass  # completo: un ciclo es todo lo que hay que ver
        except _Stop:
            result.complete = False
        self._summarize()
        return result

    def tcp(self, q: list[float], tool: int = 0) -> tuple[float, float, float]:
        """Punta de la herramienta en el mundo (la brida si no se conoce)."""
        m = mat_mul(self.model.fk(q), self.tools.get(tool, identity()))
        return (m[0][3], m[1][3], m[2][3])

    # -- avisos -----------------------------------------------------------------

    def _issue(self, severity: str, where: str, message: str) -> None:
        self._result.issues.append(Issue(severity, where, message, self._elapsed))

    def _note_on_last_error(self, note: str) -> None:
        """Agrega una aclaración al último error en vez de sumar otro aviso
        (un error y su recuperación son UN problema para el usuario)."""
        for issue in reversed(self._result.issues):
            if issue.severity == "error":
                issue.message = f"{issue.message}. {note}"
                return

    def _add_segment(self, segment: Segment) -> None:
        self._result.segments.append(segment)
        self._elapsed += segment.duration_s

    def _summarize(self) -> None:
        r = self._result
        r.missing_tools = sorted(self._missing_tools)
        r.missing_frames = sorted(self._missing_frames)
        missing = []
        if self._missing_tools:
            missing.append("herramienta " + ", ".join(map(str, sorted(self._missing_tools))))
        if self._missing_frames:
            missing.append("coordenadas " + ", ".join(map(str, sorted(self._missing_frames))))
        if missing:
            r.issues.append(Issue(
                "aviso", "programa",
                f"Falta cargar {' y '.join(missing)} (valores del pad, pestañas Herramientas y "
                f"Coordenadas): los MOVEL que las usan no se simularon."))
        for reason, wheres in self._unevaluated.items():
            first, when = wheres[0]
            r.issues.append(Issue(
                "aviso", first,
                f"{len(wheres)} MOVEL sin evaluar porque no se sabía dónde estaba el robot "
                f"después de {reason} (el primero en {first}). Se retoma en el próximo "
                f"MOVEJ.", when))
        for code, wheres in self._unknown.items():
            r.issues.append(Issue(
                "aviso", wheres[0], f"Acción {code} desconocida ({len(wheres)} vez/veces): no se simula"))

    # -- intérprete -----------------------------------------------------------------

    def _tick(self, where: str) -> None:
        self._actions += 1
        if self._actions > MAX_ACTIONS:
            self._issue("aviso", where, f"Se cortó la simulación después de {MAX_ACTIONS} "
                                        f"acciones: el programa es demasiado largo o repite mucho.")
            raise _Stop
        if self.progress is not None and self._actions % PROGRESS_EVERY == 0:
            self.progress(self._actions)  # puede tirar Cancelled

    def _goto(self, name: str, flag: int, labels: dict, where: str, stack: tuple) -> int:
        """PC de la etiqueta `flag`, cortando si es un bucle infinito."""
        if flag not in labels:
            self._issue("error", where, f"Salto a la etiqueta {flag}, que no existe en {name}")
            raise _Stop
        pc = labels[flag]
        key = (stack, name, pc)
        if key in self._seen_jumps:
            self._result.cyclic = True
            self._issue("info", where,
                        f"El programa vuelve a la etiqueta {flag}: es cíclico (con estas "
                        f"entradas se repite siempre igual). Se simuló un ciclo: "
                        f"{self._elapsed:.1f} s.")
            raise _Cycle
        self._seen_jumps.add(key)
        return pc

    def _run_program(self, name: str, actions: list[Action], modules, depth: int,
                     stack: tuple) -> None:
        if depth > MAX_CALL_DEPTH:
            self._issue("error", name, f"Más de {MAX_CALL_DEPTH} llamadas anidadas (¿recursión?)")
            raise _Stop
        labels = {a["flag"]: i for i, a in enumerate(actions) if a["action"] == LABEL}
        pc = 0
        while pc < len(actions):
            action = actions[pc]
            code = action["action"]
            where = f"{name}[{action.get('insertedIndex', '?')}]"
            self._tick(where)
            pc += 1
            if code in (MOVEJ, MOVEL):
                self._move(action, where)
            elif code == WAIT:
                self._wait(action, where)
            elif code == SET_OUT:
                self._result.outputs.append(
                    (self._elapsed, action["point"], bool(action["pointStatus"])))
            elif code == IF_INPUT_GOTO:
                state = self.inputs.get(action["point"], False)
                if state == bool(action["pointStatus"]):
                    pc = self._goto(name, action["flag"], labels, where, stack)
            elif code == CALL:
                module = modules.get(int(action["module"]))
                if module is None:
                    self._issue("error", where, f"Llamada al módulo {action['module']}, que no existe")
                    continue
                self._run_program(module.name, module.actions, modules, depth + 1,
                                  stack + ((name, pc),))
                # Hipótesis (docs/PAD_FORMAT.md): flag != -1 = saltar al volver.
                if action["flag"] != -1:
                    flag = int(action["flag"])
                    if flag in labels:
                        pc = self._goto(name, flag, labels, where, stack)
                    else:
                        self._issue("aviso", where, f"La llamada salta a la etiqueta {flag}, "
                                                    f"que no existe: se sigue de largo")
            elif code in (RETURN, END):
                return
            elif code == LABEL:
                # Pasar por una etiqueta (sin saltar) también cuenta como
                # visitarla: así un GOTO hacia atrás corta al primer regreso.
                self._seen_jumps.add((stack, name, pc - 1))
            elif code in (COMMENT, SET_COORD, SET_TOOL):
                pass
            else:
                self._unknown.setdefault(code, []).append(where)

    def _wait(self, action: Action, where: str) -> None:
        seconds = float(action.get("limit", 0))
        if action.get("isUnlimit"):
            self._issue("aviso", where, "WAIT sin límite de tiempo: se simula como 0 s")
            seconds = 0.0
        if seconds < 0:
            self._issue("aviso", where, f"WAIT negativo ({seconds}): se simula como 0 s")
            seconds = 0.0
        q = list(self.q) if self.q is not None else list(self._last_known)
        self._add_segment(Segment("WAIT", where, [q], seconds, times=[0.0]))

    # -- movimientos ------------------------------------------------------------------

    def _move(self, action: Action, where: str) -> None:
        r = self._result
        r.total_moves += 1
        pos = action["points"][0]["pos"]
        values = [float(pos[f"m{i}"]) for i in range(6)]
        speed = float(action.get("speed", 100)) / 100.0
        if speed <= 0:
            self._issue("aviso", where, "SPEED 0: se simula al 1 %")
            speed = 0.01
        name = action.get("customName", "")
        tool_coord = int(action.get("toolCoord", 0))
        tool, coord = tool_coord >> 16, tool_coord & 0xFFFF
        if action.get("relativeType", 0) != 0:
            self._issue("aviso", where, "Movimiento relativo (relativeType): se simula como "
                                        "absoluto, puede no coincidir con el robot")

        if action["action"] == MOVEJ:
            target = values
            bad = self.model.out_of_limits(target)
            if bad:
                ejes = ", ".join(f"J{i + 1}={target[i]:.1f}°" for i in bad)
                self._issue("error", where, f"MOVEJ fuera de rango: {ejes}")
            if self.q is None:
                # Primer MOVEJ (o el primero después de perder la pose): el robot
                # se ubica ahí. La pose inicial es solo la del primero de todos.
                if r.start_deg is None:
                    r.start_deg = list(target)
                self._add_segment(Segment("MOVEJ", where, [list(target)], 0.0, name,
                                          tool if tool in self.tools else 0, [0.0]))
            else:
                samples = self._joint_path(self.q, target)
                times, ramp = self._times(samples, speed)
                self._add_segment(Segment("MOVEJ", where, samples, times[-1], name,
                                          tool if tool in self.tools else 0, times,
                                          accel_s=ramp))
            self.q = list(target)
            self._last_known = list(target)
            return

        # MOVEL
        missing = False
        if tool not in self.tools:
            self._missing_tools.add(tool)
            missing = True
        if coord not in self.frames:
            self._missing_frames.add(coord)
            missing = True
        if missing:
            r.skipped_moves += 1
            r.no_frames_moves += 1
            self._lose(f"un MOVEL con herramienta/coordenadas sin cargar ({where})")
            return
        if self.q is None:
            r.skipped_moves += 1
            r.unevaluated_moves += 1
            self._unevaluated.setdefault(self._lost_reason, []).append((where, self._elapsed))
            return
        target_tcp = mat_mul(self.frames[coord], pose_matrix(*values))
        samples, failed = self._linear_path(target_tcp, self.tools[tool], where)
        tips = None
        if self.model.max_linear_speed_mms:
            tips = []
            for q in samples:
                m = mat_mul(self.model.fk(q), self.tools[tool])
                tips.append((m[0][3], m[1][3], m[2][3]))
        times, ramp = self._times(samples, speed, tips)
        self._add_segment(Segment("MOVEL", where, samples, times[-1], name, tool, times, failed,
                                  accel_s=ramp))
        if failed:
            r.skipped_moves += 1
            r.failed_moves += 1
            self._recover(mat_mul(target_tcp, invert(self.tools[tool])), where, samples[-1])
        else:
            self.q = samples[-1]
        self._last_known = list(samples[-1])

    def _lose(self, reason: str) -> None:
        self.q = None
        self._lost_reason = reason

    def _recover(self, target_flange: Matrix, where: str, last: list[float]) -> None:
        """Después de un MOVEL fallido: seguir desde el destino si alguna
        configuración llega, para no encadenar errores falsos."""
        seeds = [self.q] + [
            [s[0], s[1], s[2], s[3] + d4, s[4] * k, s[5] + d6]
            for s in [self.q] for d4, d6, k in ((180, 180, -1), (-180, -180, -1), (90, 0, 1), (-90, 0, 1))
        ]
        fallback = None
        for n, seed in enumerate(seeds):
            q = self.model.ik(target_flange, seed)
            if q is not None:
                q = self.model.wrap_into_limits(q, seed)
                if fallback is None:
                    fallback = q
                if not self.model.out_of_limits(q):
                    self.q = q
                    self._last_known = list(q)
                    how = "" if n == 0 else " (con otra configuración del brazo)"
                    self._note_on_last_error(f"Se sigue desde el destino{how}; en la "
                                             f"animación el robot salta ahí.")
                    # Salto sin tiempo ni trayectoria: no es un movimiento real.
                    self._add_segment(Segment("SALTO", where, [list(last), list(q)], 0.0,
                                              times=[0.0, 0.0]))
                    return
        if fallback is not None:
            # El destino solo se alcanza fuera de rango: se sigue igual desde ahí,
            # para no dejar sin evaluar todo lo que viene (el error ya se reportó).
            self.q = fallback
            self._last_known = list(fallback)
            self._note_on_last_error("Se sigue desde el destino, aunque ahí algún eje queda "
                                     "fuera de rango.")
            self._add_segment(Segment("SALTO", where, [list(last), list(fallback)], 0.0,
                                      times=[0.0, 0.0]))
            return
        self._lose(f"el MOVEL fallido en {where}")

    def _joint_path(self, q0: list[float], q1: list[float]) -> list[list[float]]:
        n = max(2, int(max(abs(b - a) for a, b in zip(q0, q1)) / 2) + 1)
        return [[a + (b - a) * i / (n - 1) for a, b in zip(q0, q1)] for i in range(n)]

    def _linear_path(self, target_tcp: Matrix, tool: Matrix, where: str) -> tuple[list[list[float]], bool]:
        """Recta de la PUNTA. Devuelve las muestras hasta donde se llegó y si falló."""
        tool_inv = invert(tool)
        start = mat_mul(self.model.fk(self.q), tool)
        dist = math.dist([start[i][3] for i in range(3)], [target_tcp[i][3] for i in range(3)])
        rot = math.sqrt(sum(e * e for e in pose_error(start, target_tcp)[3:]))
        n = max(2, int(max(dist / LINEAR_STEP_MM, math.degrees(rot) / LINEAR_STEP_DEG)) + 1)
        q, samples = list(self.q), [list(self.q)]
        worst_jump, jump_at = 0.0, 0.0
        for i in range(1, n):
            t = i / (n - 1)
            r = _interp_rotation(start, target_tcp, t)
            p = [start[k][3] + (target_tcp[k][3] - start[k][3]) * t for k in range(3)]
            tcp = [r[0] + [p[0]], r[1] + [p[1]], r[2] + [p[2]], [0.0, 0.0, 0.0, 1.0]]
            nq = self.model.ik(mat_mul(tcp, tool_inv), q)
            if nq is None:
                # ¿El destino se alcanza? Entonces el problema es el camino: casi
                # siempre la muñeca pasando por J5 = 0.
                end = self.model.ik(mat_mul(target_tcp, tool_inv), q) or self.model.ik(
                    mat_mul(target_tcp, tool_inv), [q[0], q[1], q[2], q[3] + 90, -q[4], q[5]])
                if end is not None and (abs(q[4]) < 15 or end[4] * q[4] < 0):
                    why = (f"la recta pasa por la singularidad de muñeca (J5 ≈ 0°; J4 y J6 "
                           f"tendrían que girar de golpe). El destino sí se alcanza: probá un "
                           f"MOVEJ o un punto intermedio")
                elif end is not None:
                    why = "la recta pasa por donde el brazo no llega (el destino sí se alcanza)"
                else:
                    why = "el destino está fuera de alcance"
                self._issue("error", where, f"MOVEL inalcanzable al {t:.0%} del recorrido: {why}")
                return samples, True
            # Sin "acomodar la vuelta" (wrap_into_limits): dentro de una recta
            # manda la continuidad, y si un eje pasa su límite es real.
            jump = max(abs(a - b) for a, b in zip(nq, q))
            if jump > worst_jump:
                worst_jump, jump_at = jump, t
            bad = self.model.out_of_limits(nq)
            if bad:
                detail = ", ".join(f"J{b + 1} llega a {nq[b]:.1f}° (rango "
                                   f"{self.model.joints[b].min_deg:g}/{self.model.joints[b].max_deg:g})"
                                   for b in bad)
                self._issue("error", where, f"MOVEL fuera de rango al {t:.0%} del recorrido: {detail}")
                return samples, True
            q = nq
            samples.append(q)
        if worst_jump > JUMP_THRESHOLD_DEG:
            self._issue("aviso", where,
                        f"Giro brusco de {worst_jump:.0f}° de un eje al {jump_at:.0%} del MOVEL: "
                        f"probable paso cerca de la singularidad de muñeca")
        return samples, False

    def _times(self, samples: list[list[float]], speed: float,
               tips: list[tuple[float, float, float]] | None = None) -> tuple[list[float], float]:
        """Tiempo de cada muestra y el tiempo de aceleración del tramo."""
        speeds = [j.max_speed_dps * speed for j in self.model.joints]
        linear = (self.model.max_linear_speed_mms or 0.0) * speed
        times = [0.0]
        for i, (s0, s1) in enumerate(zip(samples, samples[1:])):
            dt = max(abs(b - a) / v for a, b, v in zip(s0, s1, speeds))
            if tips is not None and linear > 0:
                dt = max(dt, math.dist(tips[i], tips[i + 1]) / linear)
            times.append(times[-1] + dt)
        ramp = self._ramp(samples, times)
        return trapezoid(times, ramp), ramp

    def _ramp(self, samples: list[list[float]], times: list[float]) -> float:
        """Tiempo de aceleración del tramo: el fijo, o con aceleraciones del
        modelo, lo que tarda el eje más exigido en llegar a su velocidad."""
        if self.accel_s is not None:
            return self.accel_s
        if not self.model.has_accelerations:
            return 0.0
        peak = [0.0] * 6
        for (s0, s1), (t0, t1) in zip(zip(samples, samples[1:]), zip(times, times[1:])):
            if t1 > t0:
                for j in range(6):
                    peak[j] = max(peak[j], abs(s1[j] - s0[j]) / (t1 - t0))
        return max(v / joint.max_accel_dps2 for v, joint in zip(peak, self.model.joints))


def trapezoid(times: list[float], accel_s: float) -> list[float]:
    """Tiempos a velocidad constante -> tiempos con arranque y frenado.

    `times` es el tiempo de cada muestra yendo todo el camino a la velocidad
    del tramo (de 0 a T). Con un perfil trapezoidal que tarda `accel_s` en
    acelerar de 0 a esa velocidad (y lo mismo en frenar), la muestra que
    estaba en t pasa a estar en:
        t < accel/2:          sqrt(2·t·accel)
        en el medio:          t + accel/2
        t > T - accel/2:      T + accel - sqrt(2·(T-t)·accel)
    Si el tramo es tan corto que no llega a la velocidad (T < accel), el
    perfil es triangular y dura 2·sqrt(T·accel)."""
    if accel_s <= 0 or len(times) < 2 or times[-1] <= 0:
        return list(times)
    total_t, a = times[-1], accel_s
    if total_t >= a:
        total = total_t + a
        half = a / 2
    else:
        total = 2 * math.sqrt(total_t * a)
        half = total_t / 2
    out = []
    for t in times:
        if t <= half:
            out.append(math.sqrt(2 * t * a))
        elif t >= total_t - half:
            out.append(total - math.sqrt(max(0.0, 2 * (total_t - t) * a)))
        else:
            out.append(t + a / 2)
    return out


def untrapezoid(tau: float, total: float, accel_s: float) -> float:
    """Inversa de `trapezoid`: instante real -> tiempo "a velocidad constante"
    (que es lo que dice cuánto camino se hizo). `total` es la duración real."""
    a = accel_s
    if a <= 0 or total <= 0:
        return tau
    tau = min(max(tau, 0.0), total)
    if total >= 2 * a:                 # trapecio: T = total - a
        nominal = total - a
        if tau <= a:
            return tau * tau / (2 * a)
        if tau >= total - a:
            return nominal - (total - tau) ** 2 / (2 * a)
        return tau - a / 2
    nominal = total * total / (4 * a)  # triángulo
    if tau <= total / 2:
        return tau * tau / (2 * a)
    return nominal - (total - tau) ** 2 / (2 * a)


def simulate(backup: PadBackup, model: RobotModel, inputs: dict[int, bool] | None = None,
             start_deg: list[float] | None = None, tools: dict[int, Pose6] | None = None,
             frames: dict[int, Pose6] | None = None, accel_s: float | None = 0.0) -> SimResult:
    return PadSimulator(model, inputs, start_deg, tools, frames, accel_s=accel_s).run(backup)


def inputs_used(backup: PadBackup) -> list[int]:
    """Entradas que el programa consulta (para ofrecerlas en la interfaz)."""
    found = set()
    for actions in [backup.act.main] + [m.actions for m in backup.act.modules]:
        for a in actions:
            if a["action"] == IF_INPUT_GOTO:
                found.add(int(a["point"]))
    return sorted(found)
