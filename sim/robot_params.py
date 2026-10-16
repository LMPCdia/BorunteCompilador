"""
Planilla de parámetros de un robot (lo del datasheet que hace falta para
simular) -> modelo listo para usar.

El CAD da la geometría (`sim/robot_import.py`); la planilla da lo que el CAD
no dice: rango, velocidad y aceleración máxima de cada eje, sentido de giro,
velocidad lineal máxima, y para controlar, el alcance y las cotas del plano.
Se escribe como Google Sheet (o CSV) en la carpeta del robot de la
biblioteca, con este formato (las columnas se reconocen por el título, en
castellano o inglés; ver `template_csv`):

    Eje | Mínimo (°) | Máximo (°) | Velocidad máx (°/s) | Aceleración máx (°/s²) | Sentido | Confirmado | Notas
    J1  | -165       | 165        | 190                 |                        | 1       | parcial    | …
    …
    Dato                 | Valor | Unidad | Confirmado | Notas
    Alcance              | 1500  | mm     | sí         |
    Velocidad lineal máx |       | mm/s   |            |
    d1 … d6, a1 … a3     |       | mm     |            | cotas del plano (se comparan con el CAD)

- Un rango se puede escribir "±165" en Mínimo o en Máximo.
- Decimales con coma o punto.
- Lo vacío no se usa (la aceleración vacía = el simulador supone un perfil).
- "Confirmado": sí / parcial / no. Lo que no está confirmado se marca como
  hipótesis en el modelo (regla 1 de CLAUDE.md).

Después de armar el modelo, `verify_ik` comprueba la cinemática inversa con
poses al azar dentro de los rangos: es el control automático de que las
cotas y los rangos juntos tienen sentido.
"""

from __future__ import annotations

import csv
import io
import json
import math
import random
import re
import unicodedata
from dataclasses import dataclass, field
from pathlib import Path

DIM_KEYS = ("d1", "a1", "a2", "a3", "d4", "d6")
GENERAL_KEYS = {
    "alcance": "reach_mm", "reach": "reach_mm", "alcance maximo": "reach_mm",
    "carga": "payload_kg", "payload": "payload_kg", "carga util": "payload_kg",
    "repetibilidad": "repeatability_mm", "repeatability": "repeatability_mm",
    "velocidad lineal max": "max_linear_speed_mms", "velocidad lineal maxima": "max_linear_speed_mms",
    "velocidad lineal": "max_linear_speed_mms", "max linear speed": "max_linear_speed_mms",
    "linear speed": "max_linear_speed_mms",
}
DIM_TOLERANCE_MM = 2.0
REACH_TOLERANCE = 0.02


class ParamsError(ValueError):
    pass


@dataclass
class JointParams:
    min_deg: float
    max_deg: float
    max_speed_dps: float
    max_accel_dps2: float | None = None
    sign: int | None = None
    confirmed: str = "no"          # "sí" | "parcial" | "no"
    note: str = ""


@dataclass
class RobotParams:
    model: str = ""
    joints: list[JointParams] = field(default_factory=list)
    general: dict[str, float] = field(default_factory=dict)
    general_confirmed: dict[str, str] = field(default_factory=dict)
    dims: dict[str, float] = field(default_factory=dict)


def _plain(text: str) -> str:
    """Minúsculas, sin acentos ni signos: para reconocer títulos."""
    text = unicodedata.normalize("NFKD", text).encode("ascii", "ignore").decode()
    return re.sub(r"[^a-z0-9 ]+", " ", text.lower()).strip()


def _number(text: str) -> float | None:
    text = text.strip().replace("°", "").replace(" ", "")
    if not text:
        return None
    text = text.replace(",", ".")
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _plus_minus(text: str) -> float | None:
    m = re.fullmatch(r"\s*(?:±|\+/-|\+-)\s*([0-9.,]+)\s*°?\s*", text)
    return _number(m.group(1)) if m else None


def _confirmed(text: str) -> str:
    t = _plain(text)
    if t in ("si", "s", "yes", "y", "x", "ok", "confirmado"):
        return "sí"
    if t.startswith("parc"):
        return "parcial"
    return "sí" if text.strip() in ("✓", "✔") else "no"


def _column_roles(header: list[str]) -> dict[str, int]:
    roles = {}
    for i, title in enumerate(header):
        t = _plain(title)
        if not t:
            continue
        if "veloc" in t or "speed" in t:
            roles.setdefault("speed", i)
        elif "acel" in t or "accel" in t:
            roles.setdefault("accel", i)
        elif "sentido" in t or "sign" in t or "direcc" in t:
            roles.setdefault("sign", i)
        elif "confirm" in t:
            roles.setdefault("confirmed", i)
        elif "nota" in t or "note" in t:
            roles.setdefault("note", i)
        elif "min" in t:
            roles.setdefault("min", i)
        elif "max" in t:
            roles.setdefault("max", i)
        elif "rango" in t or "range" in t:
            roles.setdefault("range", i)
    return roles


def parse_params(text: str) -> RobotParams:
    """CSV (export de la Google Sheet) -> parámetros. Tira ParamsError con
    TODOS los problemas encontrados, con su fila."""
    rows = list(csv.reader(io.StringIO(text.lstrip("﻿"))))
    out = RobotParams()
    errors: list[str] = []
    joints: dict[int, JointParams] = {}
    roles: dict[str, int] | None = None

    def cell(row, role):
        i = roles.get(role) if roles else None
        return row[i].strip() if i is not None and i < len(row) else ""

    for n, row in enumerate(rows, start=1):
        if not row or not any(c.strip() for c in row):
            continue
        first = _plain(row[0])
        m = re.search(r"BRTIRUS\d+[A-Z]?", " ".join(row).upper())
        if m and not out.model:
            out.model = m.group(0)
        if first in ("eje", "axis", "joint", "articulacion"):
            roles = _column_roles(row)
            for need in ("speed",):
                if need not in roles:
                    errors.append(f"fila {n}: falta la columna de velocidad máxima")
            continue
        jm = re.fullmatch(r"(?:j|eje|axis|joint)? ?([1-6])", first)
        if jm and roles is not None:
            index = int(jm.group(1))
            lo_text, hi_text = cell(row, "min"), cell(row, "max")
            lo, hi = _number(lo_text), _number(hi_text)
            for t in (lo_text, hi_text, cell(row, "range")):
                pm = _plus_minus(t)
                if pm is not None:
                    lo, hi = -pm, pm
            if (lo is None or hi is None) and cell(row, "range"):
                nums = [_number(x) for x in re.split(r"\s*(?:/|a|to|~)\s*", cell(row, "range"))]
                if len(nums) == 2 and None not in nums:
                    lo, hi = nums
            speed = _number(cell(row, "speed"))
            accel = _number(cell(row, "accel"))
            sign_text = cell(row, "sign")
            sign = _number(sign_text) if sign_text else None
            where = f"fila {n} (J{index})"
            if lo is None or hi is None:
                errors.append(f"{where}: falta el rango (mínimo y máximo, o ±X)")
            elif lo >= hi:
                errors.append(f"{where}: el mínimo ({lo:g}) no es menor que el máximo ({hi:g})")
            if speed is None or speed <= 0:
                errors.append(f"{where}: falta la velocidad máxima (°/s, mayor que 0)")
            if accel is not None and accel <= 0:
                errors.append(f"{where}: la aceleración tiene que ser mayor que 0")
            if sign is not None and sign not in (1, -1):
                errors.append(f"{where}: el sentido tiene que ser 1 o -1")
            if index in joints:
                errors.append(f"{where}: J{index} está repetido")
            joints[index] = JointParams(
                lo if lo is not None else 0.0, hi if hi is not None else 0.0,
                speed or 0.0, accel, int(sign) if sign in (1, -1) else None,
                _confirmed(cell(row, "confirmed")), cell(row, "note"))
            continue
        if first in ("dato", "parametro", "item", "parameter"):
            continue
        key = GENERAL_KEYS.get(first) or (first if first in DIM_KEYS else None)
        if key is not None:
            value = _number(row[1]) if len(row) > 1 else None
            if value is None:
                continue                       # vacío: no se usa
            if value <= 0 and key != "repeatability_mm":
                errors.append(f"fila {n} ({row[0].strip()}): tiene que ser mayor que 0")
                continue
            if key in DIM_KEYS:
                out.dims[key] = value
            else:
                out.general[key] = value
                out.general_confirmed[key] = _confirmed(row[3]) if len(row) > 3 else "no"

    missing = [f"J{i}" for i in range(1, 7) if i not in joints]
    if missing:
        errors.append("faltan los ejes " + ", ".join(missing) +
                      " (una fila por eje, debajo de la fila de títulos «Eje | Mínimo | …»)")
    if errors:
        raise ParamsError("La planilla de parámetros tiene problemas:\n- " + "\n- ".join(errors))
    out.joints = [joints[i] for i in range(1, 7)]
    return out


# --- aplicar al modelo ------------------------------------------------------------------


def apply_params(data: dict, params: RobotParams, source: str = "") -> tuple[dict, list[str]]:
    """JSON del modelo (con la geometría del CAD) + planilla -> JSON nuevo y
    avisos (diferencias con el CAD, datos sin confirmar)."""
    new = json.loads(json.dumps(data))
    warnings = []
    old_joints = data.get("joints", [{}] * 6)
    new["joints"] = []
    for i, (jp, old) in enumerate(zip(params.joints, old_joints)):
        joint = {"min_deg": jp.min_deg, "max_deg": jp.max_deg, "max_speed_dps": jp.max_speed_dps,
                 "sign": jp.sign if jp.sign is not None else int(old.get("sign", 1))}
        if jp.max_accel_dps2:
            joint["max_accel_dps2"] = jp.max_accel_dps2
        new["joints"].append(joint)
    if params.general.get("max_linear_speed_mms"):
        new["max_linear_speed_mms"] = params.general["max_linear_speed_mms"]
    else:
        new.pop("max_linear_speed_mms", None)

    geometry = data.get("geometry_mm", {})
    for key, value in params.dims.items():
        cad = geometry.get(key)
        if cad is not None and abs(cad - value) > max(DIM_TOLERANCE_MM, 0.01 * abs(value)):
            warnings.append(f"{key}: el plano dice {value:g} mm y el CAD mide {cad:g} mm "
                            f"(se usa el CAD)")
    reach = params.general.get("reach_mm")
    if reach and data.get("reach_mm") and abs(data["reach_mm"] - reach) > REACH_TOLERANCE * reach:
        warnings.append(f"alcance: el datasheet dice {reach:g} mm y el CAD da "
                        f"{data['reach_mm']:g} mm: revisar el CAD o el importador")

    unconfirmed = [f"J{i + 1}" for i, j in enumerate(params.joints) if j.confirmed != "sí"]
    if unconfirmed:
        warnings.append("sin confirmar en la planilla: " + ", ".join(unconfirmed) +
                        " (se usan igual, como hipótesis)")
    if not all(j.max_accel_dps2 for j in params.joints):
        warnings.append("faltan aceleraciones de algún eje: el simulador supone un perfil")
    if not params.general.get("max_linear_speed_mms"):
        warnings.append("falta la velocidad lineal máxima: los MOVEL solo los limitan los ejes")

    parts = [f"Ejes de la planilla de parámetros{f' ({source})' if source else ''}."]
    status = {"sí": "confirmado", "parcial": "confirmado en parte", "no": "SIN CONFIRMAR"}
    for i, j in enumerate(params.joints):
        if j.confirmed != "sí" or j.note:
            note = f" ({j.note})" if j.note else ""
            parts.append(f"J{i + 1}: {status[j.confirmed]}{note}.")
    geometry_note = data.get("notes", "").split(" Ejes de la planilla")[0]
    new["notes"] = (geometry_note + " " + " ".join(parts)).strip()
    new["params"] = {
        "source": source,
        "general": params.general,
        "confirmed": {f"J{i + 1}": j.confirmed for i, j in enumerate(params.joints)},
    }
    return new, warnings


# --- verificar la cinemática -------------------------------------------------------------------


def verify_ik(model, samples: int = 60, seed: int = 1) -> tuple[int, int, float]:
    """Poses al azar dentro de los rangos (lejos de la singularidad de
    muñeca): directa -> inversa (arrancando cerca) -> directa. Devuelve
    (cuántas cerraron, cuántas se probaron, peor error en mm)."""
    rng = random.Random(seed)
    ok, worst = 0, 0.0
    for _ in range(samples):
        while True:
            q = [rng.uniform(j.min_deg, j.max_deg) for j in model.joints]
            if abs(q[4]) > 10:
                break
        target = model.fk(q)
        start = [a + rng.uniform(-8, 8) for a in q]
        sol = model.ik(target, start)
        if sol is None:
            continue
        got = model.fk(sol)
        err = math.dist([got[i][3] for i in range(3)], [target[i][3] for i in range(3)])
        rot = max(abs(got[i][k] - target[i][k]) for i in range(3) for k in range(3))
        if err < 0.1 and rot < 1e-3:
            ok += 1
        worst = max(worst, err)
    return ok, samples, worst


# --- plantilla -----------------------------------------------------------------------------------


def template_csv(data: dict) -> str:
    """Planilla para completar, con lo que ya tiene el modelo."""
    out = io.StringIO()
    w = csv.writer(out)
    w.writerow(["Parámetros del robot", data.get("name", ""), "", "", "", "", "", ""])
    w.writerow([])
    w.writerow(["Eje", "Mínimo (°)", "Máximo (°)", "Velocidad máx (°/s)",
                "Aceleración máx (°/s²)", "Sentido", "Confirmado", "Notas"])
    for i, j in enumerate(data.get("joints", [{}] * 6)):
        w.writerow([f"J{i + 1}", _g(j.get("min_deg")), _g(j.get("max_deg")),
                    _g(j.get("max_speed_dps")), _g(j.get("max_accel_dps2")),
                    _g(j.get("sign", 1)), "no", ""])
    w.writerow([])
    w.writerow(["Dato", "Valor", "Unidad", "Confirmado", "Notas"])
    w.writerow(["Alcance", _g(data.get("reach_mm")), "mm", "no", "del datasheet (se compara con el CAD)"])
    w.writerow(["Carga", "", "kg", "", ""])
    w.writerow(["Repetibilidad", "", "mm", "", ""])
    w.writerow(["Velocidad lineal máx", _g(data.get("max_linear_speed_mms")), "mm/s", "",
                "limita los MOVEL"])
    w.writerow(["Datasheet revisado", "", "", "", "ID del PDF ya transcripto (lo llena la rutina)"])
    for key in DIM_KEYS:
        w.writerow([key, "", "mm", "", "cota del plano (opcional: se compara con el CAD)"])
    return out.getvalue()


def _g(value) -> str:
    return "" if value is None else f"{value:g}" if isinstance(value, (int, float)) else str(value)


# --- desde la biblioteca ----------------------------------------------------------------------

JOINTS_FROM = "BRTIRUS1510A"   # ejes para un robot sin planilla (HIPÓTESIS, se avisa)


@dataclass
class Prepared:
    name: str
    imported: bool                 # se (re)importó el CAD
    reach_mm: float | None
    warnings: list[str]
    ik: tuple[int, int, float]     # verify_ik
    params_source: str = ""


def prepare_from_library(item, fetch, cache_dir: Path, models_dir: Path,
                         progress=lambda _text: None) -> Prepared:
    """Robot de la biblioteca listo para simular: baja la planilla (siempre)
    y el STEP (solo si el robot no está o si el STEP cambió en Drive), arma
    el modelo con los dos y verifica la cinemática inversa."""
    from sim import library
    from sim.kinematics import RobotModel, model_path

    name = item.model_name
    params = None
    if item.params_id:
        progress(f"Leyendo «{item.params_name}»…")
        params = parse_params(library.fetch_params(item, fetch))

    existing = None
    try:
        existing = model_path(name)
    except FileNotFoundError:
        pass
    current = json.loads(existing.read_text(encoding="utf-8")) if existing else None
    try:
        meta = library.remote_meta(item, fetch)
    except library.LibraryError:
        if current is None:
            raise
        meta = None                    # sin red: con lo que hay
    # Se reimporta si no está, o si es un modelo importado de Drive y el STEP
    # cambió. Un robot que viene con la app no se reimporta (ya está verificado).
    stale = (current is not None and meta is not None and current.get("cad")
             and current["cad"] != meta)
    imported = current is None or bool(stale)
    if imported:
        from sim.robot_import import import_robot_step

        progress(f"Bajando «{item.name}»…")
        step = library.download(item, cache_dir, fetch)
        progress(f"Importando {name} (unos minutos)…")
        path, _robot = import_robot_step(
            step, name, models_dir, JOINTS_FROM,
            progress=lambda i, n, part: progress(f"Importando {name}: parte {i + 1} de {n} ({part})"))
        current = json.loads(path.read_text(encoding="utf-8"))
        current["cad"] = meta

    warnings: list[str] = []
    if params is not None:
        current, warnings = apply_params(current, params, item.params_name)
    elif imported:
        warnings.append(f"sin planilla de parámetros en la carpeta: rangos y velocidades copiados "
                        f"del {JOINTS_FROM} (hipótesis)")
    if params is not None or imported:
        write_user_model(name, current, models_dir)
    progress("Verificando la cinemática inversa…")
    model = RobotModel.load(name)
    ik = verify_ik(model)
    if ik[0] < ik[1]:
        warnings.append(f"la cinemática inversa no cerró en {ik[1] - ik[0]} de {ik[1]} poses de "
                        f"prueba: revisar cotas y rangos")
    return Prepared(name, imported, current.get("reach_mm"), warnings, ik,
                    item.params_name if params is not None else "")


# --- línea de comandos -----------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    """python -m sim.robot_params PLANILLA.csv --model BRTIRUS1510A [--write]"""
    import argparse
    import sys

    from sim.kinematics import USER_MODELS_DIR, RobotModel, model_path

    parser = argparse.ArgumentParser(prog="python -m sim.robot_params",
                                     description="Aplicar la planilla de parámetros a un modelo")
    parser.add_argument("planilla", help="CSV exportado de la planilla (o «-» para la plantilla)")
    parser.add_argument("--model", required=True)
    parser.add_argument("--write", action="store_true",
                        help=f"guardar el modelo resultante en {USER_MODELS_DIR}")
    args = parser.parse_args(argv)
    try:
        data = json.loads(model_path(args.model).read_text(encoding="utf-8"))
        if args.planilla == "-":
            print(template_csv(data), end="")
            return 0
        params = parse_params(Path(args.planilla).read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2
    new, warnings = apply_params(data, params, Path(args.planilla).name)
    for i, j in enumerate(new["joints"]):
        print(f"J{i + 1}: {j['min_deg']:g}..{j['max_deg']:g}°  {j['max_speed_dps']:g} °/s  "
              f"{j.get('max_accel_dps2', '—')} °/s²  sentido {j['sign']}")
    for w in warnings:
        print(f"Aviso: {w}")
    if args.write:
        path = write_user_model(args.model, new)
        ok, n, worst = verify_ik(RobotModel.load(args.model))
        print(f"Guardado en {path}. Cinemática inversa: {ok}/{n} poses (peor error {worst:.3f} mm)")
    return 0


def write_user_model(name: str, data: dict, models_dir: Path | None = None) -> Path:
    """Guarda el modelo en la carpeta del usuario. Si las mallas no están ahí
    (robot que viene con la app), se usan las de la app (ver sim/scene.py)."""
    from sim.kinematics import USER_MODELS_DIR

    folder = Path(models_dir or USER_MODELS_DIR)
    folder.mkdir(parents=True, exist_ok=True)
    data = dict(data)
    data["name"] = name
    path = folder / f"{name}.json"
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
