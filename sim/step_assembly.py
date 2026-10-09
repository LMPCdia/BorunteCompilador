"""
Separar un ensamble STEP en sus partes, cada una ubicada donde está en el
ensamble.

Hace falta para los robots: el fabricante entrega el robot entero como UN
ensamble (base, J1, brazo, ...) y el simulador necesita una malla por
eslabón. gmsh importa el ensamble, pero junta todo y pierde a qué parte
pertenece cada cara (los nombres de Borunte vienen en GBK, no en UTF-8 como
pide la norma, y OpenCascade los descarta).

Cómo: se lee el grafo de entidades del STEP. Por cada parte (un
SHAPE_DEFINITION_REPRESENTATION que no sea el ensamble) se escribe un STEP
chico con solo las entidades que esa parte usa, se lo da a gmsh, y la malla
se mueve con la transformación del ensamble
(REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION -> ITEM_DEFINED_TRANSFORMATION).

Probado con el ensamble del BRTIRUS1510A (AP214, superficies, un nivel de
ensamble). Ensambles anidados: se compone la cadena de transformaciones.
"""

from __future__ import annotations

import math
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path

from sim.kinematics import Matrix, identity, mat_mul
from sim.meshes import Mesh, MeshError, load_step

_ENTITY = re.compile(r"#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|$)", re.S)
_REF = re.compile(r"#(\d+)")


@dataclass
class Part:
    name: str
    mesh: Mesh          # en coordenadas del ensamble
    matrix: Matrix      # dónde estaba la parte en el ensamble


def _decode(raw: str) -> str:
    """Los textos vienen como bytes crudos (leídos como latin-1). Borunte usa
    GBK; si no lo es, quedan como están."""
    data = raw.encode("latin-1")
    for encoding in ("utf-8", "gbk"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    return raw


class StepFile:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        # latin-1: cualquier byte pasa ida y vuelta sin tocarse.
        text = self.path.read_bytes().decode("latin-1")
        try:
            start = text.index("DATA;") + len("DATA;")
            end = text.rindex("ENDSEC;")
        except ValueError:
            raise MeshError(f"{self.path.name} no parece un archivo STEP") from None
        self.header = text[:start]
        self.entities: dict[int, str] = {
            int(m.group(1)): m.group(2).strip() for m in _ENTITY.finditer(text[start:end])}

    # -- consultas ------------------------------------------------------------------

    def kind(self, n: int) -> str:
        m = re.match(r"\(?\s*([A-Z_0-9]+)", self.entities[n])
        return m.group(1) if m else ""

    def refs(self, n: int) -> list[int]:
        return [int(x) for x in _REF.findall(self.entities[n])]

    def of_kind(self, kind: str) -> list[int]:
        return [n for n in self.entities if self.kind(n) == kind]

    def name(self, n: int) -> str:
        m = re.search(r"'((?:[^']|'')*)'", self.entities[n])
        return _decode(m.group(1).replace("''", "'")) if m else ""

    def closure(self, roots: list[int]) -> set[int]:
        seen, stack = set(), list(roots)
        while stack:
            n = stack.pop()
            if n in seen or n not in self.entities:
                continue
            seen.add(n)
            stack.extend(self.refs(n))
        return seen

    # -- geometría ----------------------------------------------------------------------

    def _numbers(self, n: int) -> list[float]:
        # Sin los textos ni las referencias, lo que queda son los números.
        body = _REF.sub(" ", re.sub(r"'(?:[^']|'')*'", " ", self.entities[n]))
        return [float(x) for x in re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?", body)]

    def placement(self, n: int) -> Matrix:
        """AXIS2_PLACEMENT_3D -> matriz 4x4."""
        r = self.refs(n)
        p = self._numbers(r[0])
        z = self._numbers(r[1]) if len(r) > 1 else [0.0, 0.0, 1.0]
        x = self._numbers(r[2]) if len(r) > 2 else [1.0, 0.0, 0.0]
        z = _unit(z)
        x = _unit([a - b * sum(i * j for i, j in zip(x, z)) for a, b in zip(x, z)])
        y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
        return [[x[0], y[0], z[0], p[0]], [x[1], y[1], z[1], p[1]],
                [x[2], y[2], z[2], p[2]], [0.0, 0.0, 0.0, 1.0]]

    # -- ensamble -------------------------------------------------------------------------

    def _rep_of(self) -> dict[int, int]:
        """PRODUCT_DEFINITION -> su representación de forma."""
        out = {}
        for sdr in self.of_kind("SHAPE_DEFINITION_REPRESENTATION"):
            pds, rep = self.refs(sdr)[:2]
            pd = self.refs(pds)[-1]
            out[pd] = rep
        return out

    def _transforms(self) -> dict[int, tuple[int, Matrix]]:
        """Representación hija -> (representación padre, transformación).

        Padre e hijo salen del NEXT_ASSEMBLY_USAGE_OCCURRENCE (no del orden de
        los argumentos de la relación, que cada CAD escribe a su manera)."""
        rep_of = self._rep_of()
        out = {}
        for cdsr in self.of_kind("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"):
            relation, pds = self.refs(cdsr)[:2]
            nauo = self.refs(pds)[-1]
            if self.kind(nauo) != "NEXT_ASSEMBLY_USAGE_OCCURRENCE":
                continue
            parent_pd, child_pd = self.refs(nauo)[:2]
            parent, child = rep_of.get(parent_pd), rep_of.get(child_pd)
            body = self.entities[relation]
            idt = re.search(r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)", body)
            if parent is None or child is None or not idt:
                continue
            a, b = self.refs(int(idt.group(1)))[:2]
            # Cada placement es de una de las dos representaciones: la parte
            # se dibuja en el suyo y en el padre va en el otro.
            if a in self.refs(child) and b not in self.refs(child):
                a, b = b, a
            out[child] = (parent, mat_mul(self.placement(a), _invert(self.placement(b))))
        return out

    def parts(self) -> list[tuple[str, int, Matrix]]:
        """(nombre, SHAPE_DEFINITION_REPRESENTATION, ubicación) de cada parte con
        geometría propia (no los ensambles, que solo agrupan)."""
        transforms = self._transforms()
        parents = {p for p, _m in transforms.values()}
        out = []
        for sdr in self.of_kind("SHAPE_DEFINITION_REPRESENTATION"):
            rep = self.refs(sdr)[1]
            if rep in parents:
                continue                       # un ensamble: sus partes van solas
            matrix, node = identity(), rep
            for _ in range(32):                # subir por la cadena de ensambles
                if node not in transforms:
                    break
                parent, m = transforms[node]
                matrix = mat_mul(m, matrix)
                node = parent
            out.append((self.name(rep), sdr, matrix))
        return out

    def write_part(self, sdr: int, path: Path) -> None:
        """STEP con solo lo que usa una parte (en sus propias coordenadas)."""
        rep = self.refs(sdr)[1]
        roots = [sdr]
        # Las representaciones con la geometría cuelgan de la de la parte.
        for n in self.of_kind("SHAPE_REPRESENTATION_RELATIONSHIP"):
            if self.refs(n)[:1] == [rep]:
                roots.append(n)
        keep = sorted(self.closure(roots))
        body = "".join(f"#{n} = {self.entities[n]};\n" for n in keep)
        path.write_bytes((self.header + "\n" + body + "ENDSEC;\nEND-ISO-10303-21;\n")
                         .encode("latin-1"))


# Para un robot alcanza con una malla gruesa: se ve bien y los choques se
# calculan rápido (el CAD del fabricante trae tornillos, textos, chaflanes...).
ROBOT_ELEMENTS_PER_CIRCLE = 6
ROBOT_MIN_SIZE_RATIO = 150.0


def split_assembly(path: str | Path, progress=None,
                   elements_per_circle: int = ROBOT_ELEMENTS_PER_CIRCLE,
                   min_size_ratio: float = ROBOT_MIN_SIZE_RATIO) -> list[Part]:
    """Partes del ensamble con su malla en coordenadas del ensamble."""
    step = StepFile(path)
    parts = step.parts()
    if not parts:
        raise MeshError(f"{Path(path).name}: no se encontraron partes en el ensamble")
    out = []
    with tempfile.TemporaryDirectory() as tmp:
        for i, (name, sdr, matrix) in enumerate(parts):
            if progress is not None:
                progress(i, len(parts), name)
            part_file = Path(tmp) / f"parte{i}.step"
            step.write_part(sdr, part_file)
            local = load_step(part_file, elements_per_circle, min_size_ratio)
            mesh = Mesh(skipped_faces=local.skipped_faces)
            for tri in local.triangles:
                mesh.add(*(_apply(matrix, p) for p in tri))
            out.append(Part(name, mesh, matrix))
    return out


def _unit(v: list[float]) -> list[float]:
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]


def _invert(m: Matrix) -> Matrix:
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(r[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def _apply(m: Matrix, p) -> tuple[float, float, float]:
    return tuple(m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3))
