"""
Tope de triángulos de un STEP y el buffer de la GPU con numpy (sim/meshes.py).

El tope se prueba con un gmsh de mentira que da tantos triángulos como diga una
función del detalle: mallar de verdad algo que se pase del tope tarda minutos.
"""

import random

import pytest

import sim.meshes as meshes
from sim.meshes import Mesh, _mesh_within_budget, box, cylinder, load_step

DIAG = 1000.0


class FakeGmsh:
    """Lo que `_mesh_within_budget` le pide a gmsh. `triangles(ratio)` dice
    cuántos triángulos da mallar con detalle mínimo = DIAG / ratio."""

    def __init__(self, triangles):
        self.triangles = triangles
        self.min_size = None
        self.count = 0
        self.meshed = []  # relaciones mallas, en orden
        self.option = self
        self.model = self
        self.mesh = self

    def setNumber(self, name, value):
        assert name == "Mesh.MeshSizeMin"
        self.min_size = value

    def clear(self):
        self.count = 0

    def generate(self, dim):
        ratio = DIAG / self.min_size
        self.meshed.append(round(ratio, 3))
        self.count = self.triangles(ratio)

    def getElements(self, dim):
        return [2], [list(range(self.count))], [[0] * (3 * self.count)]


def test_refines_while_the_prediction_fits():
    fake = FakeGmsh(lambda r: int(10 * r))  # crece lineal: la predicción (x4) sobreestima
    ratio, skipped = _mesh_within_budget(fake, DIAG, 2000, 10_000)
    # 200 -> 2000 tri; 400 predice 8000 y da 4000; 800 predice 16000: se queda.
    assert ratio == 400 and skipped == 0
    assert fake.meshed == [200, 400]
    assert fake.count == 4000


def test_never_finer_than_asked():
    fake = FakeGmsh(lambda r: 10)
    ratio, _ = _mesh_within_budget(fake, DIAG, 1000, 10_000)
    assert ratio == 1000
    assert fake.meshed == [200, 400, 800, 1000]
    ratio, _ = _mesh_within_budget(FakeGmsh(lambda r: 10), DIAG, 150, 10_000)
    assert ratio == 150  # pedido más grueso que el arranque: se arranca ahí


def test_goes_back_when_the_prediction_falls_short():
    # Salto brusco que el exponente 2 no predice: 200 da 100, 400 da 50.000.
    fake = FakeGmsh(lambda r: 100 if r <= 200 else 50_000)
    ratio, _ = _mesh_within_budget(fake, DIAG, 2000, 10_000)
    assert ratio == 200
    assert fake.meshed == [200, 400, 200]  # se rehízo la malla buena
    assert fake.count == 100


def test_coarsens_when_the_first_mesh_is_over_budget():
    fake = FakeGmsh(lambda r: int(r ** 1.7))  # 200 -> ~8200
    ratio, _ = _mesh_within_budget(fake, DIAG, 2000, 1_000)
    assert ratio < 200 and fake.count <= 1_000
    assert len(fake.meshed) <= 3


def test_coarsening_stops_at_the_floor():
    fake = FakeGmsh(lambda r: 1_000_000)  # no baja nunca
    ratio, _ = _mesh_within_budget(fake, DIAG, 2000, 1_000)
    assert ratio == meshes.STEP_COARSEST_RATIO
    assert fake.count == 1_000_000  # se pasa, pero termina


def _box_step(tmp_path):
    gmsh = pytest.importorskip("gmsh")
    path = tmp_path / "caja.step"
    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.addBox(0, 0, 0, 100, 50, 20)
        gmsh.model.occ.synchronize()
        gmsh.write(str(path))
    finally:
        gmsh.finalize()
    return path


def test_load_step_reports_the_detail_used(tmp_path):
    path = _box_step(tmp_path)
    mesh = load_step(path)
    diag = (100 ** 2 + 50 ** 2 + 20 ** 2) ** 0.5
    assert mesh.detail_mm == pytest.approx(diag / 2000)
    assert not mesh.coarsened
    tight = load_step(path, max_triangles=1)  # ninguna malla entra
    assert tight.coarsened
    assert tight.detail_mm == pytest.approx(diag / meshes.STEP_COARSEST_RATIO)
    assert len(tight) > 0


# --- buffer de la GPU ------------------------------------------------------------------


def _loop_interleaved(mesh, monkeypatch):
    with monkeypatch.context() as m:
        m.setattr(meshes, "np", None)
        return mesh.interleaved()


def test_numpy_interleaved_is_byte_identical(monkeypatch):
    pytest.importorskip("numpy")
    rnd = random.Random(7)
    mesh = Mesh()
    for _ in range(500):
        mesh.add(*[(rnd.uniform(-900, 900), rnd.uniform(-900, 900), rnd.uniform(-900, 900))
                   for _ in range(3)])
    mesh.add((0, 0, 0), (1, 1, 1), (2, 2, 2))  # degenerado: normal (0, 0, 1)
    mesh.add((5, 5, 5), (5, 5, 5), (5, 5, 5))
    mesh.extend(box((0, 0, 10), (100, 50, 20))).extend(cylinder((0, 0, 0), (0, 0, 100), 10))
    assert mesh.interleaved() == _loop_interleaved(mesh, monkeypatch)


def test_interleaved_of_an_empty_mesh(monkeypatch):
    assert Mesh().interleaved() == b"" == _loop_interleaved(Mesh(), monkeypatch)
