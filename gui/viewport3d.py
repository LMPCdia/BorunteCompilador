"""
Vista 3D con Qt3D (viene con PySide6: no es una dependencia nueva).

⚠️ Qt3D sin OpenGL no tira una excepción: hace caer el proceso entero
(segmentation fault). Por eso `opengl_available()` se consulta SIEMPRE antes
de crear un `Viewport3D`, y los tests no lo construyen nunca (corren con
QT_QPA_PLATFORM=offscreen, donde no hay OpenGL).

Cada cosa dibujada es un `_Node`: la entidad de Qt3D más las referencias
Python a sus componentes. Se guardan JUNTAS y se sueltan juntas al borrar el
nodo; antes iban a una lista global que crecía sin límite (cada edición de la
tabla de piezas volvía a subir la malla entera a la GPU).

Ejes: Z hacia arriba, mm, origen en la base del robot.
"""

from __future__ import annotations

import struct
from typing import Callable

from PySide6.QtCore import QByteArray
from PySide6.QtGui import QColor, QMatrix4x4, QOffscreenSurface, QOpenGLContext, QVector3D
from PySide6.QtWidgets import QWidget

from gui.camera_nav import CameraNav, NavigationFilter
from sim.kinematics import Matrix
from sim.meshes import Mesh, box

FOV_DEG = 40.0
AXIS_COLORS = ((1.0, 0.25, 0.25), (0.3, 0.9, 0.3), (0.35, 0.55, 1.0))  # X, Y, Z


def opengl_available() -> bool:
    """True si se puede crear un contexto OpenGL de verdad."""
    context = QOpenGLContext()
    if not context.create():
        return False
    surface = QOffscreenSurface()
    surface.create()
    ok = surface.isValid() and context.makeCurrent(surface)
    if ok:
        context.doneCurrent()
    return ok


def _qmatrix(m: Matrix) -> QMatrix4x4:
    return QMatrix4x4(*[float(m[i][j]) for i in range(4) for j in range(4)])


class _Node:
    def __init__(self, entity, transform, refs: list) -> None:
        self.entity = entity
        self.transform = transform
        self.refs = refs  # componentes que Qt3D usa por debajo: viven con la entidad
        self.material = None

    def remove(self) -> None:
        self.entity.setParent(None)
        self.entity.deleteLater()
        self.refs = []
        self.material = None


class Viewport3D:
    """Escena: piso, robot (un nodo por eslabón), piezas, trayectoria y ejes."""

    def __init__(self) -> None:
        from PySide6.Qt3DCore import Qt3DCore
        from PySide6.Qt3DExtras import Qt3DExtras
        from PySide6.Qt3DRender import Qt3DRender

        self._core, self._extras, self._render = Qt3DCore, Qt3DExtras, Qt3DRender
        self.window = Qt3DExtras.Qt3DWindow()
        self.window.defaultFrameGraph().setClearColor(QColor("#2b2f33"))
        self.widget = QWidget.createWindowContainer(self.window)
        self.widget.setMinimumSize(400, 300)

        self.root = Qt3DCore.QEntity()
        self._static: list = []  # cámara, luces, piso: viven lo mismo que la vista
        self._robot_links: list[_Node] = []
        self._objects: list[_Node] = []
        self._path: _Node | None = None
        self._axes: _Node | None = None

        self.camera = self.window.camera()
        self.camera.lens().setPerspectiveProjection(FOV_DEG, 16 / 9, 10.0, 200000.0)
        self.camera.setUpVector(QVector3D(0, 0, 1))
        # Navegación como en Inventor/AutoCAD (gui/camera_nav.py), en vez del
        # control de órbita de Qt3D.
        self.nav = CameraNav((3000.0, -2800.0, 2800.0), (600.0, 0.0, 600.0), FOV_DEG)
        self.on_fit: Callable[[], None] = lambda: self.set_view("iso")
        self._nav_filter = NavigationFilter(self.nav, self._apply_camera, lambda: self.on_fit())
        self.window.installEventFilter(self._nav_filter)
        self.set_view("iso")

        # Luces direccionales (como el sol): iluminan parejo. Una luz puntual
        # cerca del piso lo "quemaba" en un círculo blanco.
        for direction, intensity in (((-0.4, 0.5, -1.0), 0.8), ((0.6, -0.3, -0.5), 0.35)):
            light_entity = Qt3DCore.QEntity(self.root)
            light = Qt3DRender.QDirectionalLight(light_entity)
            light.setWorldDirection(QVector3D(*direction))
            light.setIntensity(intensity)
            light_entity.addComponent(light)
            self._static += [light_entity, light]

        self._add_floor()
        self.window.setRootEntity(self.root)

    # -- cámara ------------------------------------------------------------------------

    def set_view(self, name: str, center=(600.0, 0.0, 600.0), size: float = 2500.0) -> None:
        """'iso', 'arriba', 'frente', 'lado': mirando a `center` desde una
        distancia que deja ver un objeto de `size` mm."""
        cx, cy, cz = center
        d = max(size, 500.0) * 1.6
        if name == "arriba":
            eye = (cx, cy - 1, cz + d)
        elif name == "frente":
            eye = (cx + d, cy, cz)
        elif name == "lado":
            eye = (cx, cy - d, cz)
        elif name == "atras":
            eye = (cx - d, cy, cz)
        elif name == "izquierda":
            eye = (cx, cy + d, cz)
        else:
            eye = (cx + d * 0.75, cy - d * 0.7, cz + d * 0.55)
        self.nav.eye, self.nav.center = eye, (cx, cy, cz)
        self._apply_camera()

    def fit(self, lo, hi) -> None:
        """Encuadrar la caja sin cambiar desde dónde se mira (como Inventor)."""
        self.nav.fit(lo, hi)
        self._apply_camera()

    def _apply_camera(self) -> None:
        nav = self.nav
        self.camera.lens().setPerspectiveProjection(FOV_DEG, nav.width / nav.height, 10.0, 200000.0)
        self.camera.setPosition(QVector3D(*nav.eye))
        self.camera.setViewCenter(QVector3D(*nav.center))
        self.camera.setUpVector(QVector3D(*nav.up))

    # -- construcción ---------------------------------------------------------------

    def _geometry_node(self, data: bytes, count: int, second_attr: str, primitive,
                       material) -> _Node:
        core, render = self._core, self._render
        entity = core.QEntity(self.root)
        geometry = core.QGeometry(entity)
        buffer = core.QBuffer(geometry)
        buffer.setData(QByteArray(data))
        refs = [geometry, buffer, material]
        for name, offset in ((core.QAttribute.defaultPositionAttributeName(), 0), (second_attr, 12)):
            attr = core.QAttribute(geometry)
            attr.setName(name)
            attr.setVertexBaseType(core.QAttribute.VertexBaseType.Float)
            attr.setVertexSize(3)
            attr.setAttributeType(core.QAttribute.AttributeType.VertexAttribute)
            attr.setBuffer(buffer)
            attr.setByteStride(24)
            attr.setByteOffset(offset)
            attr.setCount(count)
            geometry.addAttribute(attr)
            refs.append(attr)
        renderer = render.QGeometryRenderer()
        renderer.setGeometry(geometry)
        renderer.setPrimitiveType(primitive)
        transform = core.QTransform()
        entity.addComponent(renderer)
        entity.addComponent(material)
        entity.addComponent(transform)
        refs += [renderer, transform]
        return _Node(entity, transform, refs)

    def _mesh_node(self, mesh: Mesh, color: str, matte: bool = False) -> _Node:
        material = self._extras.QPhongMaterial()
        material.setDiffuse(QColor(color))
        material.setAmbient(QColor(color).darker(250))
        # Sin brillo en el piso: con Phong, una superficie grande y plana
        # "quema" una mancha blanca donde refleja la luz.
        material.setSpecular(QColor("#000000" if matte else "#202020"))
        material.setShininess(80)
        node = self._geometry_node(
            mesh.interleaved(), len(mesh) * 3, self._core.QAttribute.defaultNormalAttributeName(),
            self._render.QGeometryRenderer.PrimitiveType.Triangles, material)
        node.material = material
        return node

    def _lines_node(self, points, colors, strip: bool) -> _Node:
        data = bytearray()
        for p, c in zip(points, colors):
            data += struct.pack("<6f", *p, *c)
        primitive = (self._render.QGeometryRenderer.PrimitiveType.LineStrip if strip
                     else self._render.QGeometryRenderer.PrimitiveType.Lines)
        return self._geometry_node(
            bytes(data), len(points), self._core.QAttribute.defaultColorAttributeName(),
            primitive, self._extras.QPerVertexColorMaterial())

    def _add_floor(self) -> None:
        floor = box((0, 0, -2), (8000, 8000, 4))
        for i in range(-8, 9):  # líneas cada 500 mm
            floor.extend(box((i * 500, 0, 0.5), (10, 8000, 1)))
            floor.extend(box((0, i * 500, 0.5), (8000, 10, 1)))
        self._static.append(self._mesh_node(floor, "#3a4048", matte=True))

    # -- robot ----------------------------------------------------------------------

    def set_robot(self, link_meshes: list[Mesh]) -> None:
        for node in self._robot_links:
            node.remove()
        colors = ["#5b6670", "#e8792b", "#e8792b", "#e8792b", "#d9dde1", "#d9dde1", "#30363d"]
        self._robot_links = [self._mesh_node(m, colors[i % len(colors)])
                             for i, m in enumerate(link_meshes)]

    def set_joint_frames(self, frames: list[Matrix]) -> None:
        """`frames[i]` ubica el eslabón i+1 (la base no se mueve)."""
        for node, frame in zip(self._robot_links[1:], frames):
            node.transform.setMatrix(_qmatrix(frame))

    # -- trayectoria y ejes ---------------------------------------------------------------

    def set_path(self, points: list[tuple[float, float, float]],
                 colors: list[tuple[float, float, float]]) -> None:
        """Línea por los puntos (sin iluminación: un color por vértice)."""
        if self._path is not None:
            self._path.remove()
            self._path = None
        if len(points) >= 2:
            self._path = self._lines_node(points, colors, strip=True)

    def set_axes(self, frames: list[Matrix], length: float = 200.0) -> None:
        """Tríada X (rojo), Y (verde), Z (azul) en cada transformación."""
        if self._axes is not None:
            self._axes.remove()
            self._axes = None
        points, colors = [], []
        for m in frames:
            origin = (m[0][3], m[1][3], m[2][3])
            for axis, color in enumerate(AXIS_COLORS):
                tip = tuple(origin[i] + m[i][axis] * length for i in range(3))
                points += [origin, tip]
                colors += [color, color]
        if points:
            self._axes = self._lines_node(points, colors, strip=False)

    # -- piezas del layout ------------------------------------------------------------

    def clear_objects(self) -> None:
        for node in self._objects:
            node.remove()
        self._objects = []

    def add_object(self, mesh: Mesh, color: str) -> int:
        self._objects.append(self._mesh_node(mesh, color))
        return len(self._objects) - 1

    def place_object(self, index: int, matrix: Matrix) -> None:
        self._objects[index].transform.setMatrix(_qmatrix(matrix))

    def set_object_color(self, index: int, color: str) -> None:
        """Para marcar en vivo una pieza contra la que el robot choca."""
        material = self._objects[index].material
        material.setDiffuse(QColor(color))
        material.setAmbient(QColor(color).darker(250))

    def object_count(self) -> int:
        return len(self._objects)
