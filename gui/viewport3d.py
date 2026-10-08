"""
Vista 3D con Qt3D (viene con PySide6: no es una dependencia nueva).

⚠️ Qt3D sin OpenGL no tira una excepción: hace caer el proceso entero
(segmentation fault). Por eso `opengl_available()` se consulta SIEMPRE antes
de crear un `Viewport3D`, y los tests no lo construyen nunca (corren con
QT_QPA_PLATFORM=offscreen, donde no hay OpenGL).

Ejes: Z hacia arriba, mm, origen en la base del robot.
"""

from __future__ import annotations

from PySide6.QtCore import QByteArray
from PySide6.QtGui import QColor, QMatrix4x4, QOffscreenSurface, QOpenGLContext, QVector3D
from PySide6.QtWidgets import QWidget

from sim.kinematics import Matrix
from sim.meshes import Mesh


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


class Viewport3D:
    """Escena: piso, robot (un nodo por eslabón) y objetos del layout."""

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
        self._keep: list = []  # referencias Python a lo que Qt3D usa por debajo
        self._robot_links: list = []
        self._objects: list = []
        self._path = None

        camera = self.window.camera()
        camera.lens().setPerspectiveProjection(40.0, 16 / 9, 10.0, 100000.0)
        camera.setUpVector(QVector3D(0, 0, 1))
        camera.setPosition(QVector3D(3200, -3000, 2400))
        camera.setViewCenter(QVector3D(500, 0, 700))
        controller = Qt3DExtras.QOrbitCameraController(self.root)
        controller.setCamera(camera)
        controller.setLinearSpeed(3000)
        controller.setLookSpeed(180)
        self._keep.append(controller)

        # Luces direccionales (como el sol): iluminan parejo. Una luz puntual
        # cerca del piso lo "quemaba" en un círculo blanco.
        for direction, intensity in (((-0.4, 0.5, -1.0), 0.8), ((0.6, -0.3, -0.5), 0.35)):
            light_entity = Qt3DCore.QEntity(self.root)
            light = Qt3DRender.QDirectionalLight(light_entity)
            light.setWorldDirection(QVector3D(*direction))
            light.setIntensity(intensity)
            light_entity.addComponent(light)
            self._keep += [light_entity, light]

        self._add_floor()
        self.window.setRootEntity(self.root)

    # -- construcción ---------------------------------------------------------------

    def _mesh_entity(self, mesh: Mesh, color: str):
        core, render, extras = self._core, self._render, self._extras
        entity = core.QEntity(self.root)
        geometry = core.QGeometry(entity)
        buffer = core.QBuffer(geometry)
        buffer.setData(QByteArray(mesh.interleaved()))
        count = len(mesh) * 3
        for name, offset in ((core.QAttribute.defaultPositionAttributeName(), 0),
                             (core.QAttribute.defaultNormalAttributeName(), 12)):
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
            self._keep.append(attr)
        renderer = render.QGeometryRenderer()
        renderer.setGeometry(geometry)
        renderer.setPrimitiveType(render.QGeometryRenderer.PrimitiveType.Triangles)
        material = extras.QPhongMaterial()
        material.setDiffuse(QColor(color))
        material.setAmbient(QColor(color).darker(250))
        material.setSpecular(QColor("#262626"))  # sin brillo: el piso reflejaba la luz
        material.setShininess(20)
        transform = core.QTransform()
        entity.addComponent(renderer)
        entity.addComponent(material)
        entity.addComponent(transform)
        self._keep += [geometry, buffer, renderer, material]
        return entity, transform

    def _add_floor(self) -> None:
        from sim.meshes import box

        floor = box((0, 0, -2), (6000, 6000, 4))
        for i in range(-6, 7):  # líneas cada 500 mm
            floor.extend(box((i * 500, 0, 0.5), (4, 6000, 1)))
            floor.extend(box((0, i * 500, 0.5), (6000, 4, 1)))
        self._mesh_entity(floor, "#3a4048")

    # -- robot ----------------------------------------------------------------------

    def set_robot(self, link_meshes: list[Mesh]) -> None:
        for entity, _ in self._robot_links:
            entity.setParent(None)
        colors = ["#5b6670", "#e8792b", "#e8792b", "#e8792b", "#d9dde1", "#d9dde1", "#30363d"]
        self._robot_links = [self._mesh_entity(m, colors[i % len(colors)])
                             for i, m in enumerate(link_meshes)]

    def set_joint_frames(self, frames: list[Matrix]) -> None:
        """`frames[i]` ubica el eslabón i+1 (la base no se mueve)."""
        for (_, transform), frame in zip(self._robot_links[1:], frames):
            transform.setMatrix(_qmatrix(frame))

    # -- trayectoria de la punta ---------------------------------------------------------

    def set_path(self, points: list[tuple[float, float, float]],
                 colors: list[tuple[float, float, float]]) -> None:
        """Línea por los puntos (sin iluminación: un color por vértice)."""
        import struct

        core, render, extras = self._core, self._render, self._extras
        if self._path is not None:
            self._path.setParent(None)
            self._path = None
        if len(points) < 2:
            return
        entity = core.QEntity(self.root)
        geometry = core.QGeometry(entity)
        data = bytearray()
        for p, c in zip(points, colors):
            data += struct.pack("<6f", *p, *c)
        buffer = core.QBuffer(geometry)
        buffer.setData(QByteArray(bytes(data)))
        for name, offset in ((core.QAttribute.defaultPositionAttributeName(), 0),
                             (core.QAttribute.defaultColorAttributeName(), 12)):
            attr = core.QAttribute(geometry)
            attr.setName(name)
            attr.setVertexBaseType(core.QAttribute.VertexBaseType.Float)
            attr.setVertexSize(3)
            attr.setAttributeType(core.QAttribute.AttributeType.VertexAttribute)
            attr.setBuffer(buffer)
            attr.setByteStride(24)
            attr.setByteOffset(offset)
            attr.setCount(len(points))
            geometry.addAttribute(attr)
            self._keep.append(attr)
        renderer = render.QGeometryRenderer()
        renderer.setGeometry(geometry)
        renderer.setPrimitiveType(render.QGeometryRenderer.PrimitiveType.LineStrip)
        material = extras.QPerVertexColorMaterial()
        entity.addComponent(renderer)
        entity.addComponent(material)
        self._keep += [geometry, buffer, renderer, material]
        self._path = entity

    # -- objetos del layout ------------------------------------------------------------

    def clear_objects(self) -> None:
        for entity, _ in self._objects:
            entity.setParent(None)
        self._objects = []

    def add_object(self, mesh: Mesh, color: str) -> int:
        self._objects.append(self._mesh_entity(mesh, color))
        return len(self._objects) - 1

    def place_object(self, index: int, matrix: Matrix) -> None:
        self._objects[index][1].setMatrix(_qmatrix(matrix))
