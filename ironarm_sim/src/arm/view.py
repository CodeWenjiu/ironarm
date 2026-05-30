"""3-D arm view — MuJoCo GPU-accelerated rendering via QOpenGLWidget."""

import mujoco
from PySide6.QtCore import QPoint, Qt, QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from .model import MODEL_PATH, compute_angles


class Arm3DView(QOpenGLWidget):
    """MuJoCo-powered 3-D view — renders directly to the OpenGL framebuffer."""

    def __init__(self, parent=None):
        super().__init__(parent)

        # --- MuJoCo model ---
        self._model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self._data = mujoco.MjData(self._model)
        self._scene = mujoco.MjvScene(self._model, maxgeom=100)
        self._context: mujoco.MjrContext | None = None
        self._cam = mujoco.MjvCamera()
        self._opt = mujoco.MjvOption()

        # --- Arm params ---
        self._l0 = 1.0
        self._l1 = 2.0
        self._base_y = 0.15
        self._t = 0.0

        # --- Orbit camera ---
        self._azimuth = 30.0
        self._elevation = 25.0
        self._distance = 3.5
        self._last_mouse: QPoint | None = None

        self.setMinimumSize(350, 300)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMouseTracking(True)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(16)  # ~60 fps

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        dt = 1.0 / 60.0
        self._t += dt

        result = compute_angles(self._t, self._l0, self._l1, self._base_y)
        if result is not None:
            j0, j1, tx, ty, tz = result
            self._data.joint("j0").qpos[0] = j0
            self._data.joint("j1").qpos[0] = -j1
            self._data.body("target").xpos[:] = (tx, ty, tz)

        mujoco.mj_forward(self._model, self._data)
        self.update()  # schedule paintGL

    # ------------------------------------------------------------------
    # OpenGL
    # ------------------------------------------------------------------

    def initializeGL(self) -> None:
        self._context = mujoco.MjrContext(
            self._model, mujoco.mjtFontScale.mjFONTSCALE_150
        )
        gl = self.context().functions()
        gl.glClearColor(0.22, 0.22, 0.24, 1.0)

    def paintGL(self) -> None:
        if self._context is None:
            return

        gl = self.context().functions()
        gl.glClear(0x00004000 | 0x00000100)  # GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT

        self._update_camera()
        mujoco.mjv_updateScene(
            self._model,
            self._data,
            self._opt,
            None,
            self._cam,
            mujoco.mjtCatBit.mjCAT_ALL,
            self._scene,
        )

        w, h = self.width(), self.height()
        viewport = mujoco.MjrRect(0, 0, w, h)
        mujoco.mjr_render(viewport, self._scene, self._context)

    def _update_camera(self) -> None:
        self._cam.lookat[:] = (0.0, 0.5, 0.0)
        self._cam.distance = self._distance
        self._cam.azimuth = self._azimuth
        self._cam.elevation = -self._elevation

    # ------------------------------------------------------------------
    # Mouse — orbit
    # ------------------------------------------------------------------

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._last_mouse = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._last_mouse is not None:
            dx = event.pos().x() - self._last_mouse.x()
            dy = event.pos().y() - self._last_mouse.y()
            self._azimuth += dx * 0.4
            self._elevation += dy * 0.4
            self._elevation = max(-89.0, min(89.0, self._elevation))
            self._last_mouse = event.pos()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        self._last_mouse = None
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y()
        self._distance *= 1.0 - delta * 0.001
        self._distance = max(0.5, min(15.0, self._distance))
        super().wheelEvent(event)
