"""3-D arm view — MuJoCo GPU-accelerated rendering via QOpenGLWidget."""

import mujoco
from PySide6.QtCore import QPoint, Qt, QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from .model import MODEL_PATH


class Arm3DView(QOpenGLWidget):
    """MuJoCo-powered 3-D view — renders directly to the OpenGL framebuffer."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._scene: mujoco.MjvScene | None = None
        self._context: mujoco.MjrContext | None = None
        self._cam = mujoco.MjvCamera()
        self._opt = mujoco.MjvOption()
        self._needs_reload = False

        self._load_model()

        self._azimuth = 30.0
        self._elevation = 25.0
        self._distance = 3.5
        self._last_mouse: QPoint | None = None

        self.setMinimumSize(350, 300)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMouseTracking(True)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(16)
        QTimer.singleShot(0, self._init_gl)

        self._watcher = QTimer(self)
        self._watcher.timeout.connect(self._poll_reload)
        self._watcher.start(1000)
        self._mtime = self._file_mtime()

    @staticmethod
    def _file_mtime() -> float:
        import os

        try:
            return os.path.getmtime(MODEL_PATH)
        except OSError:
            return 0.0

    def _load_model(self) -> None:
        saved = self._save_joint_angles()
        self._model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self._data = mujoco.MjData(self._model)
        self._scene = mujoco.MjvScene(self._model, maxgeom=100)
        self._context = None
        for jname, angle in saved:
            try:
                self._data.joint(jname).qpos[0] = angle
            except Exception:
                pass

    def _save_joint_angles(self) -> list[tuple[str, float]]:
        if self._model is None or self._data is None:
            return []
        return [
            (self._model.joint(i).name, self._data.joint(i).qpos[0])
            for i in range(self._model.njnt)
        ]

    def _poll_reload(self) -> None:
        mtime = self._file_mtime()
        if mtime != self._mtime:
            self._mtime = mtime
            self._needs_reload = True

    def _apply_reload_if_needed(self) -> None:
        if not self._needs_reload:
            return
        self._needs_reload = False
        self._load_model()
        self.makeCurrent()
        self._context = mujoco.MjrContext(
            self._model, mujoco.mjtFontScale.mjFONTSCALE_150
        )
        self.doneCurrent()

    def _init_gl(self) -> None:
        if self._context is not None:
            return
        self.makeCurrent()
        self._context = mujoco.MjrContext(
            self._model, mujoco.mjtFontScale.mjFONTSCALE_150
        )
        gl = self.context().functions()
        gl.glClearColor(0.22, 0.22, 0.24, 1.0)
        self.doneCurrent()

    # ------------------------------------------------------------------
    # Simulation (4-DOF arm: j0..j3)
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        self._apply_reload_if_needed()
        if self._data is None:
            return
        j0, j1, j2, j3, wx, wy, wz = getattr(
            self, "_angles", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        )
        self._data.joint("j0").qpos[0] = j0
        self._data.joint("j1").qpos[0] = j1
        self._data.joint("j2").qpos[0] = j2
        self._data.joint("j3").qpos[0] = j3
        bid = self._model.body("target").id
        jid = self._model.body_jntadr[bid]
        self._data.qpos[jid : jid + 3] = (wx, wy, wz)
        mujoco.mj_forward(self._model, self._data)
        self.update()

    def _on_angles(
        self,
        j0: float,
        j1: float,
        j2: float,
        j3: float,
        wx: float,
        wy: float,
        wz: float,
    ) -> None:
        self._angles = (j0, j1, j2, j3, wx, wy, wz)

    # ------------------------------------------------------------------
    # OpenGL
    # ------------------------------------------------------------------

    def initializeGL(self) -> None:
        if self._model is None:
            return
        self._context = mujoco.MjrContext(
            self._model, mujoco.mjtFontScale.mjFONTSCALE_150
        )
        gl = self.context().functions()
        gl.glClearColor(0.22, 0.22, 0.24, 1.0)

    def paintGL(self) -> None:
        if self._context is None or self._scene is None:
            return
        gl = self.context().functions()
        gl.glClear(0x00004000 | 0x00000100)
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
        mujoco.mjr_render(mujoco.MjrRect(0, 0, w, h), self._scene, self._context)

    def _update_camera(self) -> None:
        self._cam.lookat[:] = (1.0, 0.0, 0.8)
        self._cam.distance = self._distance
        self._cam.azimuth = self._azimuth
        self._cam.elevation = -self._elevation

    # ------------------------------------------------------------------
    # Mouse
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
