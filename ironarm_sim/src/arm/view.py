"""3-D arm view — MuJoCo GPU-accelerated rendering via QOpenGLWidget.

Supports hot-reload: edit ``models/ironarm.xml`` and the arm updates live.
"""

import mujoco
from PySide6.QtCore import QPoint, Qt, QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget

import ironarm_sim as _rust  # type: ignore[import-untyped]
from .model import MODEL_PATH, trajectory


class Arm3DView(QOpenGLWidget):
    """MuJoCo-powered 3-D view — renders directly to the OpenGL framebuffer."""

    def __init__(self, parent=None):
        super().__init__(parent)

        # --- MuJoCo model (hot-reloadable) ---
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._scene: mujoco.MjvScene | None = None
        self._context: mujoco.MjrContext | None = None
        self._cam = mujoco.MjvCamera()
        self._opt = mujoco.MjvOption()
        self._needs_reload = False

        self._load_model()

        # --- Arm params ---
        self._l0 = 1.0
        self._l1 = 2.0
        self._base_z = 0.15
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

        # --- Hot-reload watcher ---
        self._watcher = QTimer(self)
        self._watcher.timeout.connect(self._poll_reload)
        self._watcher.start(1000)  # check every second
        self._mtime = self._file_mtime()

    # ------------------------------------------------------------------
    # Model loading / hot-reload
    # ------------------------------------------------------------------

    @staticmethod
    def _file_mtime() -> float:
        import os

        try:
            return os.path.getmtime(MODEL_PATH)
        except OSError:
            return 0.0

    def _load_model(self) -> None:
        # Save joint angles if we have an existing model
        saved = self._save_joint_angles()

        self._model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self._data = mujoco.MjData(self._model)
        self._scene = mujoco.MjvScene(self._model, maxgeom=100)
        self._context = None  # will be recreated in initializeGL

        # Restore joint angles (name-matched against new model)
        for jname, angle in saved:
            try:
                self._data.joint(jname).qpos[0] = angle
            except Exception:
                pass  # joint renamed or removed — ignore

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
        # Re-initialise the GL context (needs a current GL context).
        self.makeCurrent()
        self._context = mujoco.MjrContext(
            self._model, mujoco.mjtFontScale.mjFONTSCALE_150
        )
        self.doneCurrent()

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        self._apply_reload_if_needed()
        if self._data is None:
            return

        dt = 1.0 / 60.0
        self._t += dt

        result = _rust.compute_angles(self._l0, self._l1, self._base_z, self._t)
        if result is not None:
            j0, j1 = result
            tx, ty, tz = trajectory(self._t, self._base_z)
            self._data.joint("j0").qpos[0] = j0
            self._data.joint("j1").qpos[0] = -j1
            self._data.body("target").xpos[:] = (tx, ty, tz)

        mujoco.mj_forward(self._model, self._data)
        self.update()

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
        self._cam.lookat[:] = (0.7, 0.0, 0.3)
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
