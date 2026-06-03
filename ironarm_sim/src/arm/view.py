"""UR5e arm view — MuJoCo GPU-accelerated rendering."""

import ironarm_sim as _rust
import mujoco
from PySide6.QtCore import QPoint, Qt, QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from .model import JOINT_NAMES, MODEL_PATH

class Arm3DView(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._model = None
        self._data = None
        self._scene = None
        self._context = None
        self._cam = mujoco.MjvCamera()
        self._opt = mujoco.MjvOption()
        self._state = (0.0,) * 9
        self._load_model()
        self._azimuth = 120.0
        self._elevation = 25.0
        self._distance = 2.5
        self.setMinimumSize(350, 300)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMouseTracking(True)
        self._poll_timer = QTimer(self); self._poll_timer.timeout.connect(self._poll); self._poll_timer.start(5)
        self._tick_timer = QTimer(self); self._tick_timer.timeout.connect(self._tick); self._tick_timer.start(16)
        QTimer.singleShot(0, self._init_gl)

    @staticmethod
    def _file_mtime():
        import os
        try: return os.path.getmtime(MODEL_PATH)
        except OSError: return 0.0

    def _load_model(self):
        self._model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self._data = mujoco.MjData(self._model)
        self._scene = mujoco.MjvScene(self._model, maxgeom=3000)
        self._context = None
        pass

    def _init_gl(self):
        if self._context is not None: return
        self.makeCurrent()
        self._context = mujoco.MjrContext(self._model, mujoco.mjtFontScale.mjFONTSCALE_150)
        gl = self.context().functions()
        gl.glClearColor(0.22, 0.22, 0.24, 1.0)
        self.doneCurrent()

    def _poll(self):
        state = _rust.poll_state()
        if state is not None: self._state = state

    def _tick(self):
        if self._data is None: return
        state = self._state
        for i, name in enumerate(JOINT_NAMES):
            self._data.joint(name).qpos[0] = state[i]
        tid = self._model.body("target").id
        jid = self._model.body_jntadr[tid]
        self._data.qpos[jid:jid+3] = (state[6], state[7], state[8])
        mujoco.mj_forward(self._model, self._data)
        self.update()

    def initializeGL(self):
        if self._model is None: return
        self._context = mujoco.MjrContext(self._model, mujoco.mjtFontScale.mjFONTSCALE_150)
        gl = self.context().functions()
        gl.glClearColor(0.22, 0.22, 0.24, 1.0)

    def paintGL(self):
        if self._context is None or self._scene is None: return
        gl = self.context().functions()
        gl.glClear(0x00004000 | 0x00000100)
        self._update_camera()
        mujoco.mjv_updateScene(self._model, self._data, self._opt, None, self._cam,
                               mujoco.mjtCatBit.mjCAT_ALL, self._scene)
        w, h = self.width(), self.height()
        mujoco.mjr_render(mujoco.MjrRect(0, 0, w, h), self._scene, self._context)

    def _update_camera(self):
        self._cam.lookat[:] = (-0.4, 0.0, 0.3)
        self._cam.distance = self._distance
        self._cam.azimuth = self._azimuth
        self._cam.elevation = -self._elevation

    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton: self._last_mouse = e.pos()
        super().mousePressEvent(e)
    def mouseMoveEvent(self, e):
        if self._last_mouse is not None:
            self._azimuth += (e.pos().x() - self._last_mouse.x()) * 0.4
            self._elevation += (e.pos().y() - self._last_mouse.y()) * 0.4
            self._elevation = max(-89, min(89, self._elevation))
            self._last_mouse = e.pos()
        super().mouseMoveEvent(e)
    def mouseReleaseEvent(self, e):
        self._last_mouse = None; super().mouseReleaseEvent(e)
    def wheelEvent(self, e):
        self._distance *= 1.0 - e.angleDelta().y() * 0.001
        self._distance = max(0.5, min(10, self._distance))
        super().wheelEvent(e)
