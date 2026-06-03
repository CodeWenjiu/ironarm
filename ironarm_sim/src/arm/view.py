"""UR5e 机械臂 3D 视图——MuJoCo GPU 加速渲染。

工作流程：
- 每 5ms 从 Copper 共享内存拉取最新状态
- 每 16ms (~60fps) 用关节角度驱动 MuJoCo 模型，更新渲染
- 支持鼠标拖拽旋转/缩放视角
"""

import ironarm_sim as _rust
import mujoco
from PySide6.QtCore import Qt, QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from .model import JOINT_NAMES, MODEL_PATH


class Arm3DView(QOpenGLWidget):
    """MuJoCo 3D 机械臂视图。"""

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

        # 定时器：拉取 Copper 状态
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.start(5)

        # 定时器：更新 MuJoCo 物理 & 触发重绘
        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self._tick)
        self._tick_timer.start(16)

        QTimer.singleShot(0, self._init_gl)

    def _load_model(self):
        """加载 UR5e MuJoCo 模型。"""
        self._model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self._data = mujoco.MjData(self._model)
        self._scene = mujoco.MjvScene(self._model, maxgeom=3000)
        self._context = None

    def _init_gl(self):
        """初始化 MuJoCo 渲染上下文。"""
        if self._context is not None:
            return
        self.makeCurrent()
        self._context = mujoco.MjrContext(
            self._model, mujoco.mjtFontScale.mjFONTSCALE_150
        )
        gl = self.context().functions()
        gl.glClearColor(0.22, 0.22, 0.24, 1.0)
        self.doneCurrent()

    def _poll(self):
        """从 Copper 共享内存拉取最新状态。"""
        state = _rust.poll_state()
        if state is not None:
            self._state = state

    def _tick(self):
        """用最新关节角度驱动 MuJoCo 模型，触发重绘。"""
        if self._data is None:
            return
        state = self._state

        # 设置关节角度
        for i, name in enumerate(JOINT_NAMES):
            self._data.joint(name).qpos[0] = state[i]

        # 移动目标标记球
        tid = self._model.body("target").id
        jid = self._model.body_jntadr[tid]
        self._data.qpos[jid : jid + 3] = (state[6], state[7], state[8])

        mujoco.mj_forward(self._model, self._data)
        self.update()

    def initializeGL(self):
        """Qt OpenGL 初始化回调。"""
        if self._model is None:
            return
        self._context = mujoco.MjrContext(
            self._model, mujoco.mjtFontScale.mjFONTSCALE_150
        )
        gl = self.context().functions()
        gl.glClearColor(0.22, 0.22, 0.24, 1.0)

    def paintGL(self):
        """Qt OpenGL 绘制回调——渲染 MuJoCo 场景。"""
        if self._context is None or self._scene is None:
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
        mujoco.mjr_render(mujoco.MjrRect(0, 0, w, h), self._scene, self._context)

    def _update_camera(self):
        """更新相机参数。"""
        self._cam.lookat[:] = (-0.4, 0.0, 0.3)
        self._cam.distance = self._distance
        self._cam.azimuth = self._azimuth
        self._cam.elevation = -self._elevation

    # ---- 鼠标交互 ----

    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self._last_mouse = e.pos()
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if self._last_mouse is not None:
            self._azimuth += (e.pos().x() - self._last_mouse.x()) * 0.4
            self._elevation += (e.pos().y() - self._last_mouse.y()) * 0.4
            self._elevation = max(-89, min(89, self._elevation))
            self._last_mouse = e.pos()
        super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e):
        self._last_mouse = None
        super().mouseReleaseEvent(e)

    def wheelEvent(self, e):
        self._distance *= 1.0 - e.angleDelta().y() * 0.001
        self._distance = max(0.5, min(10, self._distance))
        super().wheelEvent(e)
