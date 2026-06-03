"""UR5e 机械臂 3D 视图——MuJoCo GPU 加速渲染 + 末端轨迹。

工作流程：
- 每 5ms 从 Copper 共享内存拉取最新状态
- 每 16ms (~60fps) 用关节角度驱动 MuJoCo 模型，更新渲染
- 末端执行器轨迹（绿球）和目标轨迹（橙球）实时叠加到场景中
- 支持鼠标拖拽旋转/缩放视角
"""

import collections

import ironarm_sim as _rust
import mujoco
import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from .model import JOINT_NAMES, MODEL_PATH

# 轨迹最大点数
TRAIL_LEN = 20  # 0.5 秒 @ ~60fps


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

        # 轨迹记录
        self._trail: collections.deque = collections.deque(maxlen=TRAIL_LEN)
        self._tgt_pos: np.ndarray = np.zeros(3)

        # 定时器：拉取 Copper 状态
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.start(5)

        # 定时器：更新物理 & 触发重绘
        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self._tick)
        self._tick_timer.start(16)

        QTimer.singleShot(0, self._init_gl)

    def _load_model(self):
        """加载 UR5e MuJoCo 模型。"""
        self._model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self._data = mujoco.MjData(self._model)
        self._scene = mujoco.MjvScene(self._model, maxgeom=4000)
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
        """用最新关节角度驱动 MuJoCo 模型，记录轨迹，触发重绘。"""
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

        # 记录末端轨迹 & 更新目标位置
        self._trail.append(self._data.site("attachment_site").xpos.copy())
        self._tgt_pos = np.array([state[6], state[7], state[8]])

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
        """Qt OpenGL 绘制回调——渲染 MuJoCo 场景 + 轨迹叠加。"""
        if self._context is None or self._scene is None:
            return
        gl = self.context().functions()
        gl.glClear(0x00004000 | 0x00000100)
        self._update_camera()

        # MuJoCo 标准场景渲染
        mujoco.mjv_updateScene(
            self._model,
            self._data,
            self._opt,
            None,
            self._cam,
            mujoco.mjtCatBit.mjCAT_ALL,
            self._scene,
        )

        # 叠加末端轨迹线（绿色实线）
        trail = list(self._trail)
        for i in range(0, len(trail) - 1):
            a = trail[i].astype(np.float64)
            b = trail[i + 1].astype(np.float64)
            mid = (a + b) / 2.0
            d = b - a
            length = float(np.linalg.norm(d))
            if length < 1e-6:
                continue
            # 构造旋转矩阵：z 轴对齐到 d
            z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            axis = np.cross(z, d / length)
            cos_a = np.dot(z, d / length)
            if abs(cos_a) > 0.9999:
                rmat = np.eye(3, dtype=np.float64)
            else:
                k = np.array(
                    [
                        [0, -axis[2], axis[1]],
                        [axis[2], 0, -axis[0]],
                        [-axis[1], axis[0], 0],
                    ],
                    dtype=np.float64,
                )
                rmat = np.eye(3, dtype=np.float64) + k + k @ k * (1.0 / (1.0 + cos_a))
            mujoco.mjv_initGeom(
                self._scene.geoms[self._scene.ngeom],
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[0.003, length / 2.0, 0.0],
                pos=mid,
                mat=rmat.flatten(),
                rgba=[0.2, 0.9, 0.3, 1.0],
            )
            self._scene.ngeom += 1

        # 当前目标位置（单个橙色球）
        mujoco.mjv_initGeom(
            self._scene.geoms[self._scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.02, 0, 0],
            pos=self._tgt_pos.astype(np.float64),
            mat=np.eye(3, dtype=np.float64).flatten(),
            rgba=[1.0, 0.5, 0.1, 1.0],
        )
        self._scene.ngeom += 1

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
