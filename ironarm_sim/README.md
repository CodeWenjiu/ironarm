# ironarm_sim — 机械臂仿真可视化

`ironarm_sim` 是 IronArm 项目的仿真 & 可视化层，负责：

- **Python ↔ Rust 桥接**（PyO3 绑定，启动 Copper 运行时）
- **MuJoCo 3D 渲染**（Qt + OpenGL，实时显示机械臂姿态）
- **无界面测试**（通过终端验证流水线是否正常）

---

## 目录结构

```
ironarm_sim/
├── rust_core/             # PyO3 绑定（Rust → Python）
│   └── src/lib.rs
├── src/                   # Python 源码
│   ├── main.py            # 应用入口（GUI 模式）
│   ├── headless_test.py   # 无界面测试脚本
│   └── arm/
│       ├── model.py       # MuJoCo 模型路径 & 关节名
│       └── view.py        # MuJoCo 3D 渲染视图
├── models/                # MuJoCo 模型文件（已移至 ironarm_model）
└── pyproject.toml         # Python 项目配置
```

---

## 架构

```
┌──────────────────────────────────────────────────────┐
│ Python (主线程)                                      │
│                                                      │
│  QApplication ──→ MainWindow ──→ Arm3DView           │
│                                      │               │
│                              QTimer(5ms) → _poll()   │
│                                      │               │
│                              QTimer(16ms)→ _tick()   │
│                                      │               │
│                              paintGL() → MuJoCo 渲染 │
└──────────────────────────────────────────────────────┘
         │                              ▲
         │ start_copper()               │ poll_state()
         ▼                              │
┌──────────────────────────────────────────────────────┐
│ Rust (后台线程)                                      │
│                                                      │
│  ironarm_std::run_tui()                              │
│      │                                               │
│      └── Copper DAG                                  │
│          MotionPlanner → IKSolver → JointInterp →   │
│          JointDriver → StateSink → shared_state      │
│                                      │               │
│                              shared_state::write()   │
└──────────────────────────────────────────────────────┘
```

---

## 运行方式

```bash
# GUI 模式（带 MuJoCo 3D 渲染）
cd ironarm_sim && uv run python src/main.py

# 无界面测试（终端输出）
cd ironarm_sim && uv run python src/headless_test.py
```

---

## 依赖链

```
ironarm_sim (PyO3)
├── ironarm_std (Copper 运行时 + 标准任务)
│   └── ironarm_core (运动学 + 消息类型)
├── pyo3 (Python 绑定)
├── mujoco (物理仿真 & 渲染)
└── PySide6 (Qt GUI)
```
