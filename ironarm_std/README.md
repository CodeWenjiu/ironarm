# ironarm_std — Copper 运行时 & 标准任务

`ironarm_std` 是 IronArm 项目的运行时层，负责：

- **Copper 运行时启动**（读取 `copperconfig.ron`，构建 DAG，运行主循环）
- **标准任务实现**（运动规划、关节插值、状态汇集）
- **轨迹生成**（圆、直线、多路径点）
- **跨语言共享状态**（锁无关环形缓冲，Copper → Python）

该 crate 依赖 `std`（需要时间、线程、文件系统），与 `no_std` 的 `ironarm_core` 互补。

---

## 目录结构

```
ironarm_std/
├── copperconfig.ron           # DAG 配置（任务 + 连接）
├── src/
│   ├── lib.rs                 # 运行时入口
│   ├── ringbuf.rs             # 锁无关环形缓冲
│   ├── trajectory.rs          # 轨迹类型 & 采样
│   └── tasks/
│       ├── mod.rs             # 任务模块入口
│       ├── motion_planner.rs  # 运动规划器
│       ├── joint_interpolator.rs  # 关节插值器
│       └── state_sink.rs      # 状态汇集器
```

---

## 数据流

```
MotionPlanner ──→ CartesianWaypoint ──→ IKSolver (core)
                                            │
                                     JointWaypoint { target, angles[0..5] }
                                            │
                    ┌───────────────────────┼───────────────────────┐
                    ▼                       ▼                       ▼
            JointInterpolator ×6     StateSink                  （fan-out）
                    │                       │
                    ▼                       │
            JointDriver ×6 (core)           │
                    │                       │
                    ▼                       ▼
               JointState ×6         ringbuf::write()
                    │
                    ▼
               StateSink
                    │
                    ▼
               ArmState (锁无关环形缓冲)
                    │
                    ▼
               Python / MuJoCo 可视化
```

---

## 模块说明

### `lib.rs` — 运行时入口

```rust
#[copper_runtime(config = "copperconfig.ron")]
struct IronArmCli {}
```

`#[copper_runtime]` 宏展开为完整的 DAG 构建和主循环。`run_tui()` 由 Python 侧的独立线程调用。

### `ringbuf.rs` — 锁无关共享状态

使用 **seqlock** 协议实现 Copper → Python 的单向数据传输：

- **写入方**（Copper）：无等待，两次原子写
- **读取方**（Python）：无锁，序号不一致则重试

数据布局 `ArmState` 是 `#[repr(C)]`，44 字节，兼容 Python `ctypes`。

### `trajectory.rs` — 轨迹生成

纯数学模块，`sample(t)` 将时间映射为笛卡尔坐标。支持：

| 类型 | 参数 | 说明 |
|------|------|------|
| `Circle` | cx, cy, r, z, period | 水平圆 |
| `TiltedCircle` | cx, cy, cz, nx, ny, nz, r, period | 任意平面内的圆 |
| `Linear` | start, end, duration | 直线 |
| `Waypoints` | Vec<(t, wp)>, looped | 多路径点插值 |

### 任务

| 任务 | 类型 | 输入 | 输出 | 职责 |
|------|------|------|------|------|
| `MotionPlanner` | CuSrcTask | — | `CartesianWaypoint` | 按配置生成路径点 |
| `JointInterpolator` | CuTask | `JointWaypoint` | `JointCommand` | 单关节线性平滑插值 |
| `StateSink` | CuSinkTask | `JointWaypoint` + 6×`JointState` | — | 汇集数据写入环形缓冲 |

---

## 依赖关系

```
ironarm_std
├── ironarm_core      (消息类型 + IK 求解)
├── ironarm_model     (运动学参数)
├── cu29              (Copper 运行时)
├── cu_consolemon     (控制台监控)
└── std               (时间、线程、文件系统)
```

下游：
- `ironarm_sim/rust_core` — PyO3 绑定，调用 `run_tui` 和 `ringbuf::read`
- `ironarm_tui` — 独立 TUI 程序
