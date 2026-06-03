# ironarm_core — 机械臂核心算法库

`ironarm_core` 是 IronArm 项目的核心算法层，负责：

- **逆运动学求解**（IK-Geo 解析法：三平行轴 + 两相交轴）
- **正运动学**（指数积公式 / Product of Exponentials）
- **消息类型定义**（CartesianWaypoint、JointWaypoint、JointCommand、JointState）
- **铜任务实现**（IKSolver、JointDriver）

该 crate 为 `#![no_std]` 设计，仅在有 `std` feature 时引入 Copper 运行时依赖。

---

## 目录结构

```
ironarm_core/src/
├── lib.rs                  # crate 入口
├── math.rs                 # 通用数学工具（指数平滑插值）
├── messages.rs             # 消息类型定义（流水线合约）
├── ik_geo.rs               # 解析逆运动学（IK-Geo 算法）
└── tasks/
    ├── mod.rs              # 任务模块入口
    ├── ik_solver.rs        # IKSolver 铜任务
    └── joint_driver.rs     # JointDriver 铜任务
```

---

## 数据流全景

整个控制流水线按以下路径传递数据：

```
┌─────────────┐     CartesianWaypoint      ┌──────────┐
│ MotionPlanner│ ──────────────────────────→│ IKSolver │  ×6（每关节一个实例）
│ (ironarm_std)│   { x, y, z }              │          │
└─────────────┘                             └────┬─────┘
                                                  │
                                         JointWaypoint
                                         { angles: [qᵢ] }
                                                  │
                                                  ▼
┌─────────────────┐     JointCommand      ┌──────────────────┐
│ JointInterpolator│ ←────────────────────│  （同上 ×6）      │
│ (ironarm_std)    │  { angle, vel, stiff }│                  │
└────────┬────────┘                      └──────────────────┘
         │
         │  JointCommand（经过平滑插值）
         ▼
┌─────────────┐     JointState            ┌───────────┐
│ JointDriver │ ─────────────────────────→│ StateSink  │
│ (×6)        │   { current_angle, vel }  │(ironarm_std)│
└─────────────┘                           └─────┬─────┘
                                                 │
                                          ArmState（锁无关环形缓冲）
                                                 │
                                                 ▼
                                          ┌──────────┐
                                          │ Python   │
                                          │ MuJoCo   │
                                          │ 可视化   │
                                          └──────────┘
```

### 各环节说明

| 环节 | 输入 | 输出 | 职责 |
|------|------|------|------|
| **MotionPlanner** | 时间 t | `CartesianWaypoint` (x,y,z) | 生成轨迹路径点（圆、直线等） |
| **IKSolver** | `CartesianWaypoint` | `JointWaypoint` (单关节角度) | 解析逆运动学：笛卡尔 → 关节角 |
| **JointInterpolator** | `JointWaypoint` | `JointCommand` | 平滑插值，避免关节角度突变 |
| **JointDriver** | `JointCommand` | `JointState` | 仿真/实机驱动（目前透传） |
| **StateSink** | 6×`JointState` + `CartesianWaypoint` | 环形缓冲 `ArmState` | 汇集数据供 Python 侧读取 |

---

## 消息类型

所有消息定义在 `messages.rs` 中，通过 `bincode` 序列化在 Copper DAG 中传递。

| 消息类型 | 字段 | 方向 | 说明 |
|----------|------|------|------|
| `CartesianWaypoint` | `x, y, z: f32` | MotionPlanner → IKSolver | 笛卡尔空间目标点 |
| `JointWaypoint` | `angles: Vec<f32>` | IKSolver → JointInterpolator | 单个关节的目标角度 |
| `JointCommand` | `target_angle, target_velocity, stiffness: f32` | JointInterpolator → JointDriver | 关节驱动指令 |
| `JointState` | `current_angle, current_velocity: f32` | JointDriver → StateSink | 当前关节状态 |

> **注意**：`JointWaypoint` 仅包含**单个**关节的角度（`angles` 长度为 1）。
> 这是因为在 Copper DAG 中，每个关节有独立的 IKSolver → Interpolator → Driver 管道。
> 6 个 IKSolver 实例各自独立计算完整的 6-DOF 解，但只输出自己负责的那个关节的角度。

---

## 逆运动学求解器 (`ik_geo.rs`)

### 运动学模型

采用**指数积（PoE）**表示，每个关节由一对参数描述：

- `h[i]`：关节螺旋轴（零位形下基坐标系中的单位向量）
- `p[i]`：连杆偏移（关节 i → 关节 i+1 的向量）

正运动学公式：

```
R = I
pos = p[0]
for i in 0..6:
    R = R × Rot(h[i], q[i])
    pos = pos + R × p[i+1]
```

### IK 算法

实现 IK-Geo 论文中"三平行轴 + 两相交轴"的闭式解法，适用于 UR5e 等 Pieper 型机械臂。

求解步骤（对每个路径点）：

| 步骤 | 关节 | 方法 |
|------|------|------|
| 1 | q1 (shoulder_pan) | 子问题 4：旋转变换 |
| 2 | q5 (wrist_2) | 子问题 4：旋转变换 |
| 3 | θ₁₄ (q2+q3+q4 合成) | 子问题 1：点旋转匹配 |
| 4 | q3 (elbow) | 子问题 3：余弦定理 |
| 5 | q2 (shoulder_lift) | 子问题 1：点旋转匹配 |
| 6 | q4 (wrist_1) | 代数：q4 = θ₁₄ - q2 - q3 |
| 7 | q6 (wrist_3) | 子问题 1：点旋转匹配 |

每次调用返回最多 8 组解（对应各子问题解的组合）。

### 验证

- `test_fk_q0`：零位形正运动学正确性
- `test_fk_ik_roundtrip`：FK→IK 往返验证（误差 < 5cm）

---

## 运动学参数来源

`h` 和 `p` 参数由 `ironarm_model` crate 在**编译期**自动生成：

```
ur5e.xml ──→ build.rs ──→ poe_params.rs ──→ SCREW_AXES / LINK_OFFSETS
```

`build.rs` 遍历 MuJoCo body 树（q=0），处理：
- MuJoCo 默认类继承（解决 joint axis 的默认值）
- 四元数链式变换（解决 body quat 的坐标系旋转）
- attachment_site 工具法兰位置

改模型只需修改 `ur5e.xml`，`cargo build` 自动重生成参数，无需修改 Rust 代码。

---

## 添加新机械臂

1. 在 `ironarm_model/` 下放置新模型的 MuJoCo XML
2. 修改 `build.rs` 的 `xml_path` 指向新文件
3. 若新模型也满足"关节 2,3,4 平行 + 关节 5,6 相交"，IK-Geo 直接适用
4. 若结构不同，需在 `ik_geo.rs` 中实现对应的解析求解器

---

## 依赖关系

```
ironarm_core
├── ironarm_model (编译期参数生成)
├── cu29 (Copper 运行时，仅 std feature)
├── bincode (消息序列化)
└── serde (配置反序列化，仅 std feature)
```

上游依赖：
- `ironarm_std` 使用 `ironarm_core` 的任务和消息类型
- `ironarm_sim` (Python 绑定) 通过 `ironarm_std` 间接调用
