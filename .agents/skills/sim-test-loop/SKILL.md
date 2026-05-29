---
name: sim-test-loop
description: Closed-loop testing for ironarm_sim features. Use when modifying sim logic (Rhai, physics, config loading) — write a unit test, run cargo test, fix errors, iterate, only then ask user to verify GUI. Agent cannot run GUI programs.
---

# Sim 闭环测试流程

## 核心原则

**Agent 无法运行 GUI 程序（Bevy 窗口需要 X11/Wayland display）。** 所有 sim 功能变更必须在提交给用户之前通过 `cargo test` 验证。不要依赖"反正用户会跑 GUI 看到效果"。

## 测试闭环步骤

```
1. 改代码
   ↓
2. 写/跑 cargo test（不依赖 GUI）
   ↓
3. 测试失败 → 读错误消息 → 修代码 → 回到 2
   ↓
4. 测试通过 → cargo check 确认无 warning
   ↓
5. 交付用户 → 用户跑 GUI 验证
```

## 常用测试命令

```bash
# 跑单个测试，不捕获输出（能看到 println!/eprintln!）
cargo test -p ironarm_sim -- <test_name> --nocapture

# 跑 motion 模块所有测试
cargo test -p ironarm_sim -- motion::tests --nocapture

# 只编译检查（比完整 test 快）
cargo check -p ironarm_sim
```

## 各模块测试策略

### Rhai 脚本（motion.rs）

已在 `motion.rs` 底部预留 `#[cfg(test)] mod tests`。添加测试：

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motion_script() {
        let motion = RhaiMotion::load().expect("load");
        match motion.compute_angles(0, 0.016) {
            Some(a) => println!("tick=0: [{:.4}, {:.4}]", a[0], a[1]),
            None => panic!("tick(0) returned None"),
        }
    }
}
```

调试时临时加 `eprintln!` 到错误分支：

```rust
eprintln!("[RhaiMotion] error: {}", e);  // 可见于 cargo test --nocapture
```

### 配置加载（arm_config.rs / world/mod.rs）

```rust
#[test]
fn test_config_load() {
    let cfg: ArmConfig = ron::de::from_bytes(
        std::fs::read("assets/arm_config.ron").unwrap().as_bytes()
    ).expect("parse");
    // 验证 anchor 世界坐标对齐
    // anchor1_world == anchor2_world
}
```

### 物理参数（avian3d 交互）

avian3d 的 AABB 断言崩溃无法用单元测试复现（需要完整 ECS），但可以用 `cargo check --release` 确认代码路径无类型错误。

## 诊断日志约定

| 级别 | 用途 | 示例 |
|------|------|------|
| `log::warn!` | 关键状态变更（加载/重载/重建） | `[ArmConfigLoader] loaded` |
| `log::error!` | 可恢复错误（脚本语法错等） | `[RhaiMotion] compile error` |
| `eprintln!` | 仅测试调试用，测试通过后删除 | `[RhaiMotion] tick() failed: {}` |

不要用 `println!`，项目要求日志走 `.copper` 结构化或 Bevy tracing。

## 上次踩过的坑（Rhai 脚本引擎）

记录已验证的 Rhai 配置，避免重复踩坑：

```toml
# Cargo.toml
rhai = { version = "1.25.1", features = ["sync", "f32_float"] }
```

```rust
// 引擎初始化
let mut engine = Engine::new();
engine.set_optimization_level(rhai::OptimizationLevel::None);
engine.register_fn("sin", |x: f32| x.sin());
engine.register_fn("cos", |x: f32| x.cos());
engine.register_fn("floor", |x: f32| x.floor());
// ... 其他数学函数

// 脚本路径（绝对路径，不依赖 CWD）
let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap();
let script_path = PathBuf::from(&manifest).join("assets/motion.rhai");

// 常量注入
let mut scope = Scope::new();
scope.push_constant("PI", std::f32::consts::PI);

// 调用函数
let result: rhai::Dynamic = engine.call_fn(&mut scope, &ast, "tick", (tick_f32, dt_f32))?;
let arr = result.into_typed_array::<f32>()?;
```

已知限制：
- `%` 运算符不支持 `f32`，用 `x - floor(x/y)*y` 替代
- `const` 定义在函数作用域内不可见，用 `scope.push_constant` 注入
- 复杂表达式可能触发 "Expression exceeds maximum complexity"，用 `OptimizationLevel::None` 或拆分表达式
