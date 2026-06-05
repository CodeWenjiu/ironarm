# Copper 机械臂项目宪法

> Tier 3 Local Law — 本项目所有 agent 会话自动加载。与用户指令冲突时，用户指令优先。

## 零、每次行动前必须自检（违反任一条 = 立即停止并纠正）

- **终端命令？** → 必须用 `nix develop --command` 包裹（详见六.6）。
- **用户报告了问题？** → 必须先精确复现用户描述的命令和场景（详见六.12）。
- **要汇报结论？** → 必须贴出 `cargo test` 或 `headless_render.py` 的通过输出（详见六.7）。

---

## 一、项目性质

基于 Copper（cu29）框架的机械臂控制项目。最终目标是工业级机械臂的运动控制、离线回放和物理仿真。
项目为单 crate，源码在 `ironarm_cli/` 目录。

## 二、架构约束（不可偏离）

1. **关节可配置**：关节数量、参数一律定义在 `copperconfig.ron` 中，不在源码中硬编码。增加关节只改 RON，不改 Rust。
2. **硬件不耦合**：不直接控制 GPIO/PWM/总线。电机驱动由底层驱动板完成，上层只收发 `JointCommand` / `JointState` 消息。
3. **消息即契约**：task 之间的通信完全通过铜消息类型（bincode 序列化），不引入共享状态、不传裸指针、不跨 task 直接调用方法。
4. **类型安全**：所有消息用 `#[derive(Encode, Decode, Reflect)]` 宏。不允许 `serde_json::Value`、`String` 拼路径名等运行时弱类型。
5. **日志隐式**：不写 `println!`、不手写日志行。运行时行为通过 `.copper` 结构化日志自动捕获。

## 三、代码规范

1. **命名**：crate 命名 以 `ironarm_` 前缀。比如命名 `ironarm_cli`（主程序）。
2. **目录**：源码在 `ironarm_cli/src/`，task 模块在 `src/tasks/`，消息类型在 `src/messages.rs`。新 package 在根目录下一级目录（如 `ironarm_cli/`）。
3. **精简优先**：新功能从最小可用形态起步，跑通后再加。不在第一版塞 resim、bevymon、日志导出等非核心模块。
4. **复用不复制**：如果某个 task 的逻辑可以同时用于多个关节，用 `ComponentConfig` 参数化，不复制多份代码。
5. **Rust edition 2024**：和 Copper 主仓库保持一致。
6. **参考代码**：`references/` 目录存放 `cu_example_app` 和 `cu_rp_balancebot`，仅作参考，不参与编译。

## 四、不允许的操作

- **不对 git 进行任何写操作**（`commit`、`push`、`tag`、`branch` 等一律禁止）。允许只读操作：`status`、`diff`、`log`、`show`、`blame`。
- 不在 task 的 `process()` 中做阻塞 I/O
- 不直接引入硬件 HAL crate（如 `rppal`、`stm32h7xx-hal`）到 task 层——硬件通信通过 Copper bridge 隔离
- 不修改 `copperconfig.ron` 的顶层结构——`tasks` 和 `cnx` 数组的唯一来源是此文件
- 不在 `main.rs` 中写业务逻辑——入口只负责创建 logger、构建运行时、启动主循环

## 五、依赖版本对齐

- `cu29` 及 `cu-*` 系列统一从 crates.io 拉取最新 stable release，不使用 git 依赖
- 外部 crate 版本号与 `copper-project/extra-examples` master 分支保持一致

## 六、Agent 行为规范

1. **先查文档再动手。** 不熟悉的工具/语法（如 justfile `[group]`、bevy API）必须先 query-docs 或读 reference 确认，禁止凭「印象」或「推测」写代码。
2. **最小化验证，不一次改全文件。** 先写最小可运行 case，验证通过后再扩展。
3. **认真读完约束再执行。** 文档说了 `[group]` 只作用于下一个 recipe，就必须严格遵守，不是「好像可以」「试试看」。
4. **所有终端命令必须在 nix-shell 中运行。** 本项目通过 `flake.nix` 管理开发环境。运行命令时使用 `nix develop --command` 包裹，或确保已在 nix-shell 中。
5. **改完必须自测再汇报。** 每次代码修改完成后必须跑 `cargo check` + `cargo test`。涉及 Sim / GUI 的改动还需跑 `nix develop --command timeout 5 just run sim`（exit code 124=正常 timeout，1=崩溃）+ `headless_render.py`（误差 <50mm）。必须贴出通过输出，禁止「应该没问题」。
6. **justfile 内不得调用 nix。** justfile recipe 中禁止出现任何 nix 命令。需要 nix 环境由调用方通过 `nix develop --command just ...` 提供。
7. **GUI/渲染改动必须 headless 闭环。** 涉及 MuJoCo 渲染、Python view 层的改动，必须通过 `nix develop --command timeout 5 just run sim` 验证不崩溃。视觉正确性无法 headless 验证的，将关键数据写入 `/tmp/ironarm_diag.txt`，用 `grep` 检查。禁止依赖用户观察来验证渲染结果。
8. **调试失败上限。** 同一子问题连续失败 3 次后，必须停止猜测。转而：a) 查文档/源码找到正确方式，或 b) 向用户报告根因和已知事实，询问方向。禁止第四、第五次盲试。
9. **精确复现优先于猜测。** 用户报告"命令 X 不行，命令 Y 可以"时，必须先直接跑 X 和 Y 各一次，对比输出差异，定位到具体代码/配置行。禁止用替代路径绕过。用户没有骗你。
10. **先写复现测试再修 bug。** 定位到 bug 根因后，先写最小单元测试复现该 bug（跑一遍确认 FAIL），再修改产品代码，最后确认测试 PASS。Rust 侧写 `#[test]`，Python 侧用 `headless_render.py` 闭环。

## 七、Sim / PyO3 专项约束

1. **`copperconfig.ron` 的位置。** `#[copper_runtime(config = "copperconfig.ron")]` 的路径在编译期相对于 **应用该宏的 crate 的 CARGO_MANIFEST_DIR**（当前为 `ironarm_std/`）解析，而非运行时 CWD。不允许在其他位置放同名文件——那不会被读取，只会制造混淆。
2. **`uv run` 的 sync 陷阱。** `uv run python` 在运行前会自动检查环境是否和 `uv.lock` 一致，不一致就 sync——这会覆盖 `maturin develop` 刚编译好的 `.so`。justfile 中运行 Python 必须用 `uv run --no-sync python`。
3. **`maturin develop` 的脏缓存。** 改完 Rust 代码后，`maturin develop` 可能不覆盖 `.venv` 中旧 `.so`。justfile 的 `run` recipe 必须在 `maturin develop` 前执行 `rm -rf .venv/lib/python*/site-packages/ironarm_sim*`。
4. **PyO3 panic 变异常。** Rust 侧的 `panic!()` 在 PyO3 函数中会被转换为 `pyo3_runtime.PanicException`，不会让 Python 进程 crash。不能通过「进程是否崩溃」来判断 Rust 代码是否执行到。验证执行路径请用文件写入（`std::fs::write`）或 `headless_render.py`。
5. **Sim 和 TUI 必须分开验证。** 两者共用 `ironarm_std`，但入口不同（`ironarm_sim/rust_core/src/lib.rs` vs `ironarm_tui/src/main.rs`），线程模型不同（Sim 在独立线程跑 Copper）。问题可能只在一边出现，必须两边都测。

## 八、Rhai 专项约束

1. **类型转换用 `as_float()` 不用 `try_cast::<f32>()`。** Rhai 内部数值为 `f64`，`Dynamic::try_cast::<T>()` 做的是精确类型匹配（`Any::downcast_ref`），不做窄化转换。提取数值必须用 `v.as_float().ok().map(|f| f as f32)`。
2. **`import` 的模块会被 Engine 缓存。** `Engine::compile()` 会缓存已 import 的模块 AST，修改模块文件后重编译主脚本不会重新读取模块。热重载必须 `Engine::new()` 创建新实例来清空缓存。
3. **`const` 在导入模块中不可见。** 模块顶层 `const` 在 `import` 后，被调用函数内无法访问。改用函数内 `let`。`PI` 同理——直接写 `3.141592653589793` 字面量。
4. **Rhai 数值字面量默认为 `f64`。** 从 Rust 传参数进 Rhai 函数时，传 `f64` 避免 `f32 * f64` 类型不匹配。
