# Copper 机械臂项目宪法

> Tier 3 Local Law — 本项目所有 agent 会话自动加载。与用户指令冲突时，用户指令优先。

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

## 六、Agent 行为规范（给自己的约束）

1. **先查文档再动手。** 不熟悉的工具/语法（如 justfile `[group]`、bevy API）必须先 query-docs 或读 reference 确认，禁止凭「印象」或「推测」写代码。
2. **write_file 不可靠，用 `cat << 'EOF'` 替代。** `write_file` 工具写入的文件可能出现编码问题（如 BOM、不可见字符）导致解析失败。写配置文件（justfile、Cargo.toml、RON 等）一律用终端 heredoc。
3. **最小化验证，不一次改全文件。** 先写最小可运行 case（单 recipe、单 struct、单 module），验证通过后再扩展到完整版本。不要反复猜错、改全文件、再看报错。
4. **认真读完约束再执行。** 文档说了 `[group]` 只作用于下一个 recipe、中间不能有注释，就必须严格遵守，不是「好像可以」「试试看」。
5. **验证即交付。** 写完配置必须 `just --list` / `cargo check` 跑通再汇报结果，不允许「应该可以了」就交差。
6. **所有终端命令必须在 nix-shell 中运行。** 本项目通过 `flake.nix` 管理开发环境（工具链、X11 库、GPU 驱动等）。直接在当前 shell 运行 `cargo` 或程序二进制可能出现缺少依赖的错误。运行命令时使用 `nix develop` 或确保已在 nix-shell 中。
7. **改完必须自测再汇报。** 每次代码修改完成后，至少运行 `cargo check`（Rust 侧）/ `uv run ruff check . && uv run mypy .`（Python 侧）以及 Python 导入测试，验证无编译错误和类型错误。涉及 GUI / Qt / OpenGL 的改动还需在无头环境下跑最小初始化测试确认不会 crash。不允许「应该可以了」就交差，必须贴出测试通过的输出。

8. **justfile 内不得调用 nix。** 项目使用 nix 管理依赖，但应当是可选的——满足环境依赖的开发者不使用 nix 也应能正常运行。justfile recipe 中禁止出现 \、\、\ 等 nix 命令。如果需要 nix 环境，应由调用方通过 \ 提供，而非在被调用方内部绑定。

8. **justfile 内不得调用 nix。** 项目使用 nix 管理依赖，但应当是可选的——满足环境依赖的开发者不使用 nix 也应能正常运行。justfile recipe 中禁止出现任何 nix 命令。如果需要 nix 环境，应由调用方通过 "nix develop --command just ..." 提供，而非在被调用方内部绑定。
