__default:
    @just --list

# 编译整个 workspace
[group('build')]
build:
    @cargo build

# 清理构建产物
[group('build')]
clean:
    @cargo clean

# 类型检查
[group('build')]
check:
    @cargo check

# 开发模式运行（默认 tui，可选 sim）
[group('run')]
dev pkg="tui":
    @just _{{ pkg }}-dev

# release 模式运行（默认 tui，可选 sim）
[group('run')]
run pkg="tui":
    @just _{{ pkg }}-run

# 内部：TUI debug
_tui-dev:
    @cargo run -p ironarm_tui

# 内部：TUI release
_tui-run:
    @cargo run -p ironarm_tui --release

# 内部：Sim debug
_sim-dev:
    @just --justfile ironarm_sim/justfile --working-directory ironarm_sim build run

# 内部：Sim release
_sim-run:
    @just --justfile ironarm_sim/justfile --working-directory ironarm_sim build-release run

# Rust 代码检查
_lint-rs:
    @cargo clippy -- -D warnings

# Python 代码检查
_lint-py:
    @just --justfile ironarm_sim/justfile --working-directory ironarm_sim lint

# 全部检查
[group('check')]
lint: _lint-rs _lint-py

# 运行日志阅读器
logreader cmd="log-stats" log="target/ironarm_tui_log.copper":
    @cargo run -p ironarm_logreader -- {{ log }} {{ cmd }}

# 运行离线回放
resim log="target/ironarm_tui_log.copper":
    @cargo run -p ironarm_resim -- {{ log }}

# 删除 copper crash 文件
crash-clean:
    find . -name "copper-crash-*.txt" -delete
