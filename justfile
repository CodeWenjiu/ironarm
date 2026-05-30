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

# 运行 TUI
[group('run')]
dev:
    @cargo run -p ironarm_tui

# release 模式运行
[group('run')]
run:
    @cargo run -p ironarm_tui --release

# 仿真（Python）
[group('run')]
sim cmd="":
    @just --justfile ironarm_sim/justfile --working-directory ironarm_sim {{ if cmd == "" { "sim" } else { cmd } }}

# 运行日志阅读器
[group('run')]
logreader cmd="log-stats" log="target/ironarm_tui_log.copper":
    @cargo run -p ironarm_logreader -- {{ log }} {{ cmd }}

# 运行离线回放
[group('run')]
resim log="target/ironarm_tui_log.copper":
    @cargo run -p ironarm_resim -- {{ log }}

[group('check')]
lint:
    @cargo clippy -- -D warnings
    @just sim lint

# 删除 copper crash 文件
crash-clean:
    rm -f copper-crash-*.txt
