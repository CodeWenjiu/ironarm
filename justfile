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

# 类型检查（不产二进制）
[group('build')]
check:
    @cargo check

# 开发模式运行（默认 cli，传 sim 切到仿真）
[group('run')]
dev pkg="cli":
    @cargo run -p ironarm_{{pkg}}

# release 模式运行
[group('run')]
run pkg="cli":
    @cargo run -p ironarm_{{pkg}} --release

# 渲染 DAG 拓扑图
dag:
    @command -v cu29-rendercfg >/dev/null 2>&1 || cargo install --locked cu29-runtime --version "0.15.0" --bin cu29-rendercfg
    cu29-rendercfg ironarm_core/copperconfig.ron --open
