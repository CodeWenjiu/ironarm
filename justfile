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

# 开发模式运行（默认 tui，传 sim 切到仿真）
[group('run')]
dev pkg="tui":
    @cargo run -p ironarm_{{ pkg }} {{ if pkg == "sim" { "--features bevy/dynamic_linking" } else { "" } }}

# release 模式运行
[group('run')]
run pkg="tui":
    @cargo run -p ironarm_{{ pkg }} --release

# 渲染 DAG 拓扑图
dag:
    @command -v cu29-rendercfg >/dev/null 2>&1 || cargo install --locked cu29-runtime --version "0.15.0" --bin cu29-rendercfg
    cu29-rendercfg ironarm_std/copperconfig.ron --open

# 删除根目录 copper crash 文件
crash-clean:
    rm -f copper-crash-*.txt

# 运行日志阅读器
logreader cmd="log-stats" log="target/ironarm_tui_log.copper":
    @cargo run -p ironarm_logreader -- {{ log }} {{ cmd }}

# 运行离线回放
resim log="target/ironarm_tui_log.copper":
    @cargo run -p ironarm_resim -- {{ log }}
