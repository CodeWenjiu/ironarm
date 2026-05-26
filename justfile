# 机械臂项目自动化

build:
    cargo build -p robot_arm

run:
    cargo run -p robot_arm

check:
    cargo check -p robot_arm

dag:
    @command -v cu29-rendercfg >/dev/null 2>&1 || cargo install --locked cu29-runtime --version "0.15.0" --bin cu29-rendercfg
    cu29-rendercfg robot_arm/copperconfig.ron --open
