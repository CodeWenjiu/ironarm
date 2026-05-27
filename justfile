__default:
    @just --list

build:
    @cargo build

clean:
    @cargo clean

dev:
    @cargo run

run:
    @cargo run --release

check:
    @cargo check

dag:
    @command -v cu29-rendercfg >/dev/null 2>&1 || cargo install --locked cu29-runtime --version "0.15.0" --bin cu29-rendercfg
    cu29-rendercfg ironarm_cli/copperconfig.ron --open
