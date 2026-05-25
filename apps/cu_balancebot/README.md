# BalanceBot

A full Copper demo robot with:

- a physical robot implementation
- a simulation implementation
- a resimulation demoing the deterministic replay
- a log export

## Quick start

To run the simulation:

```bash
cd apps/cu_balancebot
cargo run
```

To run the simulation with the monitor embedded in Bevy:

```bash
just bevy
```

To run the simulation in the browser:

```bash
just web
```

To build a static browser bundle:

```bash
just web-dist
```

To run the resimulation (need at least one log in logs/):

```bash
cargo run --no-default-features --features sim-debug --bin balancebot-resim --release
```

To start the replay-backed remote debug server:

```bash
just resim-debug
```

## Run on the real robot

Cross compile for Arm:

```bash
cargo build --target armv7-unknown-linux-musleabihf --release --no-default-features
```

Save your log string index:

```bash
cp -rv ../../target/armv7-unknown-linux-musleabihf/release/cu29_log_index .
```

Deploy on the target:

```bash
scp ../../target/armv7-unknown-linux-musleabihf/release/balancebot copperconfig.ron copper7:copper/
```

## Export logs

```bash
cargo run --bin balancebot-logreader --release
```

## Justfile commands

- `just bevy` — run the split-view Bevy sim with cu_bevymon.
- `just web` — serve the split-view wasm demo with Trunk.
- `just web-dist` — build a deployable static wasm bundle into dist/balancebot/.
- `just balancebot-dump-text-logs` — extract human-readable logs.
- `just balancebot-fsck` — integrity check of logs/balance.copper.
- `just balancebot-set-pwm-permissions` — fix PWM sysfs permissions on the target.
- `just dag-logstats` — generate logstats and open an annotated DAG SVG.
