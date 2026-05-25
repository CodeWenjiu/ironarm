# Workspace automation helpers.

# Render the execution DAG from the app config.
dag:
  #!/usr/bin/env bash
  set -euo pipefail
  APP_DIR="${APP_DIR:-cu_example_app}"
  
  renderer="$(command -v cu29-rendercfg || true)"
  if [[ -z "$renderer" ]]; then
    echo "cu29-rendercfg not found on PATH; installing it now..." >&2
    
    cargo install --locked cu29-runtime --version "0.15.0" --bin cu29-rendercfg
    
    renderer="$(command -v cu29-rendercfg || true)"
    if [[ -z "$renderer" ]]; then
      renderer="${CARGO_HOME:-$HOME/.cargo}/bin/cu29-rendercfg"
    fi
  fi
  [[ -x "$renderer" ]] || { echo "Failed to find cu29-rendercfg after installation." >&2; exit 1; }
  "$renderer" apps/"${APP_DIR}"/copperconfig.ron --open
  

# Compatibility alias for older template docs.
rcfg: dag

# Extract the structured log via the log reader.
log:
  #!/usr/bin/env bash
  set -euo pipefail
  APP_DIR="${APP_DIR:-cu_example_app}"
  APP_NAME="${APP_NAME:-${APP_DIR}}"
  RUST_BACKTRACE=1 cargo run -p "${APP_NAME}" --features=logreader --bin "${APP_NAME}-logreader" \
    apps/"${APP_DIR}"/logs/"${APP_NAME}".copper extract-text-log target/debug/cu29_log_index

# Extract CopperLists from the log output.
cl:
  #!/usr/bin/env bash
  set -euo pipefail
  APP_DIR="${APP_DIR:-cu_example_app}"
  APP_NAME="${APP_NAME:-${APP_DIR}}"
  RUST_BACKTRACE=1 cargo run -p "${APP_NAME}" --features=logreader --bin "${APP_NAME}-logreader" \
    apps/"${APP_DIR}"/logs/"${APP_NAME}".copper extract-copperlists

resim:
  #!/usr/bin/env bash
  set -euo pipefail
  APP_DIR="${APP_DIR:-cu_example_app}"
  APP_NAME="${APP_NAME:-${APP_DIR}}"
  RUST_BACKTRACE=1 cargo run -p "${APP_NAME}" --features=sim-debug --bin "${APP_NAME}-resim" -- \
    --log-base "apps/${APP_DIR}/logs/${APP_NAME}.copper"

resim-debug debug_base="":
  #!/usr/bin/env bash
  set -euo pipefail
  APP_DIR="${APP_DIR:-cu_example_app}"
  APP_NAME="${APP_NAME:-${APP_DIR}}"
  DEBUG_BASE="{{debug_base}}"
  if [[ -z "$DEBUG_BASE" ]]; then
    DEBUG_BASE="copper/apps/${APP_NAME}/debug/v1"
  fi
  RUST_BACKTRACE=1 cargo run -p "${APP_NAME}" --features=sim-debug --bin "${APP_NAME}-resim" -- \
    --debug-base "$DEBUG_BASE" \
    --log-base "apps/${APP_DIR}/logs/${APP_NAME}.copper" \
    --replay-log-base "apps/${APP_DIR}/logs/${APP_NAME}_resim.copper"
