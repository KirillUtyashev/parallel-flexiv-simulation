#!/usr/bin/env bash

set -Eeuo pipefail

if [[ ${TERM:-dumb} == dumb ]]; then
    export TERM=xterm
fi

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ISAAC_WS=$(cd -- "$SCRIPT_DIR/../../.." && pwd)
BRIDGE_DIR="$ISAAC_WS/standalone_examples/api/isaacsim.robot.manipulators/flexiv"
FLEET_DIR="$ISAAC_WS/scripts/controllers"
CONFIG=${FLEXIV_ISAAC_CONFIG:-"$ISAAC_WS/configs/16-environments.yaml"}

if [[ $# -gt 0 && "$1" != --* ]]; then
    CONFIG=$1
    shift
fi

if [[ ! -f "$CONFIG" ]]; then
    printf 'Isaac GPU async fleet config not found: %s\n' "$CONFIG" >&2
    exit 1
fi

# One DDS UserNode per simulated robot consumes file descriptors in the Isaac
# process.  The default interactive-shell limit (often 1024) is insufficient
# for the 512-robot checkpoint, although the host hard limit is much higher.
REQUIRED_NOFILE=${FLEXIV_ISAAC_NOFILE:-65536}
CURRENT_NOFILE=$(ulimit -Sn)
if [[ "$CURRENT_NOFILE" != unlimited ]] && (( CURRENT_NOFILE < REQUIRED_NOFILE )); then
    ulimit -Sn "$REQUIRED_NOFILE"
fi
printf 'Isaac open-file soft limit: %s\n' "$(ulimit -Sn)"

if [[ -n "${FLEXIV_FLEET_MANIFEST:-}" ]]; then
    MANIFEST=$(python3 -c \
        'import pathlib, sys; print(pathlib.Path(sys.argv[1]).expanduser().resolve())' \
        "$FLEXIV_FLEET_MANIFEST")
else
    MANIFEST=$(mktemp /tmp/flexiv-isaac-manifest.XXXXXX.json)
fi

python3 "$FLEET_DIR/generate_robot_run_manifest.py" \
    --isaac-config "$CONFIG" \
    --output "$MANIFEST"

ASYNC_MODULE_DIR=$(
    "$FLEET_DIR/build_async_fleet_module.sh" \
        "${ISAAC_PYTHON:-$HOME/miniforge3/envs/isaaclab/bin/python}"
)

printf 'Isaac GPU async config: %s\n' "$CONFIG"
printf 'Fleet manifest: %s\n' "$MANIFEST"
printf 'Native async module: %s\n' "$ASYNC_MODULE_DIR"
printf 'Controller command (second terminal):\n'
printf '  cd %q && ./scripts/controllers/run_generated_robot_fleet.sh --manifest %q --physics isaac\n' \
    "$ISAAC_WS" "$MANIFEST"

if [[ -n "${PYTHONPATH:-}" ]]; then
    ASYNC_PYTHONPATH="$ASYNC_MODULE_DIR:$PYTHONPATH"
else
    ASYNC_PYTHONPATH="$ASYNC_MODULE_DIR"
fi

exec env PYTHONPATH="$ASYNC_PYTHONPATH" \
    "${CONDA_EXE:-$HOME/miniforge3/bin/conda}" run --no-capture-output -n isaaclab \
    "${ISAACLAB_ROOT:-$HOME/IsaacLab}/isaaclab.sh" -p \
    "$BRIDGE_DIR/flexiv_isaac_lab_gpu_async_fleet_bridge_app.py" \
    --config "$CONFIG" \
    --manifest "$MANIFEST" \
    "$@"
