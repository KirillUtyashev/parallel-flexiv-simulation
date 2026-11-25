#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ISAAC_WS=$(cd -- "$SCRIPT_DIR/../.." && pwd)
NATIVE_ROOT="$ISAAC_WS/native/flexiv_async_fleet"
VENDOR_ROOT="$ISAAC_WS/vendor/flexiv_sim_plugin"
PYTHON=${FLEXIV_ASYNC_PYTHON:-$HOME/miniforge3/envs/isaaclab/bin/python}

if [[ $# -gt 0 ]]; then
    PYTHON=$1
fi

if [[ ! -x "$PYTHON" ]]; then
    printf 'Python interpreter is not executable: %s\n' "$PYTHON" >&2
    exit 1
fi

PYTHON_TAG=$($PYTHON -c 'import sys; print(f"py{sys.version_info.major}{sys.version_info.minor}")')
PYBIND11_DIR=$($PYTHON -m pybind11 --cmakedir)
BUILD_DIR="$ISAAC_WS/build/async_fleet_$PYTHON_TAG"
PRIVATE_SIM_PLUGIN_ARCHIVE="$BUILD_DIR/libflexiv_sim_plugin.shared_participant.a"
PRIVATE_DUMMY_PEER_ARCHIVE="$BUILD_DIR/libflexiv_sim_plugin.dds_dummy_peer.a"

"$PYTHON" "$NATIVE_ROOT/tools/prepare_async_fleet_archive.py" \
    --source "$VENDOR_ROOT/lib/libflexiv_sim_plugin.x86_64-linux-gnu.a" \
    --output "$PRIVATE_SIM_PLUGIN_ARCHIVE" >&2

"$PYTHON" "$NATIVE_ROOT/tools/prepare_dds_dummy_peer_archive.py" \
    --source "$VENDOR_ROOT/lib/libflexiv_sim_plugin.x86_64-linux-gnu.a" \
    --output "$PRIVATE_DUMMY_PEER_ARCHIVE" \
    --mutation-tries 4096 >&2

cmake -S "$NATIVE_ROOT" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE="$PYTHON" \
    -Dpybind11_DIR="$PYBIND11_DIR" \
    -DFLEXIV_SIM_PLUGIN_ARCHIVE="$PRIVATE_SIM_PLUGIN_ARCHIVE" \
    -DFLEXIV_DDS_DUMMY_PEER_ARCHIVE="$PRIVATE_DUMMY_PEER_ARCHIVE" \
    -DFLEXIV_ASYNC_DDS_SHARED_PARTICIPANT=ON >&2
cmake --build "$BUILD_DIR" --parallel >&2

MODULE_PATH=$(find "$BUILD_DIR" -maxdepth 1 -type f -name '_flexiv_async_fleet*.so' -print -quit)
if [[ -z "$MODULE_PATH" ]]; then
    printf 'Built async fleet module was not found in %s\n' "$BUILD_DIR" >&2
    exit 1
fi

printf '%s\n' "$BUILD_DIR"
