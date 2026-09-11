#!/usr/bin/env bash
# Build Rover firmware for the DAKEFPV H743 flight controller.
# Run from anywhere; the script cds to the ArduPilot tree root.
#
# Usage:
#   Tools/scripts/build_rover_dakefpvh743.sh
#   BOARD=DAKEFPVH743Pro Tools/scripts/build_rover_dakefpvh743.sh
#
# AP_FLAKE8_CLEAN

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

BOARD="${BOARD:-DAKEFPVH743}"
ARM_BIN="/opt/gcc-arm-none-eabi-10-2020-q4-major/bin"

if [[ -d "$ARM_BIN" ]]; then
    export PATH="$ARM_BIN:$PATH"
fi

if [[ -f "$HOME/venv-ardupilot/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/venv-ardupilot/bin/activate"
fi

if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    echo "arm-none-eabi-gcc not found. Install the ArduPilot STM32 toolchain:" >&2
    echo "  Linux: Tools/environment_install/install-prereqs-ubuntu.sh -y" >&2
    echo "  macOS: Tools/environment_install/install-prereqs-mac.sh -y" >&2
    exit 1
fi

./waf configure --board "$BOARD"
./waf rover

echo
echo "Firmware:"
ls -lh "build/${BOARD}/bin/ardurover."{apj,bin,hex} 2>/dev/null || ls -lh "build/${BOARD}/bin"/ardurover*
echo
echo "Flash build/${BOARD}/bin/ardurover.apj with Mission Planner / QGroundControl"
echo "(Setup -> Install Firmware -> Load custom firmware)."
