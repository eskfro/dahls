#!/usr/bin/env bash
# Take in an arduino project folder and compile and upload it.
# Takes in the arduino project location as an input.
# Also need to have a port as a variable.
#
# Usage: uno_r3_compile_and_upload.sh <project_dir> [port]
#   port defaults to $PORT_UNO, or ttyACM0 if that is unset.
set -euo pipefail

FQBN=arduino:avr:uno

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    echo "Usage: $0 <project_dir> [port]" >&2
    exit 1
fi

PROJECT=$1
PORT=${2:-${PORT_UNO:-ttyACM0}}

if [ ! -d "$PROJECT" ]; then
    echo "Error: project folder '$PROJECT' not found" >&2
    exit 1
fi

arduino-cli compile --fqbn "$FQBN" "$PROJECT"
arduino-cli upload -p "/dev/$PORT" --fqbn "$FQBN" "$PROJECT"
