#!/usr/bin/env bash
# Compile a native avr-gcc project (no .ino) and upload it.
#
# Usage: uno_r3_native_compile_and_upload.sh <project_dir> [port]
#   port defaults to $PORT_UNO, or ttyACM0 if that is unset.

set -euo pipefail

TOOLS=$HOME/.arduino15/packages/arduino/tools
GCC_BIN=$(ls -d "$TOOLS"/avr-gcc/*/bin | tail -1)
AVRDUDE_DIR=$(ls -d "$TOOLS"/avrdude/* | tail -1)

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

BUILD=$PROJECT/build
mkdir -p "$BUILD"

"$GCC_BIN/avr-gcc" -mmcu=atmega328p -DF_CPU=16000000UL -Os -std=c11 -Wall -Wextra \
    -o "$BUILD/main.elf" "$PROJECT"/*.c
"$GCC_BIN/avr-objcopy" -O ihex -R .eeprom "$BUILD/main.elf" "$BUILD/main.hex"
"$GCC_BIN/avr-size" --mcu=atmega328p -C "$BUILD/main.elf"
"$AVRDUDE_DIR/bin/avrdude" -C "$AVRDUDE_DIR/etc/avrdude.conf" \
    -p m328p -c arduino -P "/dev/$PORT" -b 115200 -U "flash:w:$BUILD/main.hex:i"
