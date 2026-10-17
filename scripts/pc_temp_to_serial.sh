#!/usr/bin/env bash
# Send the PC cpu temperature and fan speeds to the uno every second
# as "temp fan1 fan2", temp in C and fans in percent of max rpm.
#
# Usage: pc_temp_to_serial.sh [port]
#   port defaults to $PORT_UNO, or ttyACM0 if that is unset.

set -euo pipefail

PORT=${1:-${PORT_UNO:-ttyACM0}}
DEV=/dev/$PORT

TEMP_SENSOR=k10temp
FAN_SENSOR=lenovo_wmi_other
FANS=(1 4)

# hwmon numbers can change between boots, so look them up by name.
find_hwmon() {
    for h in /sys/class/hwmon/hwmon*; do
        if [ "$(cat "$h/name")" = "$1" ]; then
            echo "$h"
        fi
    done
}

fan_percent() {
    if [ -z "$FAN_DIR" ]; then
        echo 0
        return
    fi
    echo $((100 * $(cat "$FAN_DIR/fan$1_input") / $(cat "$FAN_DIR/fan$1_max")))
}

TEMP_DIR=$(find_hwmon "$TEMP_SENSOR")
FAN_DIR=$(find_hwmon "$FAN_SENSOR")

if [ -z "$TEMP_DIR" ]; then
    echo "Error: temperature sensor '$TEMP_SENSOR' not found" >&2
    exit 1
fi

stty -F "$DEV" 9600 raw -echo
exec 3> "$DEV"

# Opening the port resets the uno, keep it open and wait for the bootloader.
sleep 2

while true; do
    temp=$(($(cat "$TEMP_DIR/temp1_input") / 1000))
    fan1=$(fan_percent "${FANS[0]}")
    fan2=$(fan_percent "${FANS[1]}")

    echo "$temp $fan1 $fan2" >&3
    echo "temp: ${temp}C  fan1: $fan1%  fan2: $fan2%"

    sleep 1
done
