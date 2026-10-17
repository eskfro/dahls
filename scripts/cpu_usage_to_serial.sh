#!/usr/bin/env bash
# Send the PC cpu usage in percent to the uno every second.
#
# Usage: cpu_usage_to_serial.sh [port]
#   port defaults to $PORT_UNO, or ttyACM0 if that is unset.

set -euo pipefail

PORT=${1:-${PORT_UNO:-ttyACM0}}
DEV=/dev/$PORT

# Prints "busy idle" cpu time since boot from /proc/stat.
read_cpu() {
    local cpu user nice system idle iowait irq softirq steal rest
    read -r cpu user nice system idle iowait irq softirq steal rest < /proc/stat
    echo $((user + nice + system + irq + softirq + steal)) $((idle + iowait))
}

stty -F "$DEV" 9600 raw -echo
exec 3> "$DEV"

# Opening the port resets the uno, keep it open and wait for the bootloader.
sleep 2

read -r busy idle <<< "$(read_cpu)"

while true; do
    sleep 1
    read -r busy_now idle_now <<< "$(read_cpu)"

    d_busy=$((busy_now - busy))
    d_total=$((d_busy + idle_now - idle))
    usage=$((100 * d_busy / d_total))

    echo "$usage" >&3
    echo "cpu: $usage%"

    busy=$busy_now
    idle=$idle_now
done
