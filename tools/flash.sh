#!/bin/sh
# Build and flash the probe.
#
#   tools/flash.sh /dev/ttyACM0
#
# Board options are not negotiable here: this ESP32-S3 carries 16MB of flash and 8MB of octal
# PSRAM, and getting PSRAM=opi wrong produces a board that flashes and then misbehaves rather
# than one that fails loudly. Same reasoning as keti-reconfig's flash.sh -- hence a script
# instead of a line in the README for someone to mistype.
set -eu

PORT=${1:-}
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
FQBN='esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc'

if [ -z "$PORT" ]; then
  echo "ports that look like an ESP32-S3:"
  for p in /dev/ttyACM*; do
    [ -e "$p" ] || continue
    model=$(udevadm info -q property -n "$p" 2>/dev/null | grep -E '^ID_MODEL=' || true)
    id=$(udevadm info -q property -n "$p" 2>/dev/null | grep -E '^ID_SERIAL_SHORT=' || true)
    case "$model" in *JTAG*) echo "  $p  ${id#ID_SERIAL_SHORT=}" ;; esac
  done
  echo "re-run with the port you want"
  exit 1
fi

arduino-cli compile --fqbn "$FQBN" "$ROOT/firmware/lidar_probe"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$ROOT/firmware/lidar_probe"
echo "done. the board prints its address over serial; open http://<that>/"
