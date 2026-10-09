#!/bin/bash
set -u

usage() {
  echo "usage: $0 [re] [--dry-run]"
  echo "  (default)  bring up the leg and head CAN channels that are not up yet (1 Mbps, txqueuelen 1000)"
  echo "  re         take every leg and head channel down first, then bring it up again"
  echo "  --dry-run  print the commands without running them"
}

RESTART=0
DRY=0
for arg in "$@"; do
  case "$arg" in
    re|--re) RESTART=1 ;;
    --dry-run) DRY=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $arg"; usage; exit 2 ;;
  esac
done

MAP="${ROBONEX_BUS_MAP:-$HOME/.config/robonex/bus_map.json}"
CHANNELS=$(python3 - "$MAP" <<'PY'
import json, os, sys
mapping = {"left_leg": "can0", "right_leg": "can1", "head": "can4"}
path = sys.argv[1]
if os.path.isfile(path):
    with open(path, encoding="utf-8") as handle:
        loaded = json.load(handle)
    mapping.update({group: loaded[group] for group in mapping if group in loaded})
seen = []
for group in ("left_leg", "right_leg", "head"):
    if mapping[group] not in seen:
        seen.append(mapping[group])
print(" ".join(seen))
PY
) || { echo "could not read the bus map $MAP"; exit 1; }

run() {
  if [ "$DRY" = 1 ]; then
    echo "  [dry-run] $*"
  else
    "$@"
  fi
}

echo "bus map : $MAP"
echo "channels: $CHANNELS (left_leg, right_leg, head)"
ADAPTERS=$(lsusb 2>/dev/null | grep -c 1d50:606f)
echo "USB-CAN adapters: $ADAPTERS"

MISSING=0
for c in $CHANNELS; do
  if [ ! -e "/sys/class/net/$c" ]; then
    echo "  $c: no such device (check the USB-CAN hub cable and power)"
    MISSING=1
  fi
done
[ "$MISSING" = 0 ] || exit 1

for c in $CHANNELS; do
  state=$(cat "/sys/class/net/$c/operstate")
  if [ "$RESTART" = 1 ]; then
    run sudo ip link set "$c" down
  elif [ "$state" = "up" ]; then
    echo "  $c: already up, left as is"
    continue
  fi
  run sudo ip link set "$c" up type can bitrate 1000000
  run sudo ip link set "$c" txqueuelen 1000
done

FAIL=0
for c in $CHANNELS; do
  can_state=$(ip -details link show "$c" | grep -o 'can state [A-Z-]*')
  echo "  $c: $(cat "/sys/class/net/$c/operstate"), ${can_state:-no can state}"
  case "$can_state" in
    *ERROR-ACTIVE*) ;;
    *) FAIL=1 ;;
  esac
done
if [ "$FAIL" = 1 ] && [ "$DRY" = 0 ]; then
  echo "not every channel is ERROR-ACTIVE: check that channel's cable, termination and motor power, then run '$0 re'"
  exit 1
fi
