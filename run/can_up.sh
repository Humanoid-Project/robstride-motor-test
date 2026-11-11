#!/bin/bash
set -u

usage() {
  echo "usage: $0 [re] [--dry-run] [--strict | --all] [channel ...]"
  echo "  (default)  bring up every channel of the bus map that exists on this machine (1 Mbps, txqueuelen 1000);"
  echo "             groups: left_leg, right_leg, head and every group named in the bus map file"
  echo "  channel    bring up only these channels (e.g. can0); a named channel that does not exist is an error"
  echo "  --all      also the groups not wired today (left_arm, right_arm)"
  echo "  --strict   every selected channel must exist (assembled robot); otherwise exit 1 before any change"
  echo "  re         take each selected channel down first, then bring it up again"
  echo "  --dry-run  print the commands without running them"
}

RESTART=0
DRY=0
STRICT=0
ALL=0
EXPLICIT=()
for arg in "$@"; do
  case "$arg" in
    re|--re) RESTART=1 ;;
    --dry-run) DRY=1 ;;
    --strict) STRICT=1 ;;
    --all) ALL=1 ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "unknown option: $arg"; usage; exit 2 ;;
    *)
      if [[ ! "$arg" =~ ^[A-Za-z0-9_.-]{1,15}$ ]]; then
        echo "not a valid interface name: $arg"; exit 2
      fi
      EXPLICIT+=("$arg") ;;
  esac
done
if [ "${#EXPLICIT[@]}" -gt 0 ] && [ "$ALL" = 1 ]; then
  echo "--all selects channels from the bus map; do not combine it with explicit channels"; exit 2
fi

SYSFS="${CAN_UP_SYSFS:-/sys/class/net}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
if [ -n "${CAN_UP_PYTHON:-}" ]; then
  PY="$CAN_UP_PYTHON"
elif [ -x "$REPO/.venv/bin/python" ]; then
  PY="$REPO/.venv/bin/python"
else
  PY=python3
fi

MAP_OUT=$("$PY" - "$ALL" <<'PY'
import importlib.util
import json
import os
import sys
from pathlib import Path

include_all = sys.argv[1] == "1"
buses = None
source = "built-in copy in can_up.sh (robonex-common not importable)"
try:
    spec = importlib.util.find_spec("robonex_common")
except (ImportError, ValueError):
    spec = None
for folder in (spec.submodule_search_locations or []) if spec else []:
    candidate = Path(folder) / "buses.py"
    if candidate.is_file():
        module_spec = importlib.util.spec_from_file_location("robonex_common_buses", candidate)
        buses = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(buses)
        source = f"robonex-common {candidate}"
        break

if buses is not None:
    groups, path, load = buses.GROUPS, buses.bus_map_path(), buses.bus_map
else:
    groups = ("left_leg", "right_leg", "left_arm", "right_arm", "head")
    defaults = {"left_leg": "can0", "right_leg": "can1", "left_arm": "can2", "right_arm": "can3", "head": "can4"}
    configured = os.environ.get("ROBONEX_BUS_MAP")
    path = Path(configured).expanduser() if configured else Path.home() / ".config" / "robonex" / "bus_map.json"

    def load():
        mapping = dict(defaults)
        if path.is_file():
            with open(path, "r", encoding="utf-8") as handle:
                loaded = json.load(handle)
            unknown = set(loaded) - set(groups)
            if unknown:
                raise ValueError(f"{path}: unknown motor groups {sorted(unknown)}; known: {groups}")
            for group, channel in loaded.items():
                if not isinstance(channel, str) or not channel:
                    raise ValueError(f"{path}: channel for {group} must be a non-empty string")
                mapping[group] = channel
        return mapping

try:
    mapping = load()
    named = []
    if path.is_file():
        with open(path, "r", encoding="utf-8") as handle:
            named = list(json.load(handle))
except Exception as error:
    print(f"bad bus map {path}: {type(error).__name__}: {error}", file=sys.stderr)
    sys.exit(1)

selected = [g for g in groups if include_all or g in ("left_leg", "right_leg", "head") or g in named]
print(f"SOURCE\t{source}")
print(f"FILE\t{path}" + ("" if path.is_file() else " (not found, defaults only)"))
order, served = [], {}
for group in groups:
    channel = mapping[group]
    if channel not in served:
        order.append(channel)
        served[channel] = []
    served[channel].append(group + ("" if group in selected else "*"))
for channel in order:
    print(f"CH\t{channel}\t{','.join(served[channel])}\t{int(any(g in selected for g in groups if mapping[g] == channel))}")
PY
) || { echo "could not read the bus map (python: $PY)"; exit 1; }

run() {
  if [ "$DRY" = 1 ]; then
    echo "  [dry-run] $*"
  else
    "$@"
  fi
}

declare -A GROUPS_OF=()
MAPPED=()
CANDIDATES=()
while IFS=$'\t' read -r kind a b c; do
  case "$kind" in
    SOURCE) echo "default map: $a" ;;
    FILE) echo "bus map    : $a" ;;
    CH)
      GROUPS_OF[$a]="$b"
      MAPPED+=("$a")
      [ "$c" = 1 ] && CANDIDATES+=("$a") ;;
  esac
done <<< "$MAP_OUT"

echo "channel map (* = group not selected):"
for c in "${MAPPED[@]}"; do
  echo "  $c: ${GROUPS_OF[$c]}"
done
ADAPTERS=$(lsusb 2>/dev/null | grep -c 1d50:606f)
echo "USB-CAN adapters (1d50:606f): $ADAPTERS"

is_can() {
  [ -e "$SYSFS/$1" ] || return 1
  [ ! -r "$SYSFS/$1/type" ] || [ "$(cat "$SYSFS/$1/type")" = 280 ]
}

CHANNELS=()
if [ "${#EXPLICIT[@]}" -gt 0 ]; then
  BAD=0
  for c in "${EXPLICIT[@]}"; do
    if [ ! -e "$SYSFS/$c" ]; then
      echo "  $c: no such device"; BAD=1
    elif ! is_can "$c"; then
      echo "  $c: not a CAN interface"; BAD=1
    else
      case " ${CHANNELS[*]-} " in *" $c "*) ;; *) CHANNELS+=("$c") ;; esac
    fi
  done
  [ "$BAD" = 0 ] || exit 1
else
  MISSING=0
  for c in "${CANDIDATES[@]}"; do
    if is_can "$c"; then
      CHANNELS+=("$c")
    elif [ -e "$SYSFS/$c" ]; then
      echo "  $c: not a CAN interface, skipped (${GROUPS_OF[$c]})"; MISSING=1
    else
      echo "  $c: no such device, skipped (${GROUPS_OF[$c]})"; MISSING=1
    fi
  done
  if [ "$STRICT" = 1 ] && [ "$MISSING" = 1 ]; then
    echo "--strict: not every mapped channel exists (check the USB-CAN hub cable and power); nothing changed"
    exit 1
  fi
  if [ "${#CHANNELS[@]}" = 0 ]; then
    echo "no mapped CAN channel exists (check the USB-CAN adapter, 'sudo modprobe gs_usb'); nothing changed"
    exit 1
  fi
fi
echo "channels: ${CHANNELS[*]}"

for c in "${CHANNELS[@]}"; do
  state=$(cat "$SYSFS/$c/operstate" 2>/dev/null)
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
for c in "${CHANNELS[@]}"; do
  can_state=$(ip -details link show "$c" 2>/dev/null | grep -o 'can state [A-Z-]*')
  echo "  $c: $(cat "$SYSFS/$c/operstate" 2>/dev/null), ${can_state:-no can state} (${GROUPS_OF[$c]:-not in the bus map})"
  case "$can_state" in
    *ERROR-ACTIVE*) ;;
    *) FAIL=1 ;;
  esac
done
if [ "$FAIL" = 1 ] && [ "$DRY" = 0 ]; then
  echo "not every channel is ERROR-ACTIVE: check that channel's cable, termination and motor power, then run '$0 re'"
  exit 1
fi
exit 0
