import can

try:
    import robonex_common
except ValueError as error:
    if type(error).__name__ != "BusMapError":
        raise
    raise SystemExit(f"ERROR: {error}") from None
from robonex_common.buses import DEFAULT_BUS_MAP, bus_map, bus_map_path
from robonex_common.identity import (
    ROBOT_IDENTITY_ENV,
    ROBOT_IDENTITY_FILE,
    IdentityError,
    identity_path,
    read_identity,
    require_identity,
)
from robonex_common.joints import (
    ALL_MOTORS,
    GROUP_ID_RANGES,
    LEGACY_VARIANT_NAMES,
    MOTOR_BY_ID,
    VARIANT_MOTOR_IDS,
    canonical_variant,
    motors_for_variant,
)
from robonex_common.limits import (
    PLACEHOLDER_CONSENT_FLAG,
    PlaceholderLimitError,
    require_placeholder_consent,
)

GROUP_ALIASES = {
    "legs": ("left_leg", "right_leg"),
    "arms": ("left_arm", "right_arm"),
}
SELECTOR_NAMES = tuple(GROUP_ID_RANGES) + tuple(GROUP_ALIASES) + ("all",)
MOTOR_BY_NAME = {
    name: joint for joint in ALL_MOTORS for name in (joint.hardware_name, joint.model_name)
}
SELECTOR_HELP = (
    "motor IDs, group names (" + ", ".join(SELECTOR_NAMES) + ") or joint names "
    "such as left_knee_pitch; space- or comma-separated"
)


class SelectionError(ValueError):
    pass


class BusOpenError(RuntimeError):
    def __init__(self, channel, reason):
        self.channel = channel
        self.reason = reason
        self.hint = ip_link_hint(channel)
        super().__init__(f"{channel}: open failed ({reason}); bring it up with: {self.hint}")


def checked_variant(value, path=None):
    path = identity_path() if path is None else path
    if canonical_variant(value) not in VARIANT_MOTOR_IDS:
        known = ", ".join(sorted(VARIANT_MOTOR_IDS))
        legacy = ", ".join(f"{old} = {new}" for old, new in LEGACY_VARIANT_NAMES.items())
        raise SelectionError(f"{path} names an unknown robot model {value!r} (known: {known}; legacy: {legacy})")
    return canonical_variant(value)


def attached_variant(path=None):
    try:
        value = read_identity(path)
    except IdentityError as error:
        raise SelectionError(str(error)) from None
    return None if value is None else checked_variant(value, path)


def required_variant(path=None):
    try:
        value = require_identity(path)
    except IdentityError as error:
        raise SelectionError(str(error)) from None
    return checked_variant(value, path)


def add_placeholder_option(parser):
    parser.add_argument(PLACEHOLDER_CONSENT_FLAG, action="store_true",
                        help="Also move head and arm motors (IDs 16-27), whose joint limits are unmeasured "
                             "placeholders; joint-limit checks still use those placeholders")


def motion_problems(motor_ids, allow_placeholder_limits, path=None):
    problems = []
    try:
        required_variant(path)
    except SelectionError as error:
        problems.append(str(error))
    registered = [motor_id for motor_id in motor_ids if motor_id in MOTOR_BY_ID]
    try:
        require_placeholder_consent(registered, allowed=allow_placeholder_limits)
    except PlaceholderLimitError as error:
        problems.append(str(error))
    return problems


def variant_motors(variant):
    return ALL_MOTORS if variant is None else motors_for_variant(variant)


def format_ids(motor_ids):
    ids = sorted(motor_ids)
    runs = []
    for motor_id in ids:
        if runs and motor_id == runs[-1][1] + 1:
            runs[-1][1] = motor_id
        else:
            runs.append([motor_id, motor_id])
    return ", ".join(str(lo) if lo == hi else f"{lo}-{hi}" for lo, hi in runs)


def pool_label(variant, pool):
    name = variant or "every registered motor"
    return f"{name} (IDs {format_ids(joint.motor_id for joint in pool)})"


def split_selectors(tokens):
    return [part.strip() for raw in tokens for part in str(raw).split(",") if part.strip()]


def parse_int(token):
    try:
        return int(token, 0)
    except ValueError:
        return None


def match_selector(token, pool, variant):
    if token == "all":
        return tuple(pool)
    if token in GROUP_ALIASES or token in GROUP_ID_RANGES:
        groups = GROUP_ALIASES.get(token, (token,))
        matches = tuple(joint for joint in pool if joint.group in groups)
        if not matches:
            raise SelectionError(f"{token!r} has no motors on {pool_label(variant, pool)}")
        return matches
    motor_id = parse_int(token)
    if motor_id is not None:
        joint = MOTOR_BY_ID.get(motor_id)
        if joint is None:
            raise SelectionError(f"ID {token} is not a registered RoboNex motor (known IDs {format_ids(MOTOR_BY_ID)})")
    else:
        joint = MOTOR_BY_NAME.get(token)
        if joint is None:
            raise SelectionError(f"unknown motor {token!r}; use {SELECTOR_HELP}")
    if joint not in pool:
        raise SelectionError(
            f"ID {joint.motor_id} ({joint.hardware_name}) is not part of {pool_label(variant, pool)}"
        )
    return (joint,)


def resolve_motors(tokens, variant):
    pool = variant_motors(variant)
    if not tokens:
        return tuple(pool)
    selected = {}
    for token in split_selectors(tokens):
        for joint in match_selector(token, pool, variant):
            selected[joint.motor_id] = joint
    if not selected:
        raise SelectionError("no motors selected")
    return tuple(selected[motor_id] for motor_id in sorted(selected))


def motors_by_channel(motors):
    channels = {}
    for joint in sorted(motors, key=lambda joint: joint.motor_id):
        channels.setdefault(joint.channel, []).append(joint)
    return {channel: tuple(joints) for channel, joints in channels.items()}


def channel_groups(channel):
    return [group for group, mapped in bus_map().items() if mapped == channel]


def bus_map_summary():
    mapping = bus_map()
    overrides = {group: channel for group, channel in mapping.items() if DEFAULT_BUS_MAP.get(group) != channel}
    if not overrides:
        return "bus map : defaults (" + ", ".join(f"{g} {c}" for g, c in mapping.items()) + ")"
    changes = ", ".join(f"{group} -> {channel}" for group, channel in overrides.items())
    return f"bus map : {bus_map_path()} overrides {changes}"


def variant_summary(variant, pool):
    if variant is None:
        return (f"robot   : unknown (no {identity_path()}); using every registered motor "
                f"(IDs {format_ids(joint.motor_id for joint in pool)})")
    return f"robot   : {variant} (from {identity_path()}), IDs {format_ids(joint.motor_id for joint in pool)}"


def ip_link_hint(channel):
    return f"sudo ip link set {channel} up type can bitrate 1000000"


def open_bus(channel, interface):
    try:
        return can.Bus(channel=channel, interface=interface)
    except (OSError, can.CanError) as error:
        raise BusOpenError(channel, error) from error
