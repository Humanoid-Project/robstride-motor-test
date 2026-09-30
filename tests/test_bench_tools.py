import argparse
import contextlib
import csv
import importlib.util
import io
import json
import struct
import sys
import threading
from pathlib import Path

import can
import pytest

from robonex_common.buses import bus_map
from robonex_common.protocol import (
    COMM_DEVICE_ID,
    COMM_PARAMETER_READ,
    DEVICE_ID_DESTINATION,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    build_arbitration_id,
    parse_arbitration_id,
)

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import motor_selection  # noqa: E402


def load_script(relative):
    path = SCRIPTS / relative
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


read_joint_values = load_script("measurements/joint/read_joint_values.py")
motor_id_tool = load_script("calibration/motor_id/motor_id.py")
set_motor_pose = load_script("motor_control/set_motor_pose.py")


class FakeMotors:
    def __init__(self, channel, positions):
        self.positions = positions
        self.bus = can.Bus(interface="virtual", channel=channel)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            msg = self.bus.recv(timeout=0.01)
            if msg is None:
                continue
            comm_type, data16, target = parse_arbitration_id(msg.arbitration_id)
            if target not in self.positions:
                continue
            if comm_type == COMM_DEVICE_ID:
                self.bus.send(can.Message(
                    arbitration_id=build_arbitration_id(COMM_DEVICE_ID, target, DEVICE_ID_DESTINATION),
                    data=bytes([target] * 8), is_extended_id=True))
            elif comm_type == COMM_PARAMETER_READ:
                payload = bytearray(8)
                struct.pack_into("<H", payload, 0, MECHANICAL_POSITION_INDEX)
                struct.pack_into("<f", payload, 4, self.positions[target])
                self.bus.send(can.Message(
                    arbitration_id=build_arbitration_id(COMM_PARAMETER_READ, target, data16 & 0xFF),
                    data=bytes(payload), is_extended_id=True))

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.0)
        self.bus.shutdown()


@contextlib.contextmanager
def fake_motors(layout):
    fakes = [FakeMotors(channel, positions) for channel, positions in layout.items()]
    try:
        yield
    finally:
        for fake in fakes:
            fake.close()


@pytest.fixture
def robot(tmp_path, monkeypatch):
    bus_file = tmp_path / "bus_map.json"
    bus_file.write_text(json.dumps({}))
    monkeypatch.setenv("ROBONEX_BUS_MAP", str(bus_file))
    identity = tmp_path / "robot_model"
    monkeypatch.setenv(motor_selection.ROBOT_IDENTITY_ENV, str(identity))
    bus_map.cache_clear()

    def attach(variant=None, mapping=None):
        if mapping is not None:
            bus_file.write_text(json.dumps(mapping))
        bus_map.cache_clear()
        if variant is None:
            identity.unlink(missing_ok=True)
        else:
            identity.write_text(variant + "\n")

    yield attach
    bus_map.cache_clear()


def ids(joints):
    return [joint.motor_id for joint in joints]


def test_variant_default_sets(robot):
    robot("ver2_edu")
    assert ids(motor_selection.resolve_motors(None, "ver2_edu")) == list(range(1, 14))
    assert ids(motor_selection.resolve_motors(None, "ver2_pro")) == list(range(1, 14)) + [15, 16, 17, 18, 20, 21, 22, 23]
    assert ids(motor_selection.resolve_motors(None, "ver2_max")) == list(range(1, 19)) + [20, 21, 22, 23]
    assert motor_selection.attached_variant() == "ver2_edu"
    robot(None)
    assert motor_selection.attached_variant() is None
    assert ids(motor_selection.resolve_motors(None, None)) == list(range(1, 19)) + [20, 21, 22, 23]


def test_selectors(robot):
    resolve = motor_selection.resolve_motors
    assert ids(resolve(["head"], "ver2_max")) == [13, 14]
    assert ids(resolve(["head"], "ver2_edu")) == [13]
    assert ids(resolve(["arms"], "ver2_pro")) == [15, 16, 17, 18, 20, 21, 22, 23]
    assert ids(resolve(["legs"], "ver2_edu")) == list(range(1, 13))
    assert ids(resolve(["1,2", "left_knee_pitch", "r_knee_pitch_joint", "0x0c"], "ver2_edu")) == [1, 2, 4, 10, 12]
    assert ids(resolve(["all"], "ver2_edu")) == list(range(1, 14))
    with pytest.raises(motor_selection.SelectionError, match="not part of ver2_edu"):
        resolve(["14"], "ver2_edu")
    with pytest.raises(motor_selection.SelectionError, match="has no motors on ver2_edu"):
        resolve(["arms"], "ver2_edu")
    with pytest.raises(motor_selection.SelectionError, match="not a registered"):
        resolve(["19"], "ver2_max")
    with pytest.raises(motor_selection.SelectionError, match="unknown motor"):
        resolve(["left_toe"], "ver2_max")


def test_channels_follow_bus_map(robot):
    robot("ver2_max")
    channels = motor_selection.motors_by_channel(motor_selection.variant_motors("ver2_max"))
    assert {ch: ids(joints) for ch, joints in channels.items()} == {
        "can0": [1, 2, 3, 4, 5, 6], "can1": [7, 8, 9, 10, 11, 12], "can4": [13, 14],
        "can2": [15, 16, 17, 18], "can3": [20, 21, 22, 23],
    }
    assert "defaults" in motor_selection.bus_map_summary()
    robot("ver2_edu", {"head": "can0"})
    channels = motor_selection.motors_by_channel(motor_selection.variant_motors("ver2_edu"))
    assert ids(channels["can0"]) == [1, 2, 3, 4, 5, 6, 13]
    assert "overrides head -> can0" in motor_selection.bus_map_summary()


def test_unknown_identity_is_an_error(robot, tmp_path):
    robot("ver3_mega")
    with pytest.raises(motor_selection.SelectionError, match="unknown robot model"):
        motor_selection.attached_variant()


def test_read_once_on_virtual_bus(robot, tmp_path, capsys):
    robot("ver2_max")
    args = read_joint_values.build_parser().parse_args(["--ids", "left_leg", "head"])
    channels = read_joint_values.select_channels(args, "ver2_max")
    assert {ch: ids(joints) for ch, joints in channels.items()} == {"can0": [1, 2, 3, 4, 5, 6], "can4": [13, 14]}
    log = read_joint_values.CsvLog(tmp_path / "once.csv")
    with fake_motors({"can0": {1: 0.1, 2: -0.2, 3: 0.0, 4: -0.5, 5: 0.3, 6: 0.0}, "can4": {13: 0.7}}):
        missing = read_joint_values.read_once(channels, "virtual", HOST_ID, 0.05, log)
    log.close()
    out = capsys.readouterr().out
    assert missing == 1
    assert "[can4] head" in out
    assert "+0.1000 rad" in out
    line_14 = next(line for line in out.splitlines() if "neck_yaw" in line)
    assert "no response" in line_14 and "rs05" in line_14
    line_13 = next(line for line in out.splitlines() if "neck_pitch" in line)
    assert "beyond PLACEHOLDER limit" in line_13
    rows = list(csv.DictReader(open(tmp_path / "once.csv")))
    assert len(rows) == 8
    assert rows[0]["channel"] == "can0" and float(rows[0]["position_rad"]) == pytest.approx(0.1)
    assert rows[-1]["motor_id"] == "14" and rows[-1]["position_rad"] == ""


def test_missing_channel_shows_ip_link(robot, monkeypatch, capsys):
    robot("ver2_pro")
    real_bus = can.Bus

    def bus(channel, interface):
        if channel == "can2":
            raise OSError(19, "No such device")
        return real_bus(channel=channel, interface=interface)

    monkeypatch.setattr(motor_selection.can, "Bus", bus)
    args = read_joint_values.build_parser().parse_args(["--ids", "left_arm", "13"])
    channels = read_joint_values.select_channels(args, "ver2_pro")
    missing = read_joint_values.read_once(channels, "virtual", HOST_ID, 0.01)
    out = capsys.readouterr().out
    assert missing == 5
    assert "open failed" in out
    assert "sudo ip link set can2 up type can bitrate 1000000" in out
    assert out.count("channel not open") == 4


def test_can_filter(robot):
    robot("ver2_edu")
    parser = read_joint_values.build_parser()
    channels = read_joint_values.select_channels(parser.parse_args(["--can", "can1"]), "ver2_edu")
    assert list(channels) == ["can1"]
    with pytest.raises(motor_selection.SelectionError, match="no selected motor is on can3"):
        read_joint_values.select_channels(parser.parse_args(["--can", "can3"]), "ver2_edu")


def test_main_rejects_motor_outside_variant(robot, capsys):
    robot("ver2_edu")
    with pytest.raises(SystemExit):
        read_joint_values.main(["--ids", "14"])
    assert "not part of ver2_edu" in capsys.readouterr().err


def test_watch_two_seconds(robot, tmp_path):
    robot("ver2_edu")
    channels = read_joint_values.select_channels(
        read_joint_values.build_parser().parse_args(["--ids", "legs"]), "ver2_edu")
    out = io.StringIO()
    log = read_joint_values.CsvLog(tmp_path / "watch.csv")
    positions = {motor_id: 0.01 * motor_id for motor_id in range(1, 13)}
    with fake_motors({"can0": {k: v for k, v in positions.items() if k <= 6},
                      "can1": {k: v for k, v in positions.items() if k > 6}}):
        read_joint_values.watch(channels, "virtual", HOST_ID, 0.02, 10.0, 2.0, ["header line"], log, out)
    log.close()
    frames = out.getvalue().split(read_joint_values.CLEAR_SCREEN)
    assert len(frames) >= 15
    last = frames[-1]
    assert "header line" in last and "no response" not in last
    assert "+0.1200 rad" in last
    rates = [float(line.split()[-2]) for line in last.splitlines() if line.startswith("[can")]
    assert len(rates) == 2 and all(rate > 10.0 for rate in rates)
    rows = list(csv.DictReader(open(tmp_path / "watch.csv")))
    assert len(rows) > 12 * 20
    assert all(row["position_rad"] for row in rows)


def test_motor_id_find_on_virtual_bus(robot, monkeypatch, capsys):
    robot("ver2_max")
    monkeypatch.setattr(motor_id_tool, "DEFAULT_INTERFACE", "virtual")
    monkeypatch.setattr(motor_id_tool, "SCAN_TIMEOUT", 0.005)
    monkeypatch.setattr(motor_id_tool, "link_state", lambda channel: None)
    with fake_motors({"can4": {14: 0.0}, "can3": {20: 0.0}}):
        result = motor_id_tool.run_find(argparse.Namespace(can=None, motor_id=None))
        out = capsys.readouterr().out
        assert result == 0
        lines = {line.split(":")[0]: line for line in out.splitlines() if line.startswith("can")}
        assert "ID 14 (UID=" in lines["can4"]
        assert "ID 20 (UID=" in lines["can3"]
        assert lines["can0"] == "can0: no motors found"
        result = motor_id_tool.run_find(argparse.Namespace(can="can4", motor_id=14))
        assert result == 0
        assert "can4: ID 14" in capsys.readouterr().out


def test_motor_id_check_uses_variant(robot, monkeypatch, capsys):
    robot("ver2_edu")
    monkeypatch.setattr(motor_id_tool, "DEFAULT_INTERFACE", "virtual")
    monkeypatch.setattr(motor_id_tool, "SCAN_TIMEOUT", 0.01)
    monkeypatch.setattr(motor_id_tool, "link_state", lambda channel: None)
    layout = {"can0": {i: 0.0 for i in range(1, 7)}, "can1": {i: 0.0 for i in range(7, 13)}, "can4": {13: 0.0}}
    with fake_motors(layout):
        result = motor_id_tool.run_check(argparse.Namespace(can=None))
    out = capsys.readouterr().out
    assert result == 0
    assert "can2" not in out and "can3" not in out
    assert "can4: OK (IDs 13)" in out


def test_set_motor_pose_refuses_placeholder_limits(robot, monkeypatch, capsys):
    robot("ver2_max")

    def no_bus(*_args, **_kwargs):
        raise AssertionError("no bus may be opened")

    monkeypatch.setattr(set_motor_pose, "open_bus", no_bus)
    monkeypatch.setattr(sys, "argv", ["set_motor_pose.py", "--ids", "head"])
    assert set_motor_pose.main() == 1
    out = capsys.readouterr().out
    assert "Refusing IDs 13-14" in out and "--allow-placeholder-limits" in out
    monkeypatch.setattr(sys, "argv", ["set_motor_pose.py", "--ids", "14"])
    robot("ver2_edu")
    with pytest.raises(SystemExit):
        set_motor_pose.main()
