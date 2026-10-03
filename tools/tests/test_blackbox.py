"""Tests for tools/blackbox.py. Run with:  python -m pytest tools/tests -q"""

from __future__ import annotations

import os
import struct
from pathlib import Path

import numpy as np
import pytest

from tools import blackbox as bb

# committed fixture (recorded on Linux); SPACECAL_TEST_FIXTURE points at a freshly recorded one
# (test.bat records it on Windows into bin-tests\ so the cross-platform layout is verified too)
FIXTURE = Path(os.environ.get("SPACECAL_TEST_FIXTURE", Path(__file__).parent / "fixtures" / "event_sample"))


def write_chunk(path: Path, records: list[tuple], mono_start=1000.0, unix_start=1.7e9, chunk_index=0, session_id=0xABCD1234, version="v-test"):
    header = bb.HEADER_STRUCT.pack(bb.MAGIC, bb.FORMAT_VERSION, bb.RECORD_SIZE, bb.HEADER_SIZE, 0, chunk_index, mono_start, unix_start, session_id, 60, version.encode())
    arr = np.zeros(len(records), dtype=bb.RECORD_DTYPE)
    for i, (t, rtype, device, a, b, code, f0, v) in enumerate(records):
        arr[i]["t"] = t
        arr[i]["type"] = rtype
        arr[i]["device"] = device
        arr[i]["a"] = a
        arr[i]["b"] = b
        arr[i]["code"] = code
        arr[i]["f0"] = f0
        arr[i]["v"][: len(v)] = v
    path.write_bytes(header + arr.tobytes())


def text_record(t, rtype, device, kind, seq, total, text: str):
    arr = np.zeros(1, dtype=bb.RECORD_DTYPE)
    arr[0]["t"] = t
    arr[0]["type"] = rtype
    arr[0]["device"] = device
    arr[0]["a"] = kind
    arr[0]["b"] = seq
    arr[0]["code"] = total
    raw = bytearray(arr.tobytes())
    payload = text.encode()[: bb.TEXT_PAYLOAD_SIZE]
    raw[16 : 16 + len(payload)] = payload
    return bytes(raw)


def test_layout_matches_header():
    assert bb.RECORD_DTYPE.itemsize == 80
    assert bb.HEADER_STRUCT.size == 64
    assert bb.RECORD_DTYPE.fields["f0"][1] == 16  # text payload starts at byte 16


def test_read_chunk_and_truncated_tail(tmp_path):
    p = tmp_path / "c.scb"
    recs = [
        (1000.0, bb.TYPE_SESSION, 255, 0, 0, 0, 0.0, [0, 0, 0.25]),
        (1000.01, bb.TYPE_POSE, 3, 200, bb.POSE_FLAG_VALID | bb.POSE_FLAG_CONNECTED, 0, -0.004, [1, 2, 3, 1, 0, 0, 0, 0.5, 0, 0, 0, 0, 0.25]),
        (1000.02, bb.TYPE_MARKER, 255, 1, 0, 7, 0.0, []),
    ]
    write_chunk(p, recs)
    # append half a record to simulate a copy taken while the chunk grew
    p.write_bytes(p.read_bytes() + b"\x00" * 37)
    c = bb.read_chunk(p)
    assert c.header.version == "v-test"
    assert c.header.session_id == 0xABCD1234
    assert c.truncated_tail
    assert len(c.records) == 3
    assert bb.RECORD_TYPES[int(c.records[1]["type"])] == "POSE"
    assert c.records[1]["device"] == 3
    assert abs(float(c.records[1]["f0"]) + 0.004) < 1e-6
    assert abs(float(c.records[1]["v"][12]) - 0.25) < 1e-6


def test_load_event_frames_and_texts(tmp_path):
    pytest.importorskip("pandas")
    folder = tmp_path / "evt"
    folder.mkdir()
    # two chunks out of order on disk, second chunk starts later
    write_chunk(folder / "00000001_000001.scb", [(1010.0, bb.TYPE_POSE, 0, 200, 3, 0, 0.0, [0, 1, 0, 1, 0, 0, 0])], mono_start=1010.0, unix_start=1.7e9 + 10, chunk_index=1)
    long_text = "class=2;role=0;sys=lighthouse;model=VIVE Tracker 3.0 MV;serial=LHR-ABCDEF01;extra=" + "x" * 30
    assert len(long_text) > 64
    raw = b"".join(
        [
            bb.HEADER_STRUCT.pack(bb.MAGIC, bb.FORMAT_VERSION, bb.RECORD_SIZE, bb.HEADER_SIZE, 0, 0, 1000.0, 1.7e9, 1, 60, b"v"),
            text_record(1000.0, bb.TYPE_DEVICE, 5, 0, 0, 2, long_text[:64]),
            text_record(1000.0, bb.TYPE_DEVICE, 5, 0, 1, 2, long_text[64:]),
            text_record(1000.5, bb.TYPE_TEXT, 255, 1, 0, 1, "2026-01-01_00-00-00_hotkey"),
        ]
    )
    (folder / "00000001_000000.scb").write_bytes(raw)
    write_chunk(folder / "00000001_000000.scb", [(1000.1, bb.TYPE_POSE, 5, 200, 3, 0, 0.0, [1, 1, 1, 1, 0, 0, 0]), (1000.2, bb.TYPE_MARKER, 255, 1, 0, 3, 0.0, [])], mono_start=1000.0, unix_start=1.7e9, chunk_index=0)
    # prepend the text records to chunk 0 (write_chunk overwrote it)
    data = (folder / "00000001_000000.scb").read_bytes()
    (folder / "00000001_000000.scb").write_bytes(data[:64] + raw[64:] + data[64:])
    (folder / "driver.json").write_text('{"source": "hotkey", "t_mark_mono": 1000.2, "incomplete": false}')

    ev = bb.load_event(folder)
    assert len(ev.chunks) == 2
    assert ev.t_mark == 1000.2
    assert list(ev.records["t"]) == sorted(ev.records["t"])
    assert ev.devices[5]["model"] == "VIVE Tracker 3.0 MV"
    assert ev.devices[5]["serial"] == "LHR-ABCDEF01"
    assert ev.devices[5]["class"] == 2
    folders = [t["text"] for t in ev.texts if t["kind"] == "marker_folder"]
    assert folders == ["2026-01-01_00-00-00_hotkey"]
    poses = ev.poses
    assert len(poses) == 2
    assert set(poses["device"]) == {0, 5}
    # unix time follows each chunk's own offset
    assert abs(poses[poses["device"] == 0]["unix"].iloc[0] - (1.7e9 + 10)) < 1e-3
    assert abs(poses[poses["device"] == 5]["unix"].iloc[0] - (1.7e9 + 0.1)) < 1e-3
    markers = ev.markers
    assert list(markers["source"]) == ["hotkey"]
    assert list(markers["label"]) == [3]


def test_calibration_state_and_lifecycle_frames(tmp_path):
    pytest.importorskip("pandas")
    folder = tmp_path / "sess"
    folder.mkdir()
    header = bb.HEADER_STRUCT.pack(bb.MAGIC, bb.FORMAT_VERSION, bb.RECORD_SIZE, bb.HEADER_SIZE, 0, 0, 1000.0, 1.7e9, 5, 60, b"v")
    arr = np.zeros(5, dtype=bb.RECORD_DTYPE)
    # applied continuous solve on device 3 against the HMD (0)
    arr[0]["t"], arr[0]["type"], arr[0]["device"], arr[0]["reserved"] = 1000.1, bb.TYPE_CALIBRATION, 3, 0
    arr[0]["a"], arr[0]["b"], arr[0]["code"], arr[0]["f0"] = 2, 1 | bb.CALIB_FLAG_CONTINUOUS, 0, 0.015
    arr[0]["v"][:13] = [0.02, 100, 0.01, 0.5, 1, 2, 3, 1, 0, 0, 0, 0.9, 0.8]
    # rejected startup solve without a previous calibration
    arr[1]["t"], arr[1]["type"], arr[1]["device"], arr[1]["a"], arr[1]["b"], arr[1]["code"], arr[1]["f0"] = 1000.2, bb.TYPE_CALIBRATION, 3, 3, 0, 3, 0.2
    arr[1]["v"][0] = -1
    # device 4 connects, then drops with a result above 255
    arr[2]["t"], arr[2]["type"], arr[2]["device"], arr[2]["a"], arr[2]["b"], arr[2]["code"] = 1000.3, bb.TYPE_DEVICE_STATE, 4, 1, 255, 200
    arr[3]["t"], arr[3]["type"], arr[3]["device"], arr[3]["a"], arr[3]["b"], arr[3]["code"] = 1000.4, bb.TYPE_DEVICE_STATE, 4, 0, 1, 300
    arr[4]["t"], arr[4]["type"], arr[4]["a"], arr[4]["code"] = 1000.5, bb.TYPE_LIFECYCLE, 3, 7
    (folder / "00000005_000000.scb").write_bytes(header + arr.tobytes())

    ev = bb.load_event(folder)
    cal = ev.calibrations
    assert list(cal["trigger"]) == ["continuous", "startup"]
    assert list(cal["outcome"]) == ["applied", "rejected"]
    assert list(cal["error_name"]) == ["none", "rms_too_high"]
    assert list(cal["continuous"]) == [True, False]
    assert cal["reference"].iloc[0] == 0 and cal["target"].iloc[0] == 3
    assert abs(cal["delta_rot_deg"].iloc[0] - 0.5) < 1e-6
    assert abs(cal["prev_rms_m"].iloc[0] - 0.02) < 1e-6
    assert np.isnan(cal["prev_rms_m"].iloc[1])
    st = ev.device_state
    assert list(st["connected"]) == [True, False]
    assert list(st["previous"]) == [None, True]
    assert list(st["result_name"]) == ["Running_OK", "Fallback_RotationOnly"]
    lc = ev.lifecycle
    assert list(lc["kind"]) == ["overlay_connected"]
    assert lc["code"].iloc[0] == 7


def test_pose_result_prefers_code():
    arr = np.zeros(2, dtype=bb.RECORD_DTYPE)
    arr[0]["a"], arr[0]["code"] = 200, 200
    arr[1]["a"], arr[1]["code"] = 300 % 256, 300  # format 2 keeps the full value in code
    assert list(bb.pose_results(arr)) == [200, 300]


@pytest.mark.skipif(not FIXTURE.exists(), reason="fixture recorded by the C++ test is not present")
def test_cpp_recorded_fixture():
    pytest.importorskip("pandas")
    ev = bb.load_event(FIXTURE)
    assert len(ev.chunks) >= 1
    assert ev.driver_json is not None
    assert not ev.poses.empty
    assert (ev.poses["result_name"] == "Running_OK").all()
    assert len(ev.markers) == 1
    assert ev.markers["source"].iloc[0] == "hotkey"
    assert any(t["kind"] == "marker_folder" for t in ev.texts)
    if os.environ.get("SPACECAL_TEST_FIXTURE"):
        # freshly recorded by the C++ test (format 2): one calibration attempt in the window
        cal = ev.calibrations
        assert len(cal) == 1
        assert cal["outcome"].iloc[0] == "applied"
        assert cal["trigger"].iloc[0] == "continuous"
