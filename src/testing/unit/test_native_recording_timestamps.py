"""Demux read-ahead after mux close is distinct from actual recorded input."""

import pytest

from integrations.recording.native_screen import native_input_timestamps


def test_legacy_input_log_keeps_every_frame() -> None:
    stamps, trailing = native_input_timestamps(
        "demuxer -> type:video pkt_pts:1000000\n"
        "demuxer -> type:video pkt_pts:1100000\n"
        "video:12kB audio:0kB muxing overhead: 0.1%\n"
    )
    assert stamps == [1.0, 1.1]
    assert trailing == 0


def test_only_packet_after_mux_close_is_reported_as_shutdown_read_ahead() -> None:
    stamps, trailing = native_input_timestamps(
        "[vist#0] demuxer -> type:video pkt_pts:1000000\n"
        "[vist#0] demuxer -> type:video pkt_pts:1100000\n"
        "[out#0] video:12kB audio:0kB muxing overhead: 0.1%\n"
        "[vist#0] demuxer -> type:video pkt_pts:1200000\n"
    )
    assert stamps == [1.0, 1.1]
    assert trailing == 1


def test_interior_inputs_remain_in_strict_frame_accounting() -> None:
    stamps, trailing = native_input_timestamps(
        "demuxer -> type:video pkt_pts:1000000\n"
        "demuxer -> type:video pkt_pts:1100000\n"
        "demuxer -> type:video pkt_pts:1200000\n"
        "video:12kB audio:0kB muxing overhead: 0.1%\n"
    )
    assert stamps == [1.0, 1.1, 1.2]
    assert trailing == 0


def test_multiple_forward_read_ahead_packets_stay_outside_recorded_input() -> None:
    stamps, trailing = native_input_timestamps(
        "demuxer -> type:video pkt_pts:1000000\n"
        "video:12kB audio:0kB muxing overhead: 0.1%\n"
        "demuxer -> type:video pkt_pts:1100000\n"
        "demuxer -> type:video pkt_pts:1200000\n"
    )
    assert stamps == [1.0]
    assert trailing == 2


def test_shutdown_input_must_not_overlap_recorded_interval() -> None:
    with pytest.raises(ValueError, match="overlaps"):
        native_input_timestamps(
            "demuxer -> type:video pkt_pts:1100000\n"
            "video:12kB audio:0kB muxing overhead: 0.1%\n"
            "demuxer -> type:video pkt_pts:1000000\n"
        )
