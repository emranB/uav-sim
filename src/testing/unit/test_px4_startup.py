"""Recording policy is explicit after the pinned default and before logger startup."""

import json
from pathlib import Path

import pytest

from integrations.px4.startup import prepare_startup_script


@pytest.mark.parametrize("mode,native_value", [("armed", 0), ("boot", 1)])
def test_private_startup_preserves_template_and_sets_policy_in_order(
    tmp_path: Path,
    mode: str,
    native_value: int,
) -> None:
    template = tmp_path / "template"
    original = b"before\nparam set-default SDLOG_MODE 1\nafter\n. rc.logging\n"
    template.write_bytes(original)
    run = tmp_path / "run"
    run.mkdir()
    destination = prepare_startup_script(template, run, mode)
    script = destination.read_text()
    assert (
        script.index("param set-default SDLOG_MODE 1")
        < script.index(f"param set SDLOG_MODE {native_value}")
        < script.index(". rc.logging")
    )
    assert template.read_bytes() == original
    assert (
        destination.read_bytes().replace(f"param set SDLOG_MODE {native_value}\n".encode(), b"")
        == original
    )
    policy = json.loads((run / "startup-policy.json").read_text())
    assert policy["sdlog_mode"] == native_value
    assert policy["startup_sha256"] != policy["template_sha256"]


@pytest.mark.parametrize("template_text", ["no marker", "param set-default SDLOG_MODE 1\n" * 2])
def test_unknown_startup_fails_without_writing_a_launch_script(
    tmp_path: Path,
    template_text: str,
) -> None:
    template = tmp_path / "template"
    template.write_text(template_text)
    run = tmp_path / "run"
    run.mkdir()
    with pytest.raises(RuntimeError, match="unique logging-default"):
        prepare_startup_script(template, run, "armed")
    assert not list(run.iterdir())


@pytest.mark.parametrize("source,enabled", [("simulation", 0), ("agent", 1)])
def test_dds_time_policy_is_explicit_before_client_launch(tmp_path, source, enabled) -> None:
    template = tmp_path / "template"
    original = (
        b"param set-default SDLOG_MODE 1\n. rc.logging\nuxrce_dds_client start -t udp -p 8888\n"
    )
    template.write_bytes(original)
    run = tmp_path / "run"
    run.mkdir()
    configured = prepare_startup_script(template, run, "armed", dds_time_source=source).read_text()
    assert configured.index(f"param set UXRCE_DDS_SYNCT {enabled}") < configured.index(
        "uxrce_dds_client start"
    )
    assert template.read_bytes() == original
    metadata = json.loads((run / "startup-policy.json").read_text())
    assert metadata["dds_time_source"] == source
    assert metadata["uxrce_dds_synct"] == enabled


def test_dds_policy_rejects_an_unknown_client_start_boundary(tmp_path) -> None:
    template = tmp_path / "template"
    template.write_text("param set-default SDLOG_MODE 1\n")
    run = tmp_path / "run"
    run.mkdir()
    with pytest.raises(RuntimeError, match="DDS-client start"):
        prepare_startup_script(template, run, "armed", dds_time_source="simulation")
    assert not list(run.iterdir())
