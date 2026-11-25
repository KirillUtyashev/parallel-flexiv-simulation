#!/usr/bin/env python3
"""Tests for fail-closed private FlexivSimulation binary preparation."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest


FLEET_SCRIPT_DIR = Path(__file__).resolve().parents[2] / "scripts" / "controllers"
sys.path.insert(0, str(FLEET_SCRIPT_DIR))

from prepare_flexiv_simulation_binary import (  # noqa: E402
    DEFAULT_INITIAL_PEERS_RANGE,
    DEFAULT_MUTATION_TRIES,
    PreparationError,
    RUN_BUSY_WAIT_PATCH,
    RUN_BUSY_WAIT_SIGNATURE,
    prepare,
)


def prepare_default(source: Path, output: Path) -> tuple[str, str]:
    return prepare(
        source,
        output,
        DEFAULT_INITIAL_PEERS_RANGE,
        DEFAULT_MUTATION_TRIES,
    )


def test_busy_wait_is_replaced_and_source_is_untouched(tmp_path: Path) -> None:
    original = b"prefix" + RUN_BUSY_WAIT_SIGNATURE + b"suffix"
    source = tmp_path / "FlexivSimulation"
    output = tmp_path / "private" / "FlexivSimulation"
    source.write_bytes(original)

    source_digest, output_digest = prepare_default(source, output)

    assert source.read_bytes() == original
    assert output.read_bytes() == b"prefix" + RUN_BUSY_WAIT_PATCH + b"suffix"
    assert source_digest != output_digest


def test_busy_wait_can_be_preserved_in_private_copy(tmp_path: Path) -> None:
    original = b"prefix" + RUN_BUSY_WAIT_SIGNATURE + b"suffix"
    source = tmp_path / "FlexivSimulation"
    output = tmp_path / "private" / "FlexivSimulation"
    source.write_bytes(original)

    source_digest, output_digest = prepare(
        source,
        output,
        DEFAULT_INITIAL_PEERS_RANGE,
        DEFAULT_MUTATION_TRIES,
        optimize_run_wait=False,
    )

    assert source.read_bytes() == original
    assert output.read_bytes() == original
    assert source_digest == output_digest


def test_missing_busy_wait_signature_fails_closed(tmp_path: Path) -> None:
    source = tmp_path / "FlexivSimulation"
    output = tmp_path / "private" / "FlexivSimulation"
    source.write_bytes(b"unsupported binary")

    with pytest.raises(PreparationError, match="busy-wait signature was not found"):
        prepare_default(source, output)

    assert not output.exists()


def test_ambiguous_busy_wait_signature_fails_closed(tmp_path: Path) -> None:
    source = tmp_path / "FlexivSimulation"
    output = tmp_path / "private" / "FlexivSimulation"
    source.write_bytes(RUN_BUSY_WAIT_SIGNATURE * 2)

    with pytest.raises(PreparationError, match="busy-wait signature is ambiguous"):
        prepare_default(source, output)

    assert not output.exists()
