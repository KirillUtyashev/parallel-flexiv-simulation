#!/usr/bin/env python3
"""Prepare an isolated RobotControlApp binary and communication config.

The installed RobotControlApp is never modified.  Binary patch locations are
accepted only for an exact, known build and are checked before they are
changed.
"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import re
import shutil
import stat
import struct
import sys
import tempfile


ORIGINAL_TIME_SYNC_PORT = 14001
SUPPORTED_BINARY_SHA256 = (
    "c8f9a77e4c6fbf6b8894e431edbd7fcde9abddc94997b3eb3f39304afe146010"
)
PATCH_SITES = (
    (0x2A99286, 2, "big", "IPv4 time-sync endpoint"),
    (0x2A994C4, 2, "big", "IPv6 time-sync endpoint"),
    (0x2A99E2E, 4, "little", "time-sync log message"),
)
DDS_DEFAULT_MUTATION_TRIES = 100
# Both middleware2 participant constructors in the supported RobotControlApp
# initialize Fast DDS BuiltinAttributes::mutation_tries with this instruction.
DDS_MUTATION_TRIES_SIGNATURE = bytes.fromhex("c78424e802000064000000")
DDS_MUTATION_TRIES_IMMEDIATE_OFFSET = 7
DDS_MUTATION_TRIES_SITE_COUNT = 2
PORT_PATTERN = re.compile(r'(\bport\s*:\s*")([0-9]+)(")')


class PreparationError(RuntimeError):
    """Raised when a runtime cannot be prepared safely."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def checked_port(value: int, label: str) -> int:
    if not 1 <= value <= 65535:
        raise PreparationError(f"{label} must be between 1 and 65535, got {value}")
    return value


def remap_config(
    text: str,
    time_sync_port: int,
    port_offset: int | None,
    port_base: int | None,
    port_block_size: int,
) -> tuple[str, list[int], str]:
    source_ports = [int(match.group(2)) for match in PORT_PATTERN.finditer(text)]
    if not source_ports:
        raise PreparationError("no quoted numeric port fields found in communication config")

    unique_source_ports = list(dict.fromkeys(source_ports))
    port_map: dict[int, int]
    strategy: str
    if port_offset is not None:
        if port_offset < 0:
            raise PreparationError(
                f"port offset must be non-negative, got {port_offset}"
            )
        port_map = {
            source_port: checked_port(
                source_port + port_offset, "remapped config port"
            )
            for source_port in unique_source_ports
        }
        strategy = f"offset={port_offset}"
    else:
        if port_base is None:
            raise PreparationError("dense port allocation requires a port base")
        checked_port(port_base, "port block base")
        if port_block_size < 2:
            raise PreparationError(
                f"port block size must be at least 2, got {port_block_size}"
            )
        block_end = port_base + port_block_size - 1
        checked_port(block_end, "port block end")
        if not port_base <= time_sync_port <= block_end:
            raise PreparationError(
                f"time-sync port {time_sync_port} is outside dense block "
                f"{port_base}-{block_end}"
            )
        available_ports = [
            port
            for port in range(port_base, block_end + 1)
            if port != time_sync_port
        ]
        if len(unique_source_ports) > len(available_ports):
            raise PreparationError(
                f"dense block {port_base}-{block_end} has room for "
                f"{len(available_ports)} config ports, but "
                f"{len(unique_source_ports)} are required"
            )
        port_map = dict(
            zip(
                unique_source_ports,
                available_ports[: len(unique_source_ports)],
                strict=True,
            )
        )
        strategy = f"block={port_base}-{block_end}"

    remapped_ports = list(port_map.values())

    def replace(match: re.Match[str]) -> str:
        source_port = int(match.group(2))
        remapped_port = port_map[source_port]
        return f"{match.group(1)}{remapped_port}{match.group(3)}"

    remapped_text = PORT_PATTERN.sub(replace, text)
    if len(remapped_ports) != len(set(remapped_ports)):
        raise PreparationError("port allocation produced duplicate communication ports")
    if time_sync_port in remapped_ports:
        raise PreparationError(
            f"time-sync port {time_sync_port} collides with a remapped communication port"
        )
    return remapped_text, remapped_ports, strategy


def patch_binary(
    path: Path,
    time_sync_port: int,
    dds_mutation_tries: int = DDS_DEFAULT_MUTATION_TRIES,
) -> None:
    with path.open("r+b") as stream:
        for offset, width, byteorder, description in PATCH_SITES:
            expected = ORIGINAL_TIME_SYNC_PORT.to_bytes(width, byteorder)
            replacement = time_sync_port.to_bytes(width, byteorder)
            stream.seek(offset)
            actual = stream.read(width)
            if actual != expected:
                raise PreparationError(
                    f"unexpected bytes at {description} offset 0x{offset:x}: "
                    f"expected {expected.hex()}, found {actual.hex()}"
                )
            stream.seek(offset)
            stream.write(replacement)

        if dds_mutation_tries != DDS_DEFAULT_MUTATION_TRIES:
            stream.seek(0)
            contents = stream.read()
            site_count = contents.count(DDS_MUTATION_TRIES_SIGNATURE)
            if site_count != DDS_MUTATION_TRIES_SITE_COUNT:
                raise PreparationError(
                    "unexpected RobotControlApp DDS mutation-tries signature count: "
                    f"expected {DDS_MUTATION_TRIES_SITE_COUNT}, found {site_count}"
                )
            replacement = bytearray(DDS_MUTATION_TRIES_SIGNATURE)
            replacement[
                DDS_MUTATION_TRIES_IMMEDIATE_OFFSET:
                DDS_MUTATION_TRIES_IMMEDIATE_OFFSET + 4
            ] = struct.pack("<I", dds_mutation_tries)
            stream.seek(0)
            stream.write(contents.replace(DDS_MUTATION_TRIES_SIGNATURE, replacement))


def temporary_path(destination: Path, suffix: str) -> Path:
    descriptor, name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=suffix, dir=destination.parent
    )
    os.close(descriptor)
    return Path(name)


def prepare(
    source_binary: Path,
    output_binary: Path,
    comm_config: Path,
    time_sync_port: int,
    port_offset: int | None,
    port_base: int | None,
    port_block_size: int,
    dds_mutation_tries: int = DDS_DEFAULT_MUTATION_TRIES,
) -> int:
    if source_binary.resolve() == output_binary.resolve():
        raise PreparationError("output binary must not be the installed source binary")
    if not source_binary.is_file():
        raise PreparationError(f"source binary not found: {source_binary}")
    if not comm_config.is_file():
        raise PreparationError(f"communication config not found: {comm_config}")
    checked_port(time_sync_port, "time-sync port")
    if not DDS_DEFAULT_MUTATION_TRIES <= dds_mutation_tries <= 65_535:
        raise PreparationError(
            "DDS mutation tries must be between "
            f"{DDS_DEFAULT_MUTATION_TRIES} and 65535, got {dds_mutation_tries}"
        )

    source_hash = sha256(source_binary)
    if source_hash != SUPPORTED_BINARY_SHA256:
        raise PreparationError(
            "unsupported RobotControlApp build: "
            f"expected SHA-256 {SUPPORTED_BINARY_SHA256}, found {source_hash}"
        )

    config_text = comm_config.read_text(encoding="utf-8")
    remapped_text, remapped_ports, strategy = remap_config(
        config_text,
        time_sync_port,
        port_offset,
        port_base,
        port_block_size,
    )

    output_binary.parent.mkdir(parents=True, exist_ok=True)
    binary_temp = temporary_path(output_binary, ".tmp")
    config_temp = temporary_path(comm_config, ".tmp")
    try:
        shutil.copy2(source_binary, binary_temp)
        patch_binary(binary_temp, time_sync_port, dds_mutation_tries)

        config_temp.write_text(remapped_text, encoding="utf-8")
        config_mode = stat.S_IMODE(comm_config.stat().st_mode)
        config_temp.chmod(config_mode)

        os.replace(binary_temp, output_binary)
        os.replace(config_temp, comm_config)
    finally:
        binary_temp.unlink(missing_ok=True)
        config_temp.unlink(missing_ok=True)

    print(
        f"prepared {output_binary}: config_ports={len(remapped_ports)} "
        f"{strategy} range={min(remapped_ports)}-{max(remapped_ports)} "
        f"time_sync={time_sync_port} dds_mutation_tries={dds_mutation_tries}"
    )
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-binary", required=True, type=Path)
    parser.add_argument("--output-binary", required=True, type=Path)
    parser.add_argument("--comm-config", required=True, type=Path)
    port_strategy = parser.add_mutually_exclusive_group(required=True)
    port_strategy.add_argument("--port-offset", type=int)
    port_strategy.add_argument("--port-base", type=int)
    parser.add_argument("--port-block-size", type=int, default=64)
    parser.add_argument("--time-sync-port", required=True, type=int)
    parser.add_argument(
        "--dds-mutation-tries", type=int, default=DDS_DEFAULT_MUTATION_TRIES
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        return prepare(
            args.source_binary,
            args.output_binary,
            args.comm_config,
            args.time_sync_port,
            args.port_offset,
            args.port_base,
            args.port_block_size,
            args.dds_mutation_tries,
        )
    except (OSError, UnicodeError, PreparationError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
