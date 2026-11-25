#!/usr/bin/env python3
"""Prepare a private Sim Plugin archive for high-count FlexivNode DDS probes.

The dummy peer uses the vendor's complementary FlexivNode implementation.  Its
DefaultNode receives the same enlarged local locator retry budget used by the
private FlexivSimulation binary so a 100-port bind retry cannot contaminate the
discovery-capacity experiment.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile


SUPPORTED_ARCHIVE_SHA256 = (
    "11aafe4a3dc4f5cb8a1178d782dc17c8208cad7b39725923086d835781ba312e"
)
DEFAULT_NODE_MEMBER = "default_node.cc.o"
DEFAULT_MUTATION_TRIES = 100
MUTATION_SIGNATURE = bytes.fromhex("c78424c002000064000000")
MUTATION_IMMEDIATE_OFFSET = 7


class PreparationError(RuntimeError):
    """Raised when the exact supported archive cannot be prepared safely."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run(command: list[str], cwd: Path) -> None:
    try:
        subprocess.run(command, cwd=cwd, check=True, capture_output=True)
    except FileNotFoundError as error:
        raise PreparationError(f"required archive tool is unavailable: {command[0]}") from error
    except subprocess.CalledProcessError as error:
        details = error.stderr.decode("utf-8", errors="replace").strip()
        raise PreparationError(
            f"archive command failed [{command[0]}]: {details or error.returncode}"
        ) from error


def prepare(source: Path, output: Path, mutation_tries: int) -> tuple[str, str]:
    source = source.resolve()
    output = output.resolve()
    if not source.is_file():
        raise PreparationError(f"Sim Plugin archive not found: {source}")
    if source == output:
        raise PreparationError("the installed Sim Plugin archive must remain untouched")
    source_digest = sha256(source)
    if source_digest != SUPPORTED_ARCHIVE_SHA256:
        raise PreparationError(
            "unsupported Sim Plugin archive: "
            f"expected {SUPPORTED_ARCHIVE_SHA256}, found {source_digest}"
        )
    if not DEFAULT_MUTATION_TRIES <= mutation_tries <= 65_535:
        raise PreparationError("mutation tries must be in [100, 65535]")

    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, output)
    try:
        with tempfile.TemporaryDirectory(prefix="flexiv-dummy-peer-archive-") as temporary:
            work_dir = Path(temporary)
            run(["ar", "x", str(output), DEFAULT_NODE_MEMBER], work_dir)
            member = work_dir / DEFAULT_NODE_MEMBER
            contents = member.read_bytes()
            count = contents.count(MUTATION_SIGNATURE)
            if count != 1:
                raise PreparationError(
                    "unexpected DefaultNode mutation signature count: "
                    f"expected 1, found {count}"
                )
            replacement = bytearray(MUTATION_SIGNATURE)
            replacement[
                MUTATION_IMMEDIATE_OFFSET:MUTATION_IMMEDIATE_OFFSET + 4
            ] = struct.pack("<I", mutation_tries)
            member.write_bytes(contents.replace(MUTATION_SIGNATURE, replacement))
            run(["ar", "r", str(output), DEFAULT_NODE_MEMBER], work_dir)
            run(["ranlib", str(output)], work_dir)
    except Exception:
        output.unlink(missing_ok=True)
        raise

    if sha256(source) != source_digest:
        output.unlink(missing_ok=True)
        raise PreparationError("source Sim Plugin archive changed during preparation")
    output_digest = sha256(output)
    if output_digest == source_digest:
        output.unlink(missing_ok=True)
        raise PreparationError("private dummy-peer archive was not changed")
    return source_digest, output_digest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mutation-tries", type=int, default=4096)
    args = parser.parse_args()
    try:
        source_digest, output_digest = prepare(
            args.source, args.output, args.mutation_tries
        )
    except (OSError, PreparationError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(
        f"prepared {args.output.resolve()}: mutation_tries={args.mutation_tries} "
        f"source_sha256={source_digest} output_sha256={output_digest}",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
