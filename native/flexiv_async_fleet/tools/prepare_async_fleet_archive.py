#!/usr/bin/env python3
"""Create a private Sim Plugin archive routed through a shared DDS participant."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


DEFAULT_NODE_MEMBER = "default_node.cc.o"
USER_NODE_MEMBER = "user_node.cpp.o"
CREATE_PARTICIPANT_SYMBOL = (
    "_ZN8eprosima7fastdds3dds24DomainParticipantFactory18create_participant"
    "EjRKNS1_20DomainParticipantQosEPNS1_25DomainParticipantListenerERKNS1_10StatusMaskE"
)
DELETE_PARTICIPANT_SYMBOL = (
    "_ZN8eprosima7fastdds3dds24DomainParticipantFactory18delete_participant"
    "EPNS1_17DomainParticipantE"
)
SHARED_CREATE_SYMBOL = "flexiv_async_create_shared_participant"
SHARED_DELETE_SYMBOL = "flexiv_async_delete_shared_participant"
CREATE_DATAREADER_SYMBOL = (
    "_ZN8eprosima7fastdds3dds10Subscriber17create_datareader"
    "EPNS1_16TopicDescriptionERKNS1_13DataReaderQosEPNS1_18DataReaderListener"
    "ERKNS1_10StatusMaskE"
)
CREATE_DATAWRITER_SYMBOL = (
    "_ZN8eprosima7fastdds3dds9Publisher17create_datawriter"
    "EPNS1_5TopicERKNS1_13DataWriterQosEPNS1_18DataWriterListener"
    "ERKNS1_10StatusMaskE"
)
UNIQUE_READER_SYMBOL = "flexiv_async_create_unique_datareader"
UNIQUE_WRITER_SYMBOL = "flexiv_async_create_unique_datawriter"


class PreparationError(RuntimeError):
    """Raised when the supported DefaultNode patch cannot be applied safely."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _run_archive_tool(command: list[str], cwd: Path) -> None:
    try:
        subprocess.run(
            command,
            cwd=cwd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError as error:
        raise PreparationError(f"required archive tool is unavailable: {command[0]}") from error
    except subprocess.CalledProcessError as error:
        details = error.stderr.decode("utf-8", errors="replace").strip()
        raise PreparationError(
            f"archive command failed [{command[0]}]: {details or error.returncode}"
        ) from error


def _symbol_table(path: Path, cwd: Path) -> str:
    try:
        result = subprocess.run(
            ["nm", str(path)],
            cwd=cwd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError as error:
        raise PreparationError("required archive tool is unavailable: nm") from error
    except subprocess.CalledProcessError as error:
        raise PreparationError(f"cannot inspect {path.name}: {error.stderr.strip()}") from error
    return result.stdout


def _redirect_symbols(
    path: Path, cwd: Path, redirects: tuple[tuple[str, str], ...]
) -> None:
    symbols = _symbol_table(path, cwd)
    for original, replacement in redirects:
        if f" U {original}\n" not in symbols:
            raise PreparationError(
                f"unsupported DefaultNode object: missing reference to {original}"
            )
        if replacement in symbols:
            raise PreparationError(
                f"DefaultNode object is already redirected to {replacement}"
            )
    command = ["objcopy"]
    for original, replacement in redirects:
        command.extend(["--redefine-sym", f"{original}={replacement}"])
    command.append(str(path))
    _run_archive_tool(command, cwd)


def _verify_patched_member(
    archive: Path,
    member_name: str,
    redirects: tuple[tuple[str, str], ...],
) -> None:
    with tempfile.TemporaryDirectory(prefix="flexiv-async-verify-") as temporary:
        work_dir = Path(temporary)
        _run_archive_tool(["ar", "x", str(archive), member_name], work_dir)
        member = work_dir / member_name
        if not member.is_file():
            raise PreparationError(f"archive member was not extracted: {member_name}")
        symbols = _symbol_table(member, work_dir)
        for original, replacement in redirects:
            if f" U {original}\n" in symbols:
                raise PreparationError(
                    f"private archive still references direct DDS symbol {original}"
                )
            if f" U {replacement}\n" not in symbols:
                raise PreparationError(
                    f"private archive does not reference shared shim {replacement}"
                )


def prepare(source: Path, output: Path) -> tuple[str, str]:
    source = source.resolve()
    output = output.resolve()
    if not source.is_file():
        raise PreparationError(f"Sim Plugin archive not found: {source}")
    if source == output:
        raise PreparationError("the installed Sim Plugin archive must remain untouched")

    source_digest = sha256(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, output)
    try:
        with tempfile.TemporaryDirectory(prefix="flexiv-async-patch-") as temporary:
            work_dir = Path(temporary)
            redirects_by_member = {
                DEFAULT_NODE_MEMBER: (
                    (CREATE_PARTICIPANT_SYMBOL, SHARED_CREATE_SYMBOL),
                    (DELETE_PARTICIPANT_SYMBOL, SHARED_DELETE_SYMBOL),
                ),
                USER_NODE_MEMBER: (
                    (CREATE_DATAREADER_SYMBOL, UNIQUE_READER_SYMBOL),
                    (CREATE_DATAWRITER_SYMBOL, UNIQUE_WRITER_SYMBOL),
                ),
            }
            for member_name, redirects in redirects_by_member.items():
                _run_archive_tool(["ar", "x", str(output), member_name], work_dir)
                member = work_dir / member_name
                if not member.is_file():
                    raise PreparationError(
                        f"Sim Plugin archive has no {member_name} member"
                    )
                _redirect_symbols(member, work_dir, redirects)
                _run_archive_tool(["ar", "r", str(output), member_name], work_dir)
            _run_archive_tool(["ranlib", str(output)], work_dir)
        for member_name, redirects in redirects_by_member.items():
            _verify_patched_member(output, member_name, redirects)
    except Exception:
        output.unlink(missing_ok=True)
        raise

    if sha256(source) != source_digest:
        output.unlink(missing_ok=True)
        raise PreparationError("source Sim Plugin archive changed during preparation")
    output_digest = sha256(output)
    if output_digest == source_digest:
        output.unlink(missing_ok=True)
        raise PreparationError("private Sim Plugin archive was not changed")
    return source_digest, output_digest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        source_digest, output_digest = prepare(args.source, args.output)
    except (OSError, PreparationError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(
        f"prepared {args.output.resolve()}: dds_shared_participant=true "
        f"source_sha256={source_digest} output_sha256={output_digest}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
