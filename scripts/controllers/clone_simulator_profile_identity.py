#!/usr/bin/env python3
"""Clone an Elements simulator profile with a new synthetic arm identity."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil
import sys


INTERNAL_SERIAL_PATTERN = re.compile(r"A02LS-05-M1-[0-9A-Za-z]{6}")


class CloneError(RuntimeError):
    """Raised when a profile cannot be cloned safely."""


def replace_exact(path: Path, source: str, target: str, expected_count: int) -> None:
    text = path.read_text(encoding="utf-8")
    actual_count = text.count(source)
    if actual_count != expected_count:
        raise CloneError(
            f"expected {expected_count} occurrence(s) of {source!r} in {path}, "
            f"found {actual_count}"
        )
    path.write_text(text.replace(source, target), encoding="utf-8")


def scan_for_identity(profile: Path, identity: str) -> list[Path]:
    encoded = identity.encode("utf-8")
    matches: list[Path] = []
    for path in profile.rglob("*"):
        if path.is_file() and encoded in path.read_bytes():
            matches.append(path)
    return matches


def clone_profile(
    source_profile: Path,
    output_profile: Path,
    source_internal_serial: str,
    target_internal_serial: str,
) -> None:
    if not INTERNAL_SERIAL_PATTERN.fullmatch(source_internal_serial):
        raise CloneError(f"invalid source Rizon 4s internal serial: {source_internal_serial}")
    if not INTERNAL_SERIAL_PATTERN.fullmatch(target_internal_serial):
        raise CloneError(f"invalid target Rizon 4s internal serial: {target_internal_serial}")
    if source_internal_serial == target_internal_serial:
        raise CloneError("source and target identities must differ")
    if not source_profile.is_dir():
        raise CloneError(f"source profile not found: {source_profile}")
    if output_profile.exists():
        raise CloneError(f"output profile already exists: {output_profile}")

    source_normalized = source_internal_serial.replace("-", "_")
    target_normalized = target_internal_serial.replace("-", "_")
    source_identity_dirs = (
        source_profile / source_internal_serial,
        source_profile / "user_data" / "settings" / source_internal_serial,
    )
    for path in source_identity_dirs:
        if not path.is_dir():
            raise CloneError(f"source identity directory not found: {path}")

    shutil.copytree(source_profile, output_profile, copy_function=shutil.copy2)

    output_root_identity = output_profile / source_internal_serial
    output_settings_identity = (
        output_profile / "user_data" / "settings" / source_internal_serial
    )
    target_root_identity = output_profile / target_internal_serial
    target_settings_identity = (
        output_profile / "user_data" / "settings" / target_internal_serial
    )
    output_root_identity.rename(target_root_identity)
    output_settings_identity.rename(target_settings_identity)

    replace_exact(
        output_profile / "user_data" / ".user_data.json",
        source_internal_serial,
        target_internal_serial,
        1,
    )
    replace_exact(
        output_profile / "user_data" / "settings" / "generated_robot_A02LS-M1.json",
        source_internal_serial,
        target_internal_serial,
        1,
    )
    replace_exact(
        target_root_identity / "arm_driver_param.xml",
        source_normalized,
        target_normalized,
        2,
    )
    replace_exact(
        target_settings_identity / "arm_driver_param.xml",
        source_normalized,
        target_normalized,
        2,
    )

    stale_paths = set()
    for identity in (source_internal_serial, source_normalized):
        stale_paths.update(scan_for_identity(output_profile, identity))
    if stale_paths:
        rendered = ", ".join(str(path) for path in sorted(stale_paths))
        raise CloneError(f"source identity remains in cloned profile: {rendered}")

    if (output_profile / source_internal_serial).exists():
        raise CloneError("source identity directory remains at profile root")
    if (output_profile / "user_data" / "settings" / source_internal_serial).exists():
        raise CloneError("source identity directory remains under user_data/settings")

    suffix = target_internal_serial.rsplit("-", 1)[1]
    print(
        f"cloned {source_profile} -> {output_profile}: "
        f"internal={target_internal_serial} public=Rizon4s-{suffix}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-profile", required=True, type=Path)
    parser.add_argument("--output-profile", required=True, type=Path)
    parser.add_argument("--source-internal-serial", required=True)
    parser.add_argument("--target-internal-serial", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        clone_profile(
            args.source_profile,
            args.output_profile,
            args.source_internal_serial,
            args.target_internal_serial,
        )
    except (CloneError, OSError, UnicodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
