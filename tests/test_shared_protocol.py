# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Check the canonical firmware header without relying on a developer checkout."""
import argparse
import os
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("firmware", nargs="?", default=os.getenv("INFRABOT_FIRMWARE_DIR"))
args = parser.parse_args()
if not args.firmware:
    print("SKIP: set INFRABOT_FIRMWARE_DIR to compare the firmware and SDK protocol headers")
    raise SystemExit(77)

canonical = Path(args.firmware) / "protocol/infrabot/protocol.h"
copy = Path(__file__).resolve().parents[1] / "include/infrabot/protocol.h"
if not canonical.is_file():
    raise SystemExit(f"Missing canonical header: {canonical}")
if canonical.read_bytes() != copy.read_bytes():
    raise SystemExit("Protocol headers differ; sync and review the SDK copy with tools/sync_protocol.sh")
print("Firmware and SDK protocol headers match byte for byte")
