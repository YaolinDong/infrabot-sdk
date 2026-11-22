#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Sync the SDK copy. A major change requires an explicit new major and SDK compatibility work.
set -eu
root=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
firmware=${1:-${INFRABOT_FIRMWARE_DIR:-}}
[ -n "$firmware" ] || { echo "usage: $0 FIRMWARE_DIR [EXPECTED_NEW_MAJOR]" >&2; exit 2; }
src="$firmware/protocol/infrabot/protocol.h"
dst="$root/include/infrabot/protocol.h"
major() { sed -n 's/^#define INFRABOT_PROTOCOL_MAJOR \([0-9][0-9]*\)U$/\1/p' "$1"; }
new=$(major "$src")
old=$(major "$dst")
[ -n "$new" ] || { echo "Missing protocol major in $src" >&2; exit 1; }
if [ "$new" != "${old:-1}" ] && [ "${2:-}" != "$new" ]; then
    echo "Protocol major changed from ${old:-1} to $new. Review SDK compatibility and pass $new to sync." >&2
    exit 1
fi
if ! cmp -s "$src" "$dst"; then
    tmp=$(mktemp "$dst.XXXXXX")
    trap 'rm -f "$tmp"' EXIT
    cat "$src" > "$tmp"
    chmod 644 "$tmp"
    mv "$tmp" "$dst"
fi
echo "SDK header matches firmware (protocol major $new)"
