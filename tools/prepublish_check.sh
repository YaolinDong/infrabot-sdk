#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Scan the exact indexed release. In a development checkout --manifest-only
# audits only the release allowlist, without copying any files.
set -eu
cd "$(dirname "$0")/.."
exec python3 - "$@" <<'PY'
import argparse
import hashlib
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import sys

parser = argparse.ArgumentParser(description="Verify a tracked source release")
scope = parser.add_mutually_exclusive_group()
scope.add_argument("--manifest-only", action="store_true", help="allow private tracked files outside the manifest")
scope.add_argument("--history", action="store_true", help="also scan snapshots and metadata reachable from all refs")
args = parser.parse_args()

def read_private_rules():
    location = os.environ.get("INFRABOT_PRIVATE_DENYLIST")
    if not location:
        if os.environ.get("CI", "").lower() in ("1", "true"):
            print("SKIP: private denylist unavailable in CI; all other release checks remain active", flush=True)
            return ()
        sys.exit("Set INFRABOT_PRIVATE_DENYLIST to an external mode-0600 file (one literal per line)")
    path = Path(location).expanduser()
    if path.resolve().is_relative_to(Path.cwd()) or path.is_symlink():
        sys.exit("The private denylist must be a regular file outside this checkout")
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        sys.exit("Cannot open the external private denylist")
    with os.fdopen(fd, "rb") as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o600:
            sys.exit("The private denylist must be a mode-0600 regular file")
        values = tuple(line for line in stream.read().splitlines() if line)
    if not values:
        sys.exit("The configured private denylist is empty")
    return values

private_rules = read_private_rules()

def git(*argv):
    return subprocess.check_output(["git", *argv])

manifest = Path("tools/public_manifest.txt")
paths = set()
components = {}
large = {}
for line in manifest.read_text().splitlines():
    if line.startswith("# license\t"):
        _, component, license_path, notice = line.split("\t")
        components[component] = (license_path, notice)
    elif line.startswith("# large\t"):
        _, size, digest, name = line.split("\t")
        large[name] = (int(size), digest)
    elif line and not line.startswith("#"):
        path = PurePosixPath(line)
        if path.is_absolute() or ".." in path.parts or path.as_posix() != line or line in paths:
            sys.exit(f"Invalid or duplicate release path: {line}")
        paths.add(line)

index = {}
for row in git("ls-files", "--stage", "-z").split(b"\0"):
    if not row:
        continue
    meta, name = row.split(b"\t", 1)
    mode, oid, stage = meta.decode().split()
    if stage != "0":
        sys.exit("Resolve the index before scanning")
    index[os.fsdecode(name)] = mode, oid

missing = paths - index.keys()
extra = index.keys() - paths
if missing or (extra and not args.manifest_only):
    sys.exit("Manifest mismatch: " + ", ".join(sorted(missing | (set() if args.manifest_only else extra))))
blocked = {".git", ".vscode", ".settings", "__pycache__", ".log"}
key_marker = re.compile(rb"BEGIN[^\r\n]*PRIVATE" + rb" KEY", re.I)

# Compare simple adjacent and plus-joined source literals as well as the raw bytes.
literal_join = re.compile(rb"""["'](?:\s*\+\s*|\s+)[bBrRuU]{0,2}["']""")

def forbidden(data):
    if key_marker.search(data) or any(rule in data for rule in private_rules):
        return True
    if private_rules and b"\x00" not in data:
        joined = literal_join.sub(b"", data)
        return any(rule in joined for rule in private_rules)
    return False

def excluded(name):
    p = PurePosixPath(name)
    return (blocked.intersection(p.parts)
            or any(x.startswith("build-") or x == "build" or x.endswith(".egg-info") for x in p.parts)
            or name.startswith(("docs/history/", "tools/legacy/"))
            or p.suffix.lower() in {".ifw", ".so", ".pyc", ".pem", ".key", ".pfx", ".p12"}
            or ".so." in p.name)

errors = []
notice_text = Path("THIRD_PARTY.md").read_text()
if not components or not {"LICENSE", "THIRD_PARTY.md", str(manifest)} <= paths:
    sys.exit("Missing release license inventory")
for component, (license_path, notice) in components.items():
    if license_path not in paths or notice not in notice_text or not any(n == component or n.startswith(component + "/") for n in paths):
        errors.append("Missing component, license or attribution: " + component)
licensed_files = {p for p, _ in components.values()}
for name in sorted(paths):
    if (name.startswith(("third_party/", "bootloader/vendor/"))
        and name not in {"third_party/README.md", "third_party/PATCHES.md"} | licensed_files
        and not any(name == prefix or name.startswith(prefix + "/") for prefix in components)):
        errors.append("Unregistered third-party component: " + name)
    p = Path(name)
    mode, oid = index[name]
    if excluded(name):
        errors.append("Excluded release path: " + name)
    if mode == "120000" and p.is_symlink():
        target = os.readlink(p)
        body = os.fsencode(target)
        dest = os.path.normpath(os.path.join(str(p.parent), target))
        if os.path.isabs(target) or dest not in paths or index[dest][0] not in ("100644", "100755"):
            errors.append("Symlink must resolve to a listed regular file: " + name)
    elif mode in ("100644", "100755") and p.is_file() and not p.is_symlink():
        body = p.read_bytes()
    else:
        errors.append("Missing or unsupported release entry: " + name)
        continue
    if forbidden(body) or forbidden(os.fsencode(name)):
        errors.append("Personal path, identity or private-key marker: " + name)
    if len(body) > 1_000_000 or name in large:
        if large.get(name) != (len(body), hashlib.sha256(body).hexdigest()):
            errors.append("Oversize file or changed size exception: " + name)
if set(large) - paths:
    errors.append("Size exception outside manifest")
# Detect a different staged version even when the working files passed.
if subprocess.run(["git", "diff", "--quiet", "--", *sorted(paths)]).returncode:
    errors.append("Tracked worktree differs from the index; scan the version to be released")
if errors:
    sys.exit("\n".join(errors))
if args.history:
    # Each tree can name the same blob differently. Check every pathname, but
    # inspect each distinct blob, commit, tree and annotated tag payload once.
    seen_trees = set()
    for revision in git("rev-list", "--all").splitlines():
        tree = git("rev-parse", revision + b"^{tree}").strip()
        if tree in seen_trees:
            continue
        seen_trees.add(tree)
        for entry in git("ls-tree", "-rz", tree).split(b"\0"):
            if not entry:
                continue
            meta, raw_name = entry.split(b"\t", 1)
            if excluded(os.fsdecode(raw_name)) or forbidden(raw_name):
                sys.exit("Excluded historical path in commit " + revision.decode())
    if forbidden(git("for-each-ref", "--format=%(refname)")):
        sys.exit("Private marker in a Git ref name")
    objects = set(git("rev-list", "--objects", "--no-object-names", "--all").splitlines())
    with subprocess.Popen(["git", "cat-file", "--batch"], stdin=subprocess.PIPE, stdout=subprocess.PIPE) as cat:
        for oid in sorted(objects):
            cat.stdin.write(oid + b"\n")
            cat.stdin.flush()
            header = cat.stdout.readline().split()
            if len(header) != 3 or header[0] != oid:
                sys.exit("Could not inspect historical object " + oid.decode())
            body = cat.stdout.read(int(header[2]))
            if cat.stdout.read(1) != b"\n" or forbidden(body):
                sys.exit("Invalid object or forbidden historical content: " + oid.decode())
        cat.stdin.close()
    if cat.returncode:
        sys.exit("Git history read failed")
    print(f"History scan passed: {len(seen_trees)} snapshots, {len(objects)} reachable Git objects")
print(f"Release scan passed: {len(paths)} tracked paths, {len(components)} licensed components, {len(large)} verified size exceptions.")
PY
