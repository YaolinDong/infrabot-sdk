# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Ed25519 manifests (RFC 8032), using the system OpenSSL 3 CLI."""
from pathlib import Path
import os
import subprocess
import tempfile

SPKI = bytes.fromhex('302a300506032b6570032100')

def verify(public, manifest, signature):
    if len(public)!=32 or len(signature)!=64:return False
    with tempfile.TemporaryDirectory() as d:
        d=Path(d);(d/'public').write_bytes(SPKI+public);(d/'manifest').write_bytes(manifest);(d/'sig').write_bytes(signature)
        return subprocess.run(['openssl','pkeyutl','-verify','-rawin','-pubin','-keyform','DER','-inkey',str(d/'public'),
            '-in',str(d/'manifest'),'-sigfile',str(d/'sig')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
