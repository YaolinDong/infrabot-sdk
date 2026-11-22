#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Stage a verified, source-independent Linux maintenance kit. No board or publishing I/O."""
import argparse
import hashlib
import os
from pathlib import Path
import platform
import shutil
import tarfile
import tempfile

from netboot_swd import default_package
from flash_board import SDK, host_tool, load_key, verify_package


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--public-key',type=Path,default=SDK/'firmware/release.pub',
                   help='trusted release public key to include in this kit')
    p.add_argument('--package',type=Path,help='signed release IFW to stage')
    args=p.parse_args()
    if platform.system()!='Linux':p.error('Build this kit on the target Linux architecture')
    raw_key=load_key(args.public_key)
    # Explicit allowlist; never tar the whole SDK, build tree or firmware directory.
    files={name: SDK/name for name in (
        'flash-net','flash-swd','tools/update/flash_board.py','tools/update/netboot_swd.py',
        'tools/update/h743_dual_bank.cfg','tools/update/firmware_signing.py',
        'LICENSE','THIRD_PARTY.md','LICENSES/BSD-3-Clause-pybind11.txt','LICENSES/GPL-3.0.txt',
        'LICENSES/LGPL-3.0.txt','third_party/infrabot_ik/README.md','third_party/infrabot_ik/LICENSE')}
    files['firmware/infrabot_firmware.ifw']=args.package or default_package(SDK/'firmware')
    files['firmware/release.pub']=args.public_key
    files.update({'README.md': SDK/'docs/update-kit.md','bin/infrabot_update':host_tool(rebuild=True)})
    out=SDK/'build/releases';out.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=out) as temp:
        stage=Path(temp)/'infrabot-update';stage.mkdir()
        hashes=[]
        for name,src in files.items():
            target=stage/name;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(src,target)
            target.chmod(0o755 if name in ('flash-net','flash-swd','bin/infrabot_update') else 0o644)
            hashes.append(hashlib.sha256(target.read_bytes()).hexdigest()+'  '+name)
        image=stage/'firmware/infrabot_firmware.ifw';verify_package(image,raw_key)
        digest=hashlib.sha256(image.read_bytes()).hexdigest()
        (stage/'SHA256SUMS').write_text('\n'.join(hashes)+'\n')
        archive=Path(temp)/'kit.tar.gz'
        with tarfile.open(archive,'w:gz') as tar:
            for name in [*files,'SHA256SUMS']:
                tar.add(stage/name,arcname='infrabot-update/'+name,recursive=False)
        result=out/f'infrabot-update-linux-{platform.machine()}-{digest[:12]}.tar.gz'
        os.replace(archive,result)
        print('交付包：',result)
        print('交付包 SHA256：',hashlib.sha256(result.read_bytes()).hexdigest())


if __name__=='__main__':main()
