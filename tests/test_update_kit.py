# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Offline delivery check; only the read-only probe uses UDP, exclusively on loopback."""
import argparse
import hashlib
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tarfile
import tempfile


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('archive',type=Path)
    args=parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        tmp=Path(tmp);home=tmp/'home';home.mkdir()
        root=tmp/'infrabot-update'
        expected={'flash-net','flash-swd','tools/update/flash_board.py','tools/update/netboot_swd.py','tools/update/h743_dual_bank.cfg','tools/update/firmware_signing.py','firmware/release.pub',
                  'bin/infrabot_update','firmware/infrabot_firmware.ifw','README.md','SHA256SUMS','LICENSE','THIRD_PARTY.md','LICENSES/BSD-3-Clause-pybind11.txt',
                  'LICENSES/GPL-3.0.txt','LICENSES/LGPL-3.0.txt','third_party/infrabot_ik/README.md','third_party/infrabot_ik/LICENSE'}
        with tarfile.open(args.archive) as archive:
            members=archive.getmembers()
            assert len(members)==len(expected)
            assert {m.name for m in members}=={'infrabot-update/'+p for p in expected}
            for m in members:
                assert m.isfile();p=tmp/m.name;p.parent.mkdir(parents=True,exist_ok=True)
                p.write_bytes(archive.extractfile(m).read());p.chmod(m.mode)
        for line in (root/'SHA256SUMS').read_text().splitlines():
            digest,path=line.split('  ',1);assert digest==hashlib.sha256((root/path).read_bytes()).hexdigest()
        # No checkout, build system, compiler or hardware utilities in PATH.
        path=tmp/'path';path.mkdir();(path/'python3').symlink_to(sys.executable);(path/'dirname').symlink_to('/usr/bin/dirname')
        (path/'openssl').symlink_to('/usr/bin/openssl')
        env={**os.environ,'HOME':str(home),'PATH':str(path)}
        def run(name,*args,ok=True):
            r=subprocess.run([root/name,*map(str,args)],cwd=tmp,env=env,capture_output=True,text=True,timeout=10)
            assert (r.returncode==0)==ok,(r.stdout,r.stderr)
            return r
        for tool in ('flash-swd','flash-net'):run(tool,'--prepare')
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
            sock.bind(('127.0.0.1',0));sock.settimeout(5)
            proc=subprocess.Popen([root/'flash-net','127.0.0.1','--probe','--port',
                                   str(sock.getsockname()[1])],cwd=tmp,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            try:
                msg,peer=sock.recvfrom(256)
                magic,ver,op,_,_,_,seq,_=struct.unpack('<4sBBBBQII',msg)
                assert (magic,ver,op)==(b'IFU2',2,1)
                reply=struct.pack('<4sBBBBQII',magic,ver,129,0,1,987,seq,0x08020400)+bytes(32)
                sock.sendto(reply,peer)
                stdout,stderr=proc.communicate(timeout=5)
                assert proc.returncode==0 and b'confirmed A' in stdout,(stdout,stderr)
            finally:
                if proc.poll() is None:proc.kill();proc.wait()
        image=root/'firmware/infrabot_firmware.ifw'
        # Existing installations can keep the old basename for one version.
        legacy=image.with_name("fdcantest.ifw")
        image.rename(legacy)
        run('flash-swd','--prepare')
        run('flash-net','--prepare')
        image.write_bytes(legacy.read_bytes())
        with image.open('r+b') as f:f.seek(-1,2);v=f.read(1);f.seek(-1,2);f.write(bytes([v[0]^1]))
        run('flash-net','--prepare',ok=False)
    print('Delivery kit: allowlist, checksums, source-free SWD/net checks, packaged native probe, public-key-only and corrupted image checks passed.')
if __name__=='__main__':main()
