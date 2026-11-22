#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Install/upgrade an H743 via fireDAP alone (no Ethernet). Linux/OpenOCD."""
import argparse
from datetime import datetime, timezone
import hashlib
from firmware_signing import verify
from pathlib import Path
import struct
import subprocess
import sys

FLASH, A, B, CFG = 0x08000000, 0x08020000, 0x08100000, 0x081e0000
SLOT, META, SECTOR = 0xe0000, 1024, 0x20000
READY, GOOD = 0x59444152, 0x444f4f47
CAPTURE = '''init
reset halt
if {([mrw 0x5c001000] & 0xfff) != 0x450 || [mrh 0x1ff1e880] != 2048} {error "Unexpected STM32 device or size"}
if {([mrw 0x5200201c] & 0x80000000) != 0} {error "Swapped flash banks are unsupported"}
flash probe 0
flash probe 1
dump_image uid.bin 0x1ff1e800 12
dump_image before.bin 0x08000000 0x200000
verify_image before.bin 0x08000000 bin
'''
RECHECK = '''init
halt
flash probe 0
flash probe 1
verify_image uid.bin 0x1ff1e800 bin
verify_image before.bin 0x08000000 bin
'''
WAIT = '''
proc wait_app {status vector generation} {
    for {set i 0} {$i < 600} {incr i} {
        if {![catch {
            set st [mrw 0x20000010]
            set v [mrw 0x20000018]
            set g [mrw 0x2000001c]
        }] && $st == $status && $v == $vector && $g == $generation} {return}
        sleep 100
    }
    error "No matching application health indication; update NOT confirmed. Check power, SWD and boot log."
}
'''


def default_package(directory):
    """Prefer the current release name; accept the previous SDK layout for one version."""
    current = directory / "infrabot_firmware.ifw"
    previous = directory / "fdcantest.ifw"
    if not current.exists() and previous.is_file():
        print(f"Using legacy package name {previous}; rename to {current.name}", file=sys.stderr)
        return previous
    return current


def require(condition, message):
    if not condition:
        raise ValueError(message)


def mark(kind, gen, base, tag):
    return struct.pack('<IIII16s', kind, gen, gen ^ 0xffffffff, base + META, tag[:16])


def valid_image(header, body, base, key):
    if len(header) != 64 or not 72 <= len(body) <= SLOT-META:
        return False
    magic, addr, length = struct.unpack_from('<8sII', header)
    sp, pc = struct.unpack_from('<II', body)
    return (magic == b'IFH743v2' and addr == base+META and length == len(body)
            and sp == 0x20020000 and pc & 1 and addr <= pc-1 < addr+length-64
            and header[32:]==hashlib.sha256(body[:-64]).digest() and verify(key,header,body[-64:]))


def unpack(package, key):
    require(len(package)>=16, 'Short IFW bundle')
    magic,n,m=struct.unpack_from('<8sII',package)
    require(magic==b'IFABv002' and len(package)==16+n+m,'Invalid A/B bundle')
    images={A:package[16:16+n], B:package[16+n:]}
    for base,blob in images.items():
        require(valid_image(blob[:64],blob[64:],base,key),'IFW authentication, address or vector check failed')
    require(images[A][16:32]==images[B][16:32],'Mixed releases')
    return images


def current(snapshot, key):
    require(len(snapshot)==0x200000,'Incomplete device backup')
    records=[]
    for base in (A,B):
        slot=snapshot[base-FLASH:base-FLASH+SLOT]
        length=struct.unpack_from('<I',slot,12)[0]
        tag=slot[32:64]
        gen=struct.unpack_from('<I',slot,68)[0]
        if (gen and slot[64:96]==mark(READY,gen,base,tag)
                and slot[128:160]==mark(GOOD,gen,base,tag)
                and slot[160:192]==b'\xff'*32
                and valid_image(slot[:64],slot[META:META+length],base,key)):
            records.append((gen,base,tag))
        elif slot[:8]==b'IFH743v2' and slot[160:192]==b'\xff'*32 and slot[128:132]==b'GOOD':
            # Unknown signing key/corrupt confirmed record: never assume it's disposable.
            raise ValueError('Cannot authenticate a confirmed slot. Check the publisher public key; nothing erased.')
    require(records,'No authenticated confirmed slot. Check key or use the verified SWD backup.')
    require(len(records)==1 or records[0][0]!=records[1][0],'Ambiguous slot generations')
    return max(records)


def plan(snapshot, images, factory, key, folder, migrate_v1=False):
    require(len(snapshot)==0x200000,'Incomplete flash snapshot')
    (folder/'config.bin').write_bytes(snapshot[CFG-FLASH:])
    if snapshot[0x400:0x404]!=b'FAG2':
        require(factory is not None,'Board has no A/B loader. First installation requires a trusted matching --factory image from the manufacturer.')
        legacy=snapshot[0x400:0x404]==b'FAG1'
        require(not legacy or migrate_v1,'v1 installed: explicit --migrate-v1 and trusted v2 --factory required')
        require(all(snapshot[b-FLASH:b-FLASH+8]!=b'IFH743v2' and
                    (legacy or snapshot[b-FLASH:b-FLASH+8]!=b'IFH743v1') for b in (A,B)),
                'Slot metadata present but loader missing; refusing automatic factory installation')
        # Same factory payload as the signed slot A in this exact build.
        old=current(factory.ljust(0x200000,b'\xff'),key)
        require(0x20408<=len(factory)<=0x100000 and factory[0x400:0x424]==b'FAG2'+key
                and old[:2]==(1,A) and factory[0x20000:0x20040]==images[A][:64]
                and factory[0x20400:]==images[A][64:], 'Factory and IFW mismatch')
        (folder/'factory.bin').write_bytes(factory)
        writing='''flash write_image erase factory.bin 0x08000000 bin
verify_image factory.bin 0x08000000 bin
flash erase_address 0x08100000 0x20000
'''
        base,gen,trial=A,1,False
    else:
        require(snapshot[0x404:0x424]==key,"Installed loader trusts another publisher")
        gen,active,tag=current(snapshot,key)
        if tag==images[active][32:64]:
            # Older factory releases may predate the SWD health mailbox. New
            # images produced by this tool include it; no erase is needed.
            return 'Already at requested image', RECHECK+WAIT+f'mww 0x20000010 0 4\nreset run\nwait_app 0x314e5753 {active+META:#x} {gen}\n'
        require(gen<0xffffffff,'Generation exhausted')
        gen+=1
        base=B if active==A else A
        trial=True
        image=images[base]
        (folder/'app.bin').write_bytes(image[64:])
        (folder/'manifest.bin').write_bytes(image[:64])
        (folder/'ready.bin').write_bytes(mark(READY,gen,base,image[32:64]))
        (folder/'active-slot.bin').write_bytes(snapshot[active-FLASH:active-FLASH+SLOT])
        (folder/'loader.bin').write_bytes(snapshot[:SECTOR])
        erase=((META+len(image)-64+SECTOR-1)//SECTOR)*SECTOR
        require(0<erase<=SLOT and base+erase<=CFG,'Invalid erase range')
        writing=f'''flash erase_address {base:#x} {erase:#x}
flash write_image app.bin {base+META:#x} bin
verify_image app.bin {base+META:#x} bin
flash write_image manifest.bin {base:#x} bin
verify_image manifest.bin {base:#x} bin
verify_image loader.bin 0x08000000 bin
verify_image active-slot.bin {active:#x} bin
flash write_image ready.bin {base+64:#x} bin
verify_image ready.bin {base+64:#x} bin
'''
    script=RECHECK+WAIT+writing+'verify_image config.bin 0x081e0000 bin\n'
    # Clear old debug handshake before reset; never set boot reason or GOOD from SWD.
    script+='mww 0x20000010 0 4\nreset run\n'
    if trial:
        script+=f'wait_app 0x31545753 {base+META:#x} {gen}\nmww 0x20000014 0x31435753\n'
    script+=f'wait_app 0x314e5753 {base+META:#x} {gen}\n'
    # Verify durable confirmation for this exact authenticated image.
    words=struct.unpack('<8I',mark(GOOD,gen,base,images[base][32:64]))
    for i,word in enumerate(words):
        script+=f'if {{[mrw {base+128+i*4:#x}] != {word:#x}}} {{error "Confirmed marker mismatch"}}\n'
    return ('SWD update to '+('A' if base==A else 'B') if trial else 'First SWD installation'),script


def openocd(folder,name,body):
    script=folder/(name+'.tcl')
    script.write_text('if {[catch {\n'+body+'} problem]} {echo $problem; shutdown error} else {shutdown}\n')
    subprocess.run(['openocd','-f','interface/cmsis-dap.cfg','-c','transport select swd',
                    '-c','adapter speed 4000','-f',str(Path(__file__).resolve().with_name('h743_dual_bank.cfg')),'-f',script.name],
                   cwd=folder,check=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--build',type=Path)
    group.add_argument('--package',type=Path,help='Released A/B .ifw; provisioned boards only need this file')
    p.add_argument('--factory',type=Path,help='Trusted local factory build; never accepted via the network')
    p.add_argument('--public-key',type=Path,required=True)
    p.add_argument('--migrate-v1',action='store_true',help='explicit one-time loader replacement from trusted factory')
    p.add_argument('--backup-dir',type=Path,required=True)
    p.add_argument('--motor-power-off',action='store_true',required=True)
    a=p.parse_args()
    key=bytes.fromhex(a.public_key.read_text().strip());require(len(key)==32,'Invalid key')
    if a.migrate_v1 and a.factory is None and a.build is None:p.error('--migrate-v1 requires a trusted factory build')
    package=a.package if a.package else default_package(a.build)
    images=unpack(package.read_bytes(),key)
    factory_path=a.factory or (a.build/'factory-bank1.bin' if a.build else None)
    factory=factory_path.read_bytes() if factory_path else None
    root=a.backup_dir.resolve();root.mkdir(parents=True,exist_ok=True)
    folder=root/datetime.now(timezone.utc).strftime('swd-%Y%m%dT%H%M%S%fZ')
    folder.mkdir(mode=0o700)
    print('Verified backup will be saved in',folder,flush=True)
    capture=CAPTURE
    if factory is None:
        capture=capture.replace('reset halt',
            'if {[mrw 0x08000400] != 0x32474146} {error "First installation requires a trusted matching --factory image"}\nreset halt',1)
    openocd(folder,'capture',capture)
    method,commands=plan((folder/'before.bin').read_bytes(),images,factory,key,folder,a.migrate_v1)
    print(method,flush=True)
    # Durable backup before the first erase, especially for the shared-loader migration.
    (folder/'before.sha256').write_text(hashlib.sha256((folder/'before.bin').read_bytes()).hexdigest()+'\n')
    import os
    for item in [*folder.iterdir(), folder, root]:
        fd=os.open(item,os.O_RDONLY)
        try:os.fsync(fd)
        finally:os.close(fd)
    openocd(folder,'program',commands)
    print('SWD completed; keep before.bin private.')
if __name__=='__main__':
    try:main()
    except (OSError,ValueError,subprocess.CalledProcessError) as exc:
        sys.exit(f'SWD not confirmed: {exc}')
