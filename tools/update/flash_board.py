# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Shared CLI for the two root launchers. Never guesses a debugger/network target."""
import argparse
import ipaddress
import os
from pathlib import Path
import hashlib
import re
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
from netboot_swd import default_package

SDK=Path(__file__).resolve().parents[2]


def run(*cmd):
    subprocess.run(list(map(str,cmd)),check=True)


def configured(build, options):
    cache=build/'CMakeCache.txt'
    if not cache.exists():return False
    entries={}
    for line in cache.read_text().splitlines():
        if line.startswith(('#','//')) or '=' not in line:continue
        field,value=line.split('=',1)
        entries[field.split(':',1)[0]]=value
    return all(entries.get(k)==str(v) for k,v in options.items())


def load_key(path):
    value=bytes.fromhex(path.read_text().strip())
    if len(value)!=32:raise ValueError(f'不是 32 字节 Ed25519 公钥：{path}')
    return value


def host_tool(*, rebuild=False):
    # Delivery kit: use its prebuilt host binary without CMake or either checkout.
    binary=SDK/'bin/infrabot_update'
    if binary.is_file() and not rebuild:
        if not os.access(binary,os.X_OK):raise ValueError(f'升级程序不可执行：{binary}')
        return binary
    if not (SDK/'CMakeLists.txt').is_file():
        raise ValueError('缺少 bin/infrabot_update，请重新解压完整升级工具包')
    build=SDK/'build/flash-tools'
    if not configured(build,{'CMAKE_HOME_DIRECTORY':SDK,'INFRABOT_BUILD_PYTHON':'OFF','INFRABOT_BUILD_TESTS':'OFF'}):
        run('cmake','-S',SDK,'-B',build,'-DINFRABOT_BUILD_PYTHON=OFF','-DINFRABOT_BUILD_TESTS=OFF')
    run('cmake','--build',build,'--target','infrabot_update','--parallel','4')
    return build/'infrabot_update'


def verify_package(path,key):
    from netboot_swd import unpack
    if not path.is_file():raise ValueError(f'缺少交付固件：{path}；请从固件发布页下载签名包并用 --package 指定路径')
    data=path.read_bytes()
    unpack(data,key)
    print(f'交付固件：{path}  SHA256: {hashlib.sha256(data).hexdigest()}',flush=True)


RELEASES='https://github.com/YaolinDong/infrabot-firmware/releases'
ASSET='infrabot_firmware.ifw'
LIMIT=8<<20
TAG=re.compile(r'v[0-9]+\.[0-9]+\.[0-9]+')


def release_url(version,base=RELEASES):
    base=base.rstrip('/')
    return f'{base}/latest/download/{ASSET}' if version=='latest' else f'{base}/download/{version}/{ASSET}'


def download_release(version,key,base=RELEASES,cache=None):
    """Fetch a signed release IFW into firmware/cache/<version>/ and verify it before use."""
    if version!='latest' and not TAG.fullmatch(version):
        raise ValueError(f'版本号应形如 v0.1.2：{version}')
    cache=Path(cache or SDK/'firmware/cache')
    url=release_url(version,base)
    if version!='latest':
        cached=cache/version/ASSET
        if cached.is_file():
            verify_package(cached,key)
            print(f'使用已下载的 {version}：{cached}',flush=True)
            return cached
    print(f'下载固件：{url}',flush=True)
    try:
        with urllib.request.urlopen(url,timeout=60) as reply:
            final=reply.geturl()
            data=reply.read(LIMIT+1)
    except urllib.error.HTTPError as exc:
        raise ValueError(f'下载失败：HTTP {exc.code}（{url}）；确认该版本已发布且仓库公开') from None
    except (urllib.error.URLError,TimeoutError) as exc:
        raise ValueError(f'下载失败：{getattr(exc,"reason",exc)}（{url}）；可设置 https_proxy 或用 --url 指定镜像') from None
    if len(data)>LIMIT:raise ValueError(f'下载文件超过 {LIMIT>>20} MiB，不是固件包：{url}')
    if version=='latest':
        tag=next((part for part in final.split('/') if TAG.fullmatch(part)),None)
        if not tag:raise ValueError(f'无法从下载地址确定版本号：{final}')
        version=tag
    target=cache/version/ASSET
    target.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=target.parent,delete=False) as tmp:
        tmp.write(data);staged=Path(tmp.name)
    try:
        verify_package(staged,key)
        staged.replace(target)
    finally:
        staged.unlink(missing_ok=True)
    print(f'已下载并校验 {version}：{target}',flush=True)
    return target


def main():
    mode=sys.argv[1]
    p=argparse.ArgumentParser(prog='flash-'+mode,description=(
        '使用 fireDAP/SWD 首装或更新备用槽，验证和试启动均不需要网线。' if mode=='swd' else
        '使用网口升级备用槽，需已安装 A/B Bootloader。'))
    if mode=='net':
        p.add_argument('ip',nargs='?',default='192.168.0.50',type=ipaddress.IPv4Address)
        group=p.add_mutually_exclusive_group()
        p.add_argument('--port',type=int,default=5002,help='维护端口（默认 5002）')
        group.add_argument('--probe',action='store_true',help='只读查询板子与当前槽位')
    else:
        group=p.add_mutually_exclusive_group()
        p.add_argument('--factory',type=Path,help='仅首装时指定私有 factory-bank1.bin')
        p.add_argument('--backup-dir',type=Path,default=Path.home()/'infrabot-flash-backups')
    group.add_argument('--prepare',action='store_true',help='离线核对已打包固件签名，不接触板子')
    p.add_argument('--motor-power-off',action='store_true',help='已支撑、失能并断开电机动力，跳过交互输入')
    p.add_argument('--public-key',type=Path,default=SDK/'firmware/release.pub',help='可信发布者公钥（默认随 SDK 提供）')
    source=p.add_mutually_exclusive_group()
    source.add_argument('--package',type=Path,help='已生成的 A/B 固件包')
    source.add_argument('--download',nargs='?',const='latest',metavar='VERSION',
                        help='从 GitHub Release 下载固件包（默认最新版，或如 v0.1.1），校验签名后使用')
    p.add_argument('--url',default=RELEASES,help='发布页地址或镜像（与 --download 一起使用）')
    args=p.parse_args(sys.argv[2:]);args.mode=mode
    probe=getattr(args,'probe',False)
    if mode=='net' and not 1<=args.port<=65535:p.error('端口范围为 1..65535')
    if not (args.prepare or probe or args.motor_power_off):
        if not sys.stdin.isatty():raise ValueError('非交互烧录需显式指定 --motor-power-off')
        answer=input('请确认机械臂已支撑、失能并断开电机动力，控制板仍供电。继续烧录？[Y/n]（回车继续，no 取消）：').strip().casefold()
        if answer in ('n','no'):
            print('已取消，未烧录。')
            return
        if answer not in ('','y','yes'):
            raise ValueError('未识别的输入，已取消，未烧录。')
    os.umask(0o077)
    key=args.public_key.expanduser().resolve()
    public=load_key(key)
    if probe:
        run(host_tool(),'--probe',args.ip,key,args.port);return
    if args.download:
        package=download_release(args.download,public,args.url)
    else:
        package=(args.package or default_package(SDK/"firmware")).expanduser().resolve()
        verify_package(package,public)
    if args.prepare:
        print('固件包离线校验通过，没有连接或修改板子。');return
    if mode=='swd':
        cmd=[sys.executable,SDK/'tools/update/netboot_swd.py','--package',package,'--public-key',key,
             '--backup-dir',args.backup_dir.expanduser(),'--motor-power-off']
        if args.factory:cmd+=['--factory',args.factory.expanduser()]
        run(*cmd)
    else:
        run(host_tool(),args.ip,package,key,args.port)


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,EOFError,subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
