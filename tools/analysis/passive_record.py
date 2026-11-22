#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Register one telemetry destination and record DMTL v2/v3; no command socket."""
import argparse
import csv
import ipaddress
import json
from pathlib import Path
import socket
import struct
import time

HELLO = b"infrabot_passive"
HEADER = ("host_time_s,mcu_time_s,sequence,lost,status,mode,profile,enabled_mask,"
          "command_active,command_applied_sequence").split(",")
for prefix in ("q", "dq", "tau", "state", "temp_mos", "q_cmd", "dq_cmd", "tau_cmd", "temp_coil", "flags"):
    HEADER.extend(f"{prefix}{i}" for i in range(1, 9))
HEADER.extend("stop_reason,stop_phase,stop_joint,stop_detail,mcu_cmd_rx,mcu_cmd_invalid,mcu_cmd_interval_max_us,mcu_control_missed".split(","))

def decode(data, host_time, previous=None):
    if len(data) not in (180, 348):
        return None
    magic, version, count, seq, _, timestamp = struct.unpack_from("<IHHIIQ", data)
    if magic != 0x444D544C or count != 8 or (version, len(data)) not in ((2,180),(3,348)):
        return None
    gap = ((seq-previous) & 0xffffffff) if previous is not None else 1
    if gap == 0 or gap >= 0x80000000:
        return None
    motors = [struct.unpack_from("<iiiBBH", data, 24+16*i) for i in range(8)]
    _, applied, active, status, enabled, mode = struct.unpack_from("<IIBBBB", data, 152)
    row = [f"{host_time:.6f}", f"{timestamp/1e6:.6f}", seq, gap-1, (status>>2)&63, mode&15, (mode>>4)&3, enabled, active, applied]
    for field in range(5):
        row.extend(v[field]/1000 if field<3 else v[field] for v in motors)
    if version == 3:
        commands = [struct.unpack_from("<iiiBBBB", data, 180+16*i) for i in range(8)]
        for field in range(5):
            row.extend(v[field]/1000 if field<3 else v[field] for v in commands)
        safety = struct.unpack_from("<4B", data, 328)
        link = struct.unpack_from("<8H", data, 308)
        row.extend((*safety,link[0],link[1],link[2],link[4]))
    else:
        row.extend([""]*48)
    assert len(row) == len(HEADER)
    return seq,row

def record(board, output, seconds=60, port=5000, bind="0.0.0.0"):
    # Never instantiate the SDK Robot: it also owns a command/heartbeat path.
    peer = (str(ipaddress.IPv4Address(board)), port)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = invalid = lost = 0
    previous = None
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock, output.open("x",newline="") as stream:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
        sock.bind((bind,0))
        sock.settimeout(.1)
        # Sole transmit operation; on the firmware this selects only the DMTL destination.
        sock.sendto(HELLO,peer)
        writer = csv.writer(stream)
        writer.writerow(HEADER)
        deadline = time.monotonic()+seconds
        try:
            while time.monotonic()<deadline:
                try:
                    packet,address = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                if address != peer:
                    invalid += 1
                    continue
                sample = decode(packet,time.monotonic(),previous)
                if sample is None:
                    invalid += 1
                    continue
                previous,row = sample
                rows += 1
                lost += row[3]
                writer.writerow(row)
        except KeyboardInterrupt:
            pass
    summary = dict(rows=rows,lost=lost,invalid=invalid,registration_packets=1,command_packets=0,
                   note="One telemetry destination; other GUI/SDK receivers must be closed.")
    output.with_suffix(".meta.json").write_text(json.dumps(summary,indent=2)+"\n")
    return summary

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("board",nargs="?",default="192.168.0.50")
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--seconds",type=float,default=60)
    a = p.parse_args()
    if a.seconds<=0:p.error("--seconds must be positive")
    print("单目的遥测将切换到本工具；先关闭其他 GUI/SDK 接收端，再用 LCD 操作。仅发送一次遥测注册。",flush=True)
    print(json.dumps(record(a.board,a.out,a.seconds),ensure_ascii=False))

if __name__ == "__main__":
    main()
