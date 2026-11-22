# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Local UDP fixture only: no real controller, no SDK heartbeat object."""
import csv
import importlib.util
from pathlib import Path
import socket
import struct
import tempfile
import threading
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location("passive",ROOT/"tools/analysis/passive_record.py")
passive=importlib.util.module_from_spec(spec)
spec.loader.exec_module(passive)

def packet(seq):
    raw=bytearray(348)
    struct.pack_into("<IHHIIQ",raw,0,0x444D544C,3,8,seq,seq,seq*1000+1000000)
    for i in range(8):
        struct.pack_into("<iiiBBH",raw,24+16*i,100+i,2,300+i,1,28,30)
        struct.pack_into("<iiiBBBB",raw,180+16*i,200+i,3,400+i,29,5,0,0)
    struct.pack_into("<IIBBBB",raw,152,4,5,0,20<<2,127,0xDA)
    struct.pack_into("<8H",raw,308,1,2,1001,1,0,1002,0,0)
    return bytes(raw)

class PassiveTests(unittest.TestCase):
    def test_schema_and_filter(self):
        seq,row=passive.decode(packet(7),1.5,5)
        r=dict(zip(passive.HEADER,row))
        self.assertEqual(len(row),98)
        self.assertEqual((r["mode"],r["profile"]),(10,1))
        self.assertEqual((seq,r["lost"],r["status"]),(7,1,20))
        self.assertEqual((r["q1"],r["q_cmd1"],r["tau_cmd8"]),(.1,.2,.407))
        self.assertIsNone(passive.decode(packet(7),1.5,7))
        self.assertIsNone(passive.decode(b"garbage",1.5))
        for prefix in ("q","dq","tau","state","temp_mos","q_cmd","dq_cmd","tau_cmd","temp_coil","flags"):
            self.assertIn(f'append_columns(header, "{prefix}", 8)',(ROOT/"src/recorder.cpp").read_text())

    def test_only_one_registration(self):
        received=[]
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as fake, tempfile.TemporaryDirectory() as temp:
            fake.bind(("127.0.0.1",0))
            fake.settimeout(.4)
            def serve():
                hello,peer=fake.recvfrom(2048)
                received.append(hello)
                for seq in range(4):
                    fake.sendto(packet(seq),peer)
                try:
                    while True:received.append(fake.recvfrom(2048)[0])
                except socket.timeout:pass
            worker=threading.Thread(target=serve)
            worker.start()
            output=Path(temp)/"capture.csv"
            result=passive.record("127.0.0.1",output,.15,port=fake.getsockname()[1],bind="127.0.0.1")
            worker.join()
            self.assertEqual(received,[passive.HELLO])
            self.assertEqual(result["rows"],4)
            with output.open() as stream:rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),4)
            self.assertEqual(list(rows[0]),passive.HEADER)

if __name__=="__main__":
    unittest.main()
