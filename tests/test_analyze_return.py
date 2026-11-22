# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
import csv
import os
import importlib.util
from pathlib import Path
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location("analyze",ROOT/"tools/analysis/analyze_return.py")
analyze=importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyze)
spec=importlib.util.spec_from_file_location("passive",ROOT/"tools/analysis/passive_record.py")
passive=importlib.util.module_from_spec(spec)
spec.loader.exec_module(passive)

class AnalysisTests(unittest.TestCase):
    def test_spectrum(self):
        t=np.arange(2000)/1000
        r=analyze.band_energy(t,2*np.sin(2*np.pi*50*t))
        self.assertAlmostEqual(r["energy_Nm2"],2,delta=.01)
        self.assertIsNone(analyze.band_energy(np.array([0,.1]),np.array([1,0]))["energy_Nm2"])

    @unittest.skipUnless(os.environ.get("INFRABOT_FIRMWARE_DIR"), "SKIP: INFRABOT_FIRMWARE_DIR is unset")
    def test_real_mcu_fk_and_log_windows(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);log=root/"serial.log";data=root/"capture.csv"
            log.write_text("ADM RET start_us=1010000 end_us=1300000 qeq_urad=220000,900000,-500000,1300000,300000,600000,-200000\n")
            with data.open("w",newline="") as stream:
                writer=csv.DictWriter(stream,fieldnames=passive.HEADER)
                writer.writeheader()
                for n in range(400):
                    row=dict.fromkeys(passive.HEADER,0)
                    row.update(mcu_time_s=1+n/1000,status=20,sequence=n)
                    for j,q in enumerate([.2,.9,-.5,1.3,.3,.6,-.2],1):
                        row[f"q{j}"]=q+(n/400*.02 if j==1 else 0)
                        row[f"q_cmd{j}"]=q+(.5 if j==1 else 0)
                        row[f"tau{j}"]=np.sin(2*np.pi*50*n/1000)
                    writer.writerow(row)
            result=analyze.analyze(data,Path(os.environ["INFRABOT_FIRMWARE_DIR"]),root/"out",log)
            r=result["returns"][0]
            self.assertEqual(r["mode"],"CART_ADM")
            self.assertEqual(r["endpoint_basis"],"ADM RET qeq_urad")
            self.assertLess(r["final_mm"],3)
            self.assertEqual(r["window_source"],"ADM RET MCU timestamps")
            self.assertEqual(len(r["joints"]),7)
            self.assertTrue((root/"out/return_001_path.csv").is_file())
            self.assertLess(r["deviation_mm"],1)
            self.assertGreater(r["joints"][0]["drive_20_200Hz"]["energy_Nm2"],.45)

if __name__=="__main__":unittest.main()
