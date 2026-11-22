#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Offline return analysis. Compile the MCU FK; no sockets, SDK or robot connection."""
import argparse
import csv
import ctypes
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import numpy as np

def windows_from_log(path):
    windows = []
    for line in Path(path).read_text(errors="replace").splitlines():
        if "ADM RET " not in line:
            continue
        item = dict(re.findall(r"(\w+)=([^\s]+)",line))
        if "start_us" not in item or "end_us" not in item:
            continue
        start,end = int(item["start_us"])/1e6,int(item["end_us"])/1e6
        # The ADM event already uses detected-release start and final/timeout end.
        eq=None
        if "qeq_urad" in item:
            values=item["qeq_urad"].split(",")
            if len(values)==7:eq=[int(v)/1e6 for v in values]
        if end>start:
            windows.append((start,end,"ADMITTANCE","ADM RET MCU timestamps",eq))
    return windows

def inferred_windows(data):
    active = np.isin(data["status"],[19,20])
    velocity = np.column_stack([data[f"dq_cmd{i}"] for i in range(1,8)])
    selected = active & (np.max(np.abs(velocity),axis=1)>.003)
    source,mode = "reference movement (inferred; includes guiding)","ADMITTANCE"
    if not np.any(selected):
        q = np.column_stack([data[f"q{i}"] for i in range(1,8)])
        moving = np.max(np.abs(np.diff(q,axis=0,prepend=q[:1])),axis=1)>.0005
        selected = active & moving
        source,mode = "motion only: may include hand pushes; use --serial-log or --window","unknown"
    indices = np.flatnonzero(selected)
    if not len(indices):
        return []
    t = data["mcu_time_s"]
    groups = np.split(indices,np.flatnonzero(np.diff(t[indices])>.3)+1)
    return [(t[g[0]],t[g[-1]],mode,source,None) for g in groups if t[g[-1]]-t[g[0]]>.15]

def fk_all(data,root,work,prefix="q"):
    lib = Path(work)/"fk.so"
    if not lib.exists():subprocess.run([os.environ.get("CC","cc"),"-std=c11","-O2","-shared","-fPIC",
                    "-I"+str(root/"control"),str(root/"control/infrabot_ik_mcu.c"),"-lm","-o",str(lib)],check=True)
    class Pose(ctypes.Structure):
        _fields_ = [("position",ctypes.c_float*3),("rotation",ctypes.c_float*9)]
    fk = ctypes.CDLL(str(lib)).ArmIk_Forward
    fk.argtypes = [ctypes.POINTER(ctypes.c_float),ctypes.POINTER(Pose)]
    result = []
    for row in data:
        pose = Pose()
        q = (ctypes.c_float*7)(*(row[f"{prefix}{i}"] for i in range(1,8)))
        fk(q,ctypes.byref(pose))
        result.append([*pose.position,*pose.rotation])
    return np.asarray(result)

def angle_between(a,b):
    # acos is adequate for offline angle magnitudes, including near pi.
    relative = a@b.T
    return np.arccos(np.clip((np.trace(relative)-1)/2,-1,1))

def band_energy(time,signal):
    if len(time)<64 or np.any(np.diff(time)>.005):
        return dict(energy_Nm2=None,reason="too short or a telemetry gap >5ms")
    dt = np.median(np.diff(time))
    if not 0<dt<=.0025:
        return dict(energy_Nm2=None,reason="sample rate insufficient for 200Hz")
    grid = np.arange(time[0],time[-1],dt)
    y = np.interp(grid,time,signal)
    y -= y.mean()
    # Mean-square torque in the frequency band (not mechanical energy in J).
    spectrum = np.fft.rfft(y)
    freq = np.fft.rfftfreq(len(y),dt)
    weights = np.full(len(spectrum),2.0)
    weights[0]=1
    if len(y)%2==0:weights[-1]=1
    energy = np.sum((np.abs(spectrum)**2*weights/len(y)**2)[(freq>=20)&(freq<=200)])
    return dict(energy_Nm2=float(energy),rate_Hz=float(1/dt),resampled=True)

def analyze(csv_path,root,out,serial_log=None,explicit=None):
    out = Path(out)
    out.mkdir(parents=True,exist_ok=True)
    data = np.genfromtxt(csv_path,delimiter=",",names=True,ndmin=1)
    if len(data)<2 or not np.all(np.diff(data["mcu_time_s"])>0):
        raise ValueError("MCU timestamps must increase; split recordings across board resets")
    if not all(np.all(np.isfinite(data[f"{prefix}{i}"])) for prefix in ("q","q_cmd","tau","tau_cmd") for i in range(1,8)):
        raise ValueError("Analysis requires complete DMTL v3 measured/commanded joints")
    windows = windows_from_log(serial_log) if serial_log else []
    if explicit:
        windows = [(float(a),float(b),"ADMITTANCE","explicit window",None) for a,b in (x.split(":") for x in explicit)]
    if not windows:
        windows = inferred_windows(data)
    with tempfile.TemporaryDirectory() as work:
        fk = fk_all(data,root,work)
        known=[i for i,w in enumerate(windows) if w[4] is not None]
        eq_poses={}
        if known:
            eq_data=np.array([tuple(windows[i][4]) for i in known],dtype=[(f"q{j}","f8") for j in range(1,8)])
            for i,pose in zip(known,fk_all(eq_data,root,work)):eq_poses[i]=pose
    reports = []
    for number,(start,end,mode,source,eq) in enumerate(windows,1):
        mask = (data["mcu_time_s"]>=start)&(data["mcu_time_s"]<=end)&np.isin(data["status"],[19,20])
        sample = data[mask]
        pose = fk[mask]
        if len(sample)<2:
            continue
        t = sample["mcu_time_s"]
        xyz,rot = pose[:,:3],pose[:,3:].reshape(-1,3,3)
        known_goal=(number-1) in eq_poses
        if known_goal:
            goal_position=eq_poses[number-1][:3];goal_rotation=eq_poses[number-1][3:].reshape(3,3);goal_q=np.array(eq)
        else:
            goal_position=xyz[-1];goal_rotation=rot[-1];goal_q=None
        mode="JOINT_ADM" if int(sample["status"][0])==19 else "CART_ADM"
        delta = goal_position-xyz[0]
        length2 = np.dot(delta,delta)
        s = np.clip((xyz-xyz[0])@delta/max(length2,1e-16),0,1)
        dev = np.linalg.norm(xyz-xyz[0]-s[:,None]*delta,axis=1)
        speed = np.zeros(len(t))
        omega = np.zeros(len(t))
        for k in range(1,len(t)):
            old = np.searchsorted(t,t[k]-.04)
            old = min(old,k-1)
            dt = t[k]-t[old]
            dp = np.linalg.norm(xyz[k]-xyz[old])
            dr = angle_between(rot[k],rot[old])
            speed[k] = 0 if dp<=.00015 else dp/dt
            omega[k] = 0 if dr<=.0005 else dr/dt
        dt = np.diff(t,prepend=t[0])
        moving = False
        sticks = 0
        for v in speed:
            if v>.01:moving=True
            elif v<.002 and moving:sticks+=1;moving=False
        report = dict(return_id=number,mode=mode,window_source=source,start_s=float(t[0]),end_s=float(t[-1]),
                      samples=len(t),deviation_mm=float(dev.max()*1000),stall_ms=float(dt[(s<.9)&(speed<.002)].sum()*1000),
                      sticks=sticks,peak_mm_s=float(speed.max()*1000),lost=int(sample["lost"].sum()),
                      endpoint_basis="ADM RET qeq_urad" if known_goal else "measured endpoint; final accuracy unavailable",
                      final_mm=float(np.linalg.norm(xyz[-1]-goal_position)*1000) if known_goal else None,
                      final_mrad=float(angle_between(rot[-1],goal_rotation)*1000) if known_goal else None,
                      ns_mrad=float(max(abs(sample[f"q{i}"][-1]-goal_q[i-1]) for i in range(1,8))*1000) if known_goal else None,
                      torque_note="tau_cmd is feedforward only; in PD mode FF-drive differs by internal PD effort",joints=[])
        for j in range(1,8):
            ff,drive = sample[f"tau_cmd{j}"],sample[f"tau{j}"]
            report["joints"].append(dict(joint=j,ff_drive_rms_Nm=float(np.sqrt(np.mean((ff-drive)**2))),
                max_ff_step_Nm=float(np.max(np.abs(np.diff(ff)))),
                drive_20_200Hz=band_energy(t,drive),ff_20_200Hz=band_energy(t,ff)))
        reports.append(report)
        with (out/f"return_{number:03d}_path.csv").open("w",newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["mcu_time_s","x_m","y_m","z_m","deviation_m","speed_m_s","angular_speed_rad_s",*(f"R{i}{j}" for i in range(3) for j in range(3))])
            for i in range(len(t)):
                writer.writerow([t[i],*xyz[i],dev[i],speed[i],omega[i],*rot[i].ravel()])
    checksum = hashlib.sha256((root/"control/infrabot_ik_mcu.c").read_bytes()+(root/"control/arm_kinematics_model.inc").read_bytes()).hexdigest()
    result = dict(fk_sha256=checksum,returns=reports,
                  phase_note="DMTL has no detected-release substate; timestamped ADM RET supplies windows and equilibrium. q_cmd is a moving reference, not equilibrium.")
    (out/"report.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    return result

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv",type=Path)
    p.add_argument("--mcu-root",type=Path,default=os.environ.get("INFRABOT_FIRMWARE_DIR") or None, help="firmware checkout (or set INFRABOT_FIRMWARE_DIR)")
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--serial-log",type=Path)
    p.add_argument("--window",action="append",help="MCU start:end seconds, repeatable")
    a = p.parse_args()
    if a.mcu_root is None:
        p.error("specify --mcu-root or set INFRABOT_FIRMWARE_DIR")
    result = analyze(a.csv,a.mcu_root,a.out,a.serial_log,a.window)
    for row in result["returns"]:
        print(f"return {row['return_id']} {row['mode']}: dev={row['deviation_mm']:.3f} mm stall={row['stall_ms']:.1f} ms sticks={row['sticks']}")
    print(a.out/"report.json")

if __name__=="__main__":
    main()
