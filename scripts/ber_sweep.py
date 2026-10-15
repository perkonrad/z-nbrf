#!/usr/bin/env python3
"""Run RTL BER sweeps (Verilator) and compare against theory. Writes build/ber.json."""
import subprocess, re, json, os, sys
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.special import erfc
ROOT = os.path.join(os.path.dirname(__file__), "..")

def theory(mod, esn0_db):
    es = 10 ** (esn0_db / 10)
    if mod == 0:   # QPSK Gray
        return 0.5 * erfc(np.sqrt(es / 2))
    # 16-QAM Gray, square constellation (standard approximation, exact at high SNR)
    return 3 / 8 * erfc(np.sqrt(es / 10))

def run(args, tag):
    rd = os.path.join(ROOT, "build", "sweep", tag)
    os.makedirs(rd, exist_ok=True)
    out = subprocess.run([os.path.join(ROOT, "build/tb/Vtb_top"), "+DUMP=0"] + args, cwd=rd,
                         capture_output=True, text=True).stdout
    m = re.search(r"STATS ok=(\d+) crc=(\d+) hdr=(\d+) lost=(-?\d+) bits=(\d+) errs=(\d+) mer=([-\d.na]+)", out)
    if not m:
        return dict(err=out[-500:])
    k = ["ok", "crc", "hdr", "lost", "bits", "errs"]
    d = {kk: int(v) for kk, v in zip(k, m.groups()[:6])}
    d["mer"] = float(m.group(7)) if m.group(7) not in ("nan", "-nan") else None
    return d

jobs = []
for R in (8, 40):
    for e in (5, 6, 7, 8, 9, 10, 11):
        jobs.append(dict(R=R, MOD=0, ESN0=e))
    for e in (12, 13, 14, 15, 16, 17, 18):
        jobs.append(dict(R=R, MOD=1, ESN0=e))
def go(j):
    npkt = 40 if j["R"] == 8 else 12
    args = [f"+R={j['R']}", f"+MOD={j['MOD']}", f"+ESN0={j['ESN0']}", f"+NPKT={npkt}", "+LMIN=200",
            "+LMAX=400", "+CFO=5000", "+SRO=10", f"+SEED={j['ESN0']*7+j['R']}"]
    r = run(args, f"R{j['R']}_M{j['MOD']}_E{j['ESN0']}")
    r.update(j)
    r["theory"] = float(theory(j["MOD"], j["ESN0"]))
    r["ber"] = r["errs"] / r["bits"] if r.get("bits") else None
    return r
with ThreadPoolExecutor(max_workers=os.cpu_count()) as ex:
    res = list(ex.map(go, jobs))
for r in res:
    print(f"R={r['R']:2d} {'16QAM' if r['MOD'] else 'QPSK '} Es/N0={r['ESN0']:4.1f}  BER={r['ber'] if r['ber'] is not None else float('nan'):.2e} "
          f"(theory {r['theory']:.2e}) frames ok/crc/hdr/lost={r.get('ok')}/{r.get('crc')}/{r.get('hdr')}/{r.get('lost')} MER={r.get('mer')}")
json.dump(res, open(os.path.join(ROOT, "build", "ber.json"), "w"), indent=1)
