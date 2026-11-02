# BEN 2026-11-02
# Change aim: how far apart can the clouds start? ICP success versus initial rotation, 5 seeds, PCL (C++) and Open3D (Python).
# File: run_experiments.py
# SIMULATED data only. Run from the repo root after building: python3 tools/run_experiments.py
import os
import re
import subprocess
import sys
import tempfile
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "python"))
import make_data
import icp_registration as ico

ANGLES = [5, 15, 30, 45, 60, 90]
SEEDS = [1, 2, 3, 4, 5]
SUCCESS_DEG = 5.0          # success = rotation error below 5 degrees


def run_cpp(d):
    out = subprocess.run(["./build/icp_registration", "--data", d, "--out", d, "--tag", "cpp"], capture_output=True, text=True, check=True).stdout
    return float(re.search(r"rotation error: ([\d.]+) deg", out).group(1))


def main():
    rows, res = ["angle (deg) | PCL C++ success | PCL mean rot err (deg) | Open3D success | Open3D mean rot err (deg)   [5 seeds, success = rot err < 5 deg]"], {}
    for ang in ANGLES:
        e = {"cpp": [], "py": []}
        for sd in SEEDS:
            with tempfile.TemporaryDirectory() as d:
                make_data.make(d, ang, sd)
                e["cpp"].append(run_cpp(d))
                r, _, _ = ico.run(os.path.join(d, "source.ply"), os.path.join(d, "target.ply"), os.path.join(d, "gt.txt"))
                e["py"].append(r["rot_err"])
        res[ang] = {k: (sum(x < SUCCESS_DEG for x in v), float(np.mean(v))) for k, v in e.items()}
        rows.append(f"{ang} | {res[ang]['cpp'][0]}/5 | {res[ang]['cpp'][1]:.2f} | {res[ang]['py'][0]}/5 | {res[ang]['py'][1]:.2f}")
    open("results/experiments.txt", "w").write("\n".join(rows) + "\n")
    print("\n".join(rows))
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    w = 0.38
    x = np.arange(len(ANGLES))
    ax.bar(x - w / 2, [res[a]["cpp"][0] for a in ANGLES], w, label="PCL (C++)", color="#3b6ea5")
    ax.bar(x + w / 2, [res[a]["py"][0] for a in ANGLES], w, label="Open3D (Python)", color="#d98c3f")
    ax.set_xticks(x); ax.set_xticklabels([f"{a} deg" for a in ANGLES]); ax.set_ylim(0, 5.5)
    ax.set_ylabel("successful runs (of 5)"); ax.set_xlabel("initial rotation between the clouds")
    ax.set_title("SIMULATED: ICP success vs starting misalignment"); ax.legend(loc="lower left")
    fig.tight_layout(); fig.savefig("results/experiments.png", dpi=130)


if __name__ == "__main__":
    main()
