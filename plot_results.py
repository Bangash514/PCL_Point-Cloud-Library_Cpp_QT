# BEN 2026-11-02
# Change aim: draw target, source before ICP and source after ICP (C++ and Python) for the README.
# File: plot_results.py
# SIMULATED data only. Run from the repo root: python3 tools/plot_results.py
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_ply(p):
    return np.loadtxt(p, skiprows=7)


src, tgt = load_ply("data/source.ply"), load_ply("data/target.ply")
cpp, py = np.loadtxt("results/aligned_cpp.xyz"), np.loadtxt("results/aligned_python.xyz")
panels = [("before ICP", src), ("after ICP: PCL (C++)", cpp), ("after ICP: Open3D (Python)", py)]
fig, ax = plt.subplots(2, 3, figsize=(11, 6.4))
for c, (t, pts) in enumerate(panels):
    for r, (i, j, n) in enumerate([(0, 2, "x-z"), (0, 1, "x-y")]):
        a = ax[r, c]
        a.scatter(tgt[::2, i], tgt[::2, j], s=1.5, c="#3b6ea5", label="target")
        a.scatter(pts[::2, i], pts[::2, j], s=1.5, c="#d94f3f", label="source")
        a.set_aspect("equal"); a.set_xticks([]); a.set_yticks([])
        a.set_title(f"{t} ({n})", fontsize=9)
ax[0, 0].legend(markerscale=6, loc="upper left", fontsize=8)
fig.suptitle("SIMULATED point clouds: blue = target, red = source")
fig.tight_layout(); fig.savefig("results/registration.png", dpi=130)
