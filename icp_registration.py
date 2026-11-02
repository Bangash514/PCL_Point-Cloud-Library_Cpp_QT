# BEN 2026-11-02
# Change aim: point-to-point ICP with Open3D, same settings and the same summary format as the PCL C++ program.
# File: icp_registration.py
# SIMULATED data only. Usage: python3 python/icp_registration.py --data data --out results --tag python
import argparse
import os
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

MAX_DIST = 0.5
MAX_ITER = 100


def rot_error_deg(r_est, r_gt):
    c = (np.trace(r_est.T @ r_gt) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def rmse_nn(src, tgt_tree, m):
    p = src @ m[:3, :3].T + m[:3, 3]
    d, _ = tgt_tree.query(p)
    return float(np.sqrt(np.mean(d ** 2)))


def run(src_path, tgt_path, gt_path):
    src = o3d.io.read_point_cloud(src_path)
    tgt = o3d.io.read_point_cloud(tgt_path)
    gt = np.loadtxt(gt_path)
    res = o3d.pipelines.registration.registration_icp(
        src, tgt, MAX_DIST, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-8, relative_rmse=1e-8, max_iteration=MAX_ITER))
    m = np.asarray(res.transformation)
    sp, tp = np.asarray(src.points), np.asarray(tgt.points)
    tree = cKDTree(tp)
    out = {"n_src": len(sp), "n_tgt": len(tp), "rmse_before": rmse_nn(sp, tree, np.eye(4)), "rmse_after": rmse_nn(sp, tree, m),
           "rot_err": rot_error_deg(m[:3, :3], gt[:3, :3]), "trans_err": float(np.linalg.norm(m[:3, 3] - gt[:3, 3])), "matrix": m}
    return out, sp, m


def summary(r):
    return (f"points: source {r['n_src']}, target {r['n_tgt']}\n"
            f"nearest-neighbour RMSE before: {r['rmse_before']:.4f}, after: {r['rmse_after']:.4f}\n"
            f"rotation error: {r['rot_err']:.3f} deg, translation error: {r['trans_err']:.4f}\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="data")
    ap.add_argument("--out", default="results")
    ap.add_argument("--tag", default="python")
    a = ap.parse_args()
    r, sp, m = run(os.path.join(a.data, "source.ply"), os.path.join(a.data, "target.ply"), os.path.join(a.data, "gt.txt"))
    os.makedirs(a.out, exist_ok=True)
    text = summary(r)
    print(text, end="")
    open(os.path.join(a.out, f"summary_{a.tag}.txt"), "w").write(text)
    np.savetxt(os.path.join(a.out, f"transform_{a.tag}.txt"), m, fmt="%.6f")
    np.savetxt(os.path.join(a.out, f"aligned_{a.tag}.xyz"), sp @ m[:3, :3].T + m[:3, 3], fmt="%.5f")
