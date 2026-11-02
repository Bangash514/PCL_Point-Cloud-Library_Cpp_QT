# BEN 2026-11-02
# Change aim: build a synthetic point cloud pair (source, target) with a known rigid transform for ICP tests.
# File: make_data.py
# SIMULATED data only. Writes ASCII PLY files and gt.txt (4x4 matrix, source -> target). Units are arbitrary.
# Usage: python3 python/make_data.py --out data --angle 30 --seed 1
import argparse
import os
import numpy as np


def sample_shape(rng, n_scale=1.0):
    # asymmetric object: big sphere + small sphere + short cylinder (no symmetry, so ICP has a unique answer)
    def sphere(n, r, c):
        v = rng.normal(size=(n, 3))
        return c + r * v / np.linalg.norm(v, axis=1, keepdims=True)

    def cylinder(n, r, h, c):
        a = rng.uniform(0, 2 * np.pi, n)
        z = rng.uniform(-h / 2, h / 2, n)
        return c + np.stack([r * np.cos(a), r * np.sin(a), z], axis=1)

    p = np.vstack([sphere(int(2000 * n_scale), 1.0, np.array([0.0, 0.0, 0.0])),
                   sphere(int(800 * n_scale), 0.5, np.array([1.2, 0.3, 0.0])),
                   cylinder(int(700 * n_scale), 0.25, 1.5, np.array([-0.8, -0.8, 0.0]))])
    return p + rng.normal(scale=0.005, size=p.shape)


def rotation(angle_deg, axis=(1.0, 2.0, 3.0)):
    a = np.array(axis, float)
    a /= np.linalg.norm(a)
    t = np.radians(angle_deg)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * k + (1 - np.cos(t)) * (k @ k)


def write_ply(path, pts):
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n" % len(pts))
        np.savetxt(f, pts, fmt="%.6f")


def make(out, angle=30.0, seed=1, shift=(0.3, -0.2, 0.15)):
    rng = np.random.default_rng(seed)
    src = sample_shape(rng)
    base = sample_shape(rng)                              # independent sampling: no point-to-point correspondences
    r = rotation(angle)
    t = np.array(shift)
    tgt = base @ r.T + t
    os.makedirs(out, exist_ok=True)
    write_ply(os.path.join(out, "source.ply"), src)
    write_ply(os.path.join(out, "target.ply"), tgt)
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    np.savetxt(os.path.join(out, "gt.txt"), m, fmt="%.8f")
    return src, tgt, m


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data")
    ap.add_argument("--angle", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    s, t, m = make(a.out, a.angle, a.seed)
    print("wrote", a.out, "points:", len(s), len(t), "angle", a.angle)
