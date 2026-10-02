#!/usr/bin/env python3
"""AERIAL-EXPLORE-BENCH A6: VIO drift vs ground truth (installation step A6).

Inputs (CSV):
  ground truth  : sim_t,x,y,z,qx,qy,qz,qw   (ENU, 10 Hz, out-of-band from Isaac)
  VIO           : t,x,y,z,qx,qy,qz,qw       (OpenVINS /poseimu, VIO world frame)

Both clocks are the same simulation clock (the a6 app stamps every message with
its physics-step time), so rows join on time directly, no wall-clock join.

Alignment: translation-only at the first VIO sample. The VIO world frame G is
gravity-aligned with yaw taken from the initial IMU pose; the drone spawns at
yaw 0, so G ≈ ENU up to the small initial-attitude error, good enough for a
drift plot (and any residual yaw shows up as horizontal drift, which is the
quantity being reported anyway). A similarity (Umeyama) fit is also computed to
report the monocular scale factor separately.

Usage:  python3 analysis/plot_vio_drift.py [--gt analysis/data/a6_gt.csv] [--vio analysis/data/a6_vio.csv]
                                           [--out analysis/vio_drift_seed20260723001.png]
                                           [--label "world 20260723001"]
"""
import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# palette (categorical slots)
C_GT = "#2a78d6"     # slot 1 blue: ground truth
C_VIO = "#eb6834"    # slot 2 orange: VIO estimate
C_ERR = "#4a3aa7"    # slot 7 violet: error norm
INK = "#1a1a19"
INK2 = "#6b6a60"
GRID = "#d8d7cd"


def load_csv(path):
    rows = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or line[0].isalpha() or line.startswith("sim_t"):
                continue
            parts = line.split(",")
            if len(parts) >= 4:
                rows.append([float(v) for v in parts[:4]])
    a = np.array(rows)
    return a[:, 0], a[:, 1:4]


def umeyama(src, dst):
    """Similarity transform dst ≈ s * R @ src + t  (Umeyama 1991). Returns s, R, t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (xs ** 2).sum() / len(src)
    s = np.trace(np.diag(D) @ S) / var_s
    t = mu_d - s * R @ mu_s
    return s, R, t


def style_ax(ax):
    ax.spines[["top", "right"]].set_visible(False)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.7, alpha=0.6)
    ax.set_axisbelow(True)
    ax.xaxis.label.set_color(INK2)
    ax.yaxis.label.set_color(INK2)
    ax.title.set_color(INK)


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--gt", default=os.path.join(here, "data", "a6_gt.csv"))
    ap.add_argument("--vio", default=os.path.join(here, "data", "a6_vio.csv"))
    ap.add_argument("--out", default=os.path.join(here, "vio_drift_seed20260723001.png"))
    ap.add_argument("--label", default="world 20260723001")
    args = ap.parse_args()

    t_gt, p_gt = load_csv(args.gt)
    t_vio, p_vio = load_csv(args.vio)

    # join on the (shared) sim clock: interpolate GT at the VIO timestamps
    keep = (t_vio >= t_gt[0]) & (t_vio <= t_gt[-1])
    t_vio, p_vio = t_vio[keep], p_vio[keep]
    gt_at_vio = np.column_stack([np.interp(t_vio, t_gt, p_gt[:, i]) for i in range(3)])

    # translation-only alignment at the first VIO sample
    tau = t_vio - t_vio[0]
    gt_rel = gt_at_vio - gt_at_vio[0]
    vio_rel = p_vio - p_vio[0]
    err = vio_rel - gt_rel
    err_n = np.linalg.norm(err, axis=1)
    rmse = float(np.sqrt((err_n ** 2).mean()))
    final = float(err_n[-1])
    dist_gt = float(np.sum(np.linalg.norm(np.diff(gt_rel, axis=0), axis=1)))

    # similarity fit (reports the monocular scale error separately)
    s, R, t = umeyama(vio_rel, gt_rel)
    ate_sim = vio_rel @ (s * R).T + t - gt_rel
    ate_rmse = float(np.sqrt((np.linalg.norm(ate_sim, axis=1) ** 2).mean()))

    print(f"window: {t_vio[0]:.1f} -> {t_vio[-1]:.1f} s sim ({tau[-1]:.1f} s, {len(t_vio)} poses)")
    print(f"GT path length in window: {dist_gt:.2f} m")
    print(f"drift (translation-aligned): final {final:.3f} m, RMSE {rmse:.3f} m")
    print(f"monocular scale (VIO->GT): {s:.3f}  |  ATE after similarity fit: {ate_rmse:.3f} m")

    fig, axes = plt.subplots(3, 1, figsize=(8.6, 10.2), constrained_layout=True)
    fig.patch.set_facecolor("white")
    fig.suptitle("VIO drift vs ground truth: OpenVINS (mono) on the A6 hover flight",
                 fontsize=13, color=INK, fontweight="bold")

    ax = axes[0]
    ax.plot(tau, gt_rel[:, 2], color=C_GT, lw=2.0, label="Ground truth")
    ax.plot(tau, vio_rel[:, 2], color=C_VIO, lw=2.0, label="VIO estimate")
    ax.set_title(f"Altitude above the takeoff point, {args.label}", fontsize=10.5, loc="left")
    ax.set_ylabel("Δz  [m]")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2, loc="lower right")
    style_ax(ax)

    ax = axes[1]
    ax.plot(tau, err_n, color=C_ERR, lw=2.0)
    ax.set_title("Position error ‖VIO − GT‖ (translation-aligned at t₀)", fontsize=10.5, loc="left")
    ax.set_ylabel("error  [m]")
    ax.set_xlabel("time since VIO initialization  [s]")
    ax.text(0.98, 0.06,
            f"final drift {final:.2f} m · RMSE {rmse:.2f} m\n"
            f"monocular scale {s:.2f}× · ATE (similarity-aligned) {ate_rmse:.2f} m\n"
            f"window {tau[-1]:.0f} s · {len(t_vio)} poses",
            transform=ax.transAxes, va="bottom", ha="right", fontsize=8.5, color=INK2)
    style_ax(ax)

    ax = axes[2]
    ax.plot(gt_rel[:, 0], gt_rel[:, 1], color=C_GT, lw=2.0, label="Ground truth")
    ax.plot(vio_rel[:, 0], vio_rel[:, 1], color=C_VIO, lw=2.0, label="VIO estimate")
    ax.plot(0, 0, "o", color=INK, ms=5)
    ax.annotate("start", (0, 0), textcoords="offset points", xytext=(6, 6),
                fontsize=8.5, color=INK2)
    ax.set_title("Top view (x–y)", fontsize=10.5, loc="left")
    ax.set_xlabel("x  [m]")
    ax.set_ylabel("y  [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2, loc="best")
    style_ax(ax)

    fig.savefig(args.out, dpi=150, facecolor="white")
    print(f"saved -> {args.out}")


if __name__ == "__main__":
    main()
