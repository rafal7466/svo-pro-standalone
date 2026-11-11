#!/usr/bin/env python3
"""Generate the README figures from the stored EuRoC MH_01 results.

    python scripts/make_readme_figure.py

Reads results/MH_01_*/ (TUM trajectories + results.txt) and writes light and
dark variants of two figures to docs/images/:
  * mh01_trajectory_{light,dark}.png - top view + position error over time
  * mh01_ate_{light,dark}.png        - ATE RMSE and runtime per configuration
Requires numpy and matplotlib.
"""
import re
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
OUT = ROOT / "docs" / "images"

RUNS = [  # folder, label, aligned trajectory, colour
    ("MH_01_stereo_vio", "Stereo VIO", "stamped_traj_backend_aligned_posyaw.txt", "#1f77b4"),
    ("MH_01_mono_vio", "Mono VIO", "stamped_traj_backend_aligned_posyaw.txt", "#d62728"),
    ("MH_01_stereo_vo", "Stereo VO (no IMU)", "stamped_traj_frontend_aligned_se3.txt", "#2ca02c"),
]

THEMES = {
    "light": dict(bg="white", fg="#24292f", grid="#d0d7de", gt="#24292f"),
    "dark": dict(bg="#0d1117", fg="#e6edf3", grid="#30363d", gt="#e6edf3"),
}


def load_tum(path):
    return np.loadtxt(path, comments="#")[:, :4]  # t x y z


def position_error(est, gt):
    """Euclidean position error of every estimate w.r.t. the nearest-in-time groundtruth."""
    idx = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
    prev_closer = (est[:, 0] - gt[idx - 1, 0]) < (gt[idx, 0] - est[:, 0])
    idx = idx - prev_closer
    return np.linalg.norm(est[:, 1:4] - gt[idx, 1:4], axis=1)


def parse_summary(path):
    txt = path.read_text(encoding="utf-8", errors="replace")
    ate = {}
    for m in re.finditer(r"^\s*(frontend|backend)\s+(posyaw|se3|sim3)\s+RMSE\s+([\d.]+)", txt, re.M):
        ate[(m.group(1), m.group(2))] = float(m.group(3))
    ms = float(re.search(r"time / frame\s*:\s*mean\s+([\d.]+)\s*ms", txt).group(1))
    return ate, ms


def style_axes(ax, th):
    ax.set_facecolor(th["bg"])
    for s in ax.spines.values():
        s.set_color(th["grid"])
    ax.tick_params(colors=th["fg"])
    ax.xaxis.label.set_color(th["fg"])
    ax.yaxis.label.set_color(th["fg"])
    ax.title.set_color(th["fg"])
    ax.grid(True, color=th["grid"], lw=0.6, alpha=0.8)
    ax.set_axisbelow(True)


def trajectory_figure(theme):
    th = THEMES[theme]
    gt = load_tum(RES / "MH_01_stereo_vio" / "stamped_groundtruth.txt")
    t0 = gt[0, 0]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.8), gridspec_kw=dict(width_ratios=[1, 1.35]))
    fig.patch.set_facecolor(th["bg"])
    a1.plot(gt[:, 1], gt[:, 2], color=th["gt"], lw=2.2, ls="--", label="Groundtruth (Leica + Vicon)")
    for folder, label, fname, col in RUNS:
        est = load_tum(RES / folder / fname)
        a1.plot(est[:, 1], est[:, 2], color=col, lw=1.1, label=label)
        err = position_error(est, gt)
        a2.plot(est[:, 0] - t0, err, color=col, lw=1.0,
                label=f"{label}  (RMSE {np.sqrt(np.mean(err**2)):.3f} m)")
    a1.set_aspect("equal", adjustable="datalim")
    a1.set_xlabel("x [m]"); a1.set_ylabel("y [m]")
    a1.set_title("EuRoC MH_01_easy - top view (aligned)")
    a2.set_xlabel("time [s]"); a2.set_ylabel("position error [m]")
    a2.set_title("Absolute position error over time")
    a2.set_ylim(0, None)
    a2.axvspan(0, 20, color=th["grid"], alpha=0.35, lw=0)
    a2.text(10, 0.01, "hand-held\nIMU excitation", ha="center", va="bottom",
            fontsize=8, color=th["fg"])
    for ax in (a1, a2):
        style_axes(ax, th)
        leg = ax.legend(fontsize=8.5, facecolor=th["bg"], edgecolor=th["grid"], loc="best")
        for t in leg.get_texts():
            t.set_color(th["fg"])
    fig.tight_layout()
    fig.savefig(OUT / f"mh01_trajectory_{theme}.png", dpi=150, facecolor=th["bg"])
    plt.close(fig)


def ate_figure(theme):
    th = THEMES[theme]
    labels, posyaw, se3, sim3, ms = [], [], [], [], []
    for folder, label, _, _ in RUNS:
        ate, t = parse_summary(RES / folder / "results.txt")
        part = "backend" if ("backend", "se3") in ate else "frontend"
        labels.append(label)
        posyaw.append(ate.get((part, "posyaw"), np.nan))
        se3.append(ate[(part, "se3")])
        sim3.append(ate[(part, "sim3")])
        ms.append(t)
    x = np.arange(len(labels)); w = 0.26
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw=dict(width_ratios=[1.6, 1]))
    fig.patch.set_facecolor(th["bg"])
    for off, vals, name, col in ((-w, posyaw, "posyaw (4-DoF)", "#8250df"),
                                  (0, se3, "SE(3)", "#1f77b4"),
                                  (w, sim3, "Sim(3)", "#2ca02c")):
        bars = a1.bar(x + off, vals, w, label=name, color=col)
        for b, v in zip(bars, vals):
            if np.isfinite(v):
                a1.text(b.get_x() + b.get_width() / 2, v + 0.002, f"{v:.3f}", ha="center",
                        va="bottom", fontsize=8, color=th["fg"])
    a1.set_xticks(x, labels)
    a1.set_ylabel("ATE RMSE [m]")
    a1.set_title("Absolute trajectory error on MH_01 (by alignment)")
    bars = a2.bar(x, ms, 0.55, color=[r[3] for r in RUNS])
    for b, v in zip(bars, ms):
        a2.text(b.get_x() + b.get_width() / 2, v + 0.2, f"{v:.1f} ms", ha="center", va="bottom",
                fontsize=9, color=th["fg"])
    a2.set_xticks(x, [l.replace(" (no IMU)", "") for l in labels])
    a2.set_ylabel("mean time per frame [ms]")
    a2.text(0.5, -0.2, "frontend + waiting for depth-filter / backend threads", transform=a2.transAxes,
            ha="center", fontsize=8, color=th["fg"])
    a2.set_title("Runtime (i9-13950HX, Release)")
    for ax in (a1, a2):
        style_axes(ax, th)
        ax.set_ylim(0, None)
        ax.margins(y=0.15)
    leg = a1.legend(fontsize=8.5, facecolor=th["bg"], edgecolor=th["grid"])
    for t in leg.get_texts():
        t.set_color(th["fg"])
    fig.tight_layout()
    fig.savefig(OUT / f"mh01_ate_{theme}.png", dpi=150, facecolor=th["bg"])
    plt.close(fig)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for theme in THEMES:
        trajectory_figure(theme)
        ate_figure(theme)
    print("figures written to", OUT)
