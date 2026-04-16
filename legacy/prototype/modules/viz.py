from __future__ import annotations

import os
from pathlib import Path
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "space_rover_legacy_matplotlib"))

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, LinearSegmentedColormap, ListedColormap
import numpy as np


PAPER_CMAP = LinearSegmentedColormap.from_list(
    "baseline_paper",
    ["#0f172a", "#14532d", "#0f766e", "#d97706", "#991b1b"],
)
DISCRETE_COLORS = [
    "#264653",
    "#2a9d8f",
    "#8ab17d",
    "#e9c46a",
    "#f4a261",
    "#e76f51",
    "#7c3aed",
    "#2563eb",
]
PATH_COLOR = "#ffb703"
START_COLOR = "#2a9d8f"
GOAL_COLOR = "#d62828"
SERIES_COLOR = "#1d3557"


def _use_publication_style():
    mpl.rcParams.update(
        {
            "font.family": "DejaVu Serif",
            "font.size": 10.0,
            "axes.titlesize": 11.5,
            "axes.titleweight": "semibold",
            "axes.labelsize": 10.0,
            "axes.edgecolor": "#334155",
            "axes.linewidth": 0.8,
            "xtick.color": "#334155",
            "ytick.color": "#334155",
            "grid.color": "#cbd5e1",
            "grid.linewidth": 0.6,
            "grid.alpha": 0.65,
            "axes.facecolor": "#f8fafc",
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.04,
            "mathtext.fontset": "stix",
        }
    )


def _finalize_axis(ax, xl="", yl="", grid=False):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color("#475569")
    ax.spines["bottom"].set_color("#475569")
    ax.tick_params(length=3.5, width=0.7)
    if xl:
        ax.set_xlabel(xl)
    if yl:
        ax.set_ylabel(yl)
    if grid:
        ax.grid(True, linestyle="--")


def _save(fig, path=None, dpi=320):
    if path is None:
        return
    fig.savefig(path, dpi=dpi)
    path_str = str(path)
    if path_str.lower().endswith(".png"):
        fig.savefig(path.with_suffix(".pdf"))
    plt.close(fig)


def _discrete_style(A):
    finite = A[np.isfinite(A)]
    if finite.size == 0:
        return None
    rounded = np.round(finite)
    if not np.all(np.abs(finite - rounded) < 1e-6):
        return None
    uniq = np.unique(rounded.astype(int))
    if uniq.size == 0 or uniq.size > len(DISCRETE_COLORS):
        return None
    cmap = ListedColormap(DISCRETE_COLORS[: uniq.size])
    bounds = np.arange(uniq.min() - 0.5, uniq.max() + 1.5, 1.0)
    norm = BoundaryNorm(bounds, cmap.N)
    return cmap, norm


def ensure_ax3d():
    _use_publication_style()
    fig = plt.figure(figsize=(5.8, 4.6), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("white")
    return fig, ax


def plot_surface(Z, title, path=None):
    fig, ax = ensure_ax3d()
    H, W = Z.shape
    X, Y = np.meshgrid(np.arange(W), np.arange(H))
    ax.plot_surface(X, Y, Z, cmap=PAPER_CMAP, linewidth=0, antialiased=True, alpha=0.96)
    ax.set_title(title, pad=10)
    ax.set_xlabel("Map x", labelpad=6)
    ax.set_ylabel("Map y", labelpad=6)
    ax.set_zlabel("Elevation", labelpad=6)
    ax.view_init(elev=31, azim=-132)
    _save(fig, path)


def heatmap(A, title, path=None):
    _use_publication_style()
    fig, ax = plt.subplots(figsize=(5.4, 4.2), constrained_layout=True)
    style = _discrete_style(A)
    if style is None:
        im = ax.imshow(A, origin="lower", cmap=PAPER_CMAP, interpolation="nearest", aspect="auto")
    else:
        cmap, norm = style
        im = ax.imshow(A, origin="lower", cmap=cmap, norm=norm, interpolation="nearest", aspect="auto")
    ax.set_title(title, pad=8)
    _finalize_axis(ax, xl="Map x", yl="Map y", grid=False)
    cbar = fig.colorbar(im, ax=ax, fraction=0.048, pad=0.03)
    cbar.outline.set_edgecolor("#94a3b8")
    cbar.outline.set_linewidth(0.7)
    _save(fig, path)


def overlay_path(Z, path_xy, title, path=None):
    _use_publication_style()
    fig, ax = plt.subplots(figsize=(5.6, 4.3), constrained_layout=True)
    style = _discrete_style(Z)
    if style is None:
        im = ax.imshow(Z, origin="lower", cmap=PAPER_CMAP, interpolation="nearest", aspect="auto")
    else:
        cmap, norm = style
        im = ax.imshow(Z, origin="lower", cmap=cmap, norm=norm, interpolation="nearest", aspect="auto")
    if len(path_xy) > 0:
        xs = [p[1] for p in path_xy]
        ys = [p[0] for p in path_xy]
        ax.plot(
            xs,
            ys,
            linewidth=2.6,
            color=PATH_COLOR,
            solid_capstyle="round",
            path_effects=[pe.Stroke(linewidth=4.4, foreground="white", alpha=0.95), pe.Normal()],
        )
        ax.scatter(xs[0], ys[0], s=48, color=START_COLOR, edgecolor="white", linewidth=0.9, zorder=4)
        ax.scatter(xs[-1], ys[-1], s=56, marker="s", color=GOAL_COLOR, edgecolor="white", linewidth=0.9, zorder=4)
    ax.set_title(title, pad=8)
    _finalize_axis(ax, xl="Map x", yl="Map y", grid=False)
    cbar = fig.colorbar(im, ax=ax, fraction=0.048, pad=0.03)
    cbar.outline.set_edgecolor("#94a3b8")
    cbar.outline.set_linewidth(0.7)
    _save(fig, path)


def series(y, x=None, title="", yl="", xl="", path=None):
    _use_publication_style()
    values = np.asarray(y, dtype=float)
    if x is None:
        x = np.arange(len(values))
    x = np.asarray(x, dtype=float)
    fig, ax = plt.subplots(figsize=(5.6, 3.35), constrained_layout=True)
    ax.plot(x, values, linewidth=2.2, color=SERIES_COLOR, solid_capstyle="round")
    if values.size > 0:
        ax.scatter([x[0], x[-1]], [values[0], values[-1]], s=22, color=SERIES_COLOR, edgecolor="white", linewidth=0.8, zorder=4)
        ax.fill_between(x, values, np.min(values), color="#93c5fd", alpha=0.14)
    ax.set_title(title, pad=8)
    _finalize_axis(ax, xl=xl, yl=yl, grid=True)
    _save(fig, path)


def scatter_xy(x, y, title="", xl="", yl="", path=None):
    _use_publication_style()
    xv = np.asarray(x, dtype=float)
    yv = np.asarray(y, dtype=float)
    fig, ax = plt.subplots(figsize=(5.6, 4.0), constrained_layout=True)
    ax.scatter(xv, yv, s=34, color="#457b9d", edgecolor="white", linewidth=0.8, alpha=0.92)
    if xv.size >= 2:
        order = np.argsort(xv)
        ax.plot(xv[order], yv[order], linewidth=1.35, color="#94a3b8", alpha=0.8)
    ax.set_title(title, pad=8)
    _finalize_axis(ax, xl=xl, yl=yl, grid=True)
    _save(fig, path)
