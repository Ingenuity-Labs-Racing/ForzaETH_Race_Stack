#!/usr/bin/env python3
"""
Figures, tables and quoted numbers for the LaTeX report (report/main.tex), from the benchmark results.

    python3 scripts/report_figures.py       -> report/figures/*.pdf, report/tables/*.tex

Every number comes from the per-run files, read through sweep_report.load_runs (driving-window
metrics), so the report can't drift from the results. Tags used:
  speedsweep          original speed sweep (scan matcher and AMCL on the filled map, SLAM Toolbox
                      with loop closure on its first reference map)
  amcl_outline        AMCL on the outline map
  slam_fixed          SLAM Toolbox, loop-closed map, loop closure off in localization
  slam-*-smoke        the 1-lap SLAM Toolbox variants (map x loop closure)
  odom_noise          the odometry noise test
Figure text is set with LaTeX (Latin Modern, like the document) when `latex` is installed.
Runs on the host (matplotlib, numpy, scipy, pyyaml, pillow).
"""
import math
import shutil
import sys
from pathlib import Path

import numpy as np

import plot_results as pr
import sweep_report as sr
import matplotlib as mpl
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch, Rectangle  # noqa: E402
from matplotlib.ticker import FuncFormatter, NullFormatter  # noqa: E402
from scipy.ndimage import distance_transform_edt  # noqa: E402

sys.path.insert(0, str(pr.SANDBOX))
from slam_toolbox_sandbox.png_to_world import load_occupancy  # noqa: E402

FIG = pr.SANDBOX / 'report' / 'figures'
TAB = pr.SANDBOX / 'report' / 'tables'
MAPS = pr.SANDBOX / 'maps'
W = 6.5  # [in] text width of the report

NAME = {'scan': 'Scan matcher', 'amcl': 'AMCL', 'slam': 'SLAM Toolbox', 'odom': 'Odometry only'}
COLOR = dict(pr.COLOR)
LOCS = ['scan', 'amcl', 'slam']
INK, INK2, FAINT, GRID = '#1f1f1f', '#5a5a5a', '#c9c9c9', '#e3e3e3'
BAD = '#c7362f'      # map pixels far from the true wall (a state, not a series)
LEVELS = ['0', '1', '2', '4', '8']
AMCL_SIGMA_HIT, AMCL_MAX_DIST = 0.2, 2.0   # config/amcl.yaml
START_ZONE = (-8.0, -2.0, -4.0, 4.0)      # where the doubled walls of victoria_ref are (x0, x1, y0, y1)
HIT_CMAP = LinearSegmentedColormap.from_list('hit', ['#ffffff', '#cfe0f5', '#8db7e6', '#3f86d1', '#1b5aa0', '#0d3566'])


def style():
    usetex = shutil.which('latex') is not None
    mpl.rcParams.update(mpl.rcParamsDefault)  # drop plot_results' screen style
    mpl.rcParams.update({
        'text.usetex': usetex,
        'text.latex.preamble': r'\usepackage[T1]{fontenc}\usepackage{lmodern}\usepackage{amsmath}',
        'font.family': 'serif', 'mathtext.fontset': 'cm',
        'font.size': 8.5, 'axes.titlesize': 8.5, 'axes.labelsize': 8.5, 'legend.fontsize': 8,
        'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5,
        'axes.edgecolor': INK2, 'axes.linewidth': 0.6, 'axes.labelcolor': INK, 'axes.titlecolor': INK,
        'axes.spines.top': False, 'axes.spines.right': False, 'axes.titlelocation': 'left',
        'axes.titlepad': 5,
        'xtick.color': INK2, 'ytick.color': INK2, 'xtick.major.width': 0.6, 'ytick.major.width': 0.6,
        'xtick.major.size': 2.8, 'ytick.major.size': 2.8, 'xtick.minor.size': 1.5, 'ytick.minor.size': 1.5,
        'xtick.minor.width': 0.5, 'ytick.minor.width': 0.5,
        'axes.grid': True, 'axes.grid.axis': 'y', 'grid.color': GRID, 'grid.linewidth': 0.5,
        'legend.frameon': False, 'legend.handlelength': 1.8, 'legend.columnspacing': 1.4,
        'lines.linewidth': 1.1, 'lines.solid_capstyle': 'round',
        'savefig.dpi': 300, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
        'pdf.fonttype': 42, 'image.interpolation': 'nearest',
    })
    return usetex


# ---------------------------------------------------------------- data helpers

def ok(runs, loc=None, noise=None):
    return [r for r in runs if r['status'] == 'ok' and (loc is None or r['localizer'] == loc)
            and (noise is None or r['noise'] == noise)]


def med(runs, key, scale=1.0):
    return sr.median([r[key] * scale for r in runs])


def lost_txt(runs_all, loc, noise=None):
    sel = [r for r in runs_all if r['localizer'] == loc and (noise is None or r['noise'] == noise)]
    good = [r for r in sel if r['status'] == 'ok']
    failed = len(sel) - len(good)
    return f"{sum(r['lost'] for r in good)}/{len(good)}" + (f" (+{failed} failed)" if failed else '')


def fmt(v, digits=1):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return '--'
    return f'{v:.0f}' if abs(v) >= 100 else f'{v:.{digits}f}'


def run_csv(r):
    """Per-sample CSV of a run (true and estimated pose, error)."""
    noise = f"_noise{r['noise']}" if float(r['noise']) > 0 else ''
    path = pr.RESULTS / r['run_dir'] / f"{r['localizer']}_speed_{r['speed']}{noise}_run{r['run']}.csv"
    data = np.genfromtxt(path, delimiter=',', names=True)
    return {k: data[k] for k in data.dtype.names}


def walls_rgba(yaml_name='victoria_outline.yaml', wall=INK2, track='#f3f3f3'):
    """Map image for backgrounds: walls dark, free track light, everything else transparent."""
    occ, free, info = load_occupancy(str(MAPS / yaml_name))
    img = np.zeros(occ.shape + (4,))
    img[free] = mpl.colors.to_rgba(track)
    img[occ] = mpl.colors.to_rgba(wall)
    h, w = occ.shape
    res, (ox, oy) = info['resolution'], info['origin'][:2]
    return img, (ox, ox + w * res, oy, oy + h * res)


def map_panel(ax, view, img=None, extent=None):
    if img is None:
        img, extent = walls_rgba()
    ax.imshow(img, extent=extent, zorder=0)
    ax.set_xlim(view[0], view[1])
    ax.set_ylim(view[2], view[3])
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    for s in ax.spines.values():
        s.set_visible(False)


def scale_bar(ax, view, length=2.0, label=r'2\,m', right=False):
    x0 = view[1] - 0.05 * (view[1] - view[0]) - length if right else view[0] + 0.05 * (view[1] - view[0])
    y0 = view[2] + 0.05 * (view[3] - view[2])
    ax.plot([x0, x0 + length], [y0, y0], color=INK, lw=1.6, solid_capstyle='butt', zorder=6)
    ax.annotate(label, (x0 + length / 2, y0), xytext=(0, 2.5), textcoords='offset points', ha='center',
                va='bottom', fontsize=7, color=INK)


def log_y(ax):
    ax.set_yscale('log')
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:g}'))
    ax.yaxis.set_minor_formatter(NullFormatter())


def corner_spans(cx, cy, n_laps):
    """(name, start, end) in laps of every pass through the corners of plot_results.CORNERS."""
    out = []
    for name, (x0, x1, y0, y1) in pr.CORNERS:
        inside = np.tile((cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1), n_laps).astype(int)
        change = np.diff(np.r_[0, inside, 0])
        out += [(name, a / sr.BINS_PER_LAP, b / sr.BINS_PER_LAP)
                for a, b in zip(np.nonzero(change == 1)[0], np.nonzero(change == -1)[0])]
    return out


# ---------------------------------------------------------------- shared plot pieces

def series_plot(ax, series, key, scale, xpos, log=True):
    """series: dicts (label, color, runs, dashed). A dot per run, a line through the medians, x = lost track."""
    n = len(series)
    for i, s in enumerate(series):
        dx = (i - (n - 1) / 2) * s.get('dodge', 0.0)
        runs = [r for r in s['runs'] if not math.isnan(r[key]) and (r[key] > 0 or not log)]
        xs = sorted({xpos(r) for r in runs})
        meds = [med([r for r in runs if xpos(r) == x], key, scale) for x in xs]
        ax.plot(np.array(xs) + dx, meds, color=s['color'], lw=1.1, ls=(0, (4, 2)) if s.get('dashed') else '-',
                zorder=2)
        for r in runs:
            x, y = xpos(r) + dx, r[key] * scale
            if r['lost']:
                ax.plot(x, y, marker='x', ms=4.2, mew=1.0, color=s['color'], ls='none', zorder=4)
            elif s.get('markers', True):
                ax.plot(x, y, marker='o', ms=3.0, mew=0.8, mec=s['color'],
                        mfc='white' if s.get('dashed') else s['color'], ls='none', zorder=3)
    if log:
        log_y(ax)


def series_handles(series, lost=True):
    h = [Line2D([], [], color=s['color'], lw=1.1, ls=(0, (4, 2)) if s.get('dashed') else '-',
                marker='o' if s.get('markers', True) else None, ms=3.0, mew=0.8, mec=s['color'],
                mfc='white' if s.get('dashed') else s['color'], label=s['label']) for s in series]
    if lost:
        h.append(Line2D([], [], color=INK2, marker='x', ls='none', ms=4.2, mew=1.0, label='lost track'))
        h.append(LOST_HANDLE)
    return h


LOST_HANDLE = Line2D([], [], color=INK2, lw=0.8, ls=(0, (1, 1.5)), label=r'0.5\,m lost-track threshold')


def lost_line(ax, y=50):
    ax.axhline(y, color=INK2, lw=0.6, ls=(0, (1, 1.5)), zorder=1)


def speed_figure(series, name, title_pos='Position error', title_head='Heading error', lost=True, log=True):
    """Two panels over commanded speed: position RMSE and heading RMSE (log scales unless log=False)."""
    fig, axes = plt.subplots(1, 2, figsize=(W, 2.45))
    xpos = lambda r: float(r['speed'])  # noqa: E731
    for ax, key, scale, title, unit in [(axes[0], 'pos_rmse', 100, title_pos, 'RMSE [cm]'),
                                        (axes[1], 'yaw_rmse', 1, title_head, r'RMSE [$^\circ$]')]:
        series_plot(ax, series, key, scale, xpos, log=log)
        if not log:
            ax.set_ylim(0, None)
        if key == 'pos_rmse' and lost:
            lost_line(ax)
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.set_xlabel('Commanded speed [m/s]')
        ax.set_xticks(np.arange(0.2, 1.01, 0.2))
        ax.set_xlim(0.16, 1.04)
    fig.legend(handles=series_handles(series, lost), loc='lower center', ncol=len(series) + 2 * lost,
               bbox_to_anchor=(0.5, 1.0), fontsize=7.5)
    fig.tight_layout(w_pad=2.5)
    save(fig, name)


written = []


def save(fig, name):
    fig.savefig(FIG / f'{name}.pdf')
    plt.close(fig)
    written.append(f'figures/{name}.pdf')


# ---------------------------------------------------------------- figures

def fig_track(cx, cy):
    img, extent = walls_rgba()
    occ, free, info = load_occupancy(str(MAPS / 'victoria_outline.yaml'))
    rows, cols = np.nonzero(free)
    res = info['resolution']
    view = (extent[0] + cols.min() * res - 0.5, extent[0] + (cols.max() + 1) * res + 0.5,
            extent[3] - (rows.max() + 1) * res - 0.5, extent[3] - rows.min() * res + 0.5)
    fig, ax = plt.subplots(figsize=(4.6, 4.6 * (view[3] - view[2]) / (view[1] - view[0])))
    map_panel(ax, view, img, extent)
    for name, (x0, x1, y0, y1) in pr.CORNERS:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc='#9fb4d3', ec='#7f9bc4', lw=0.6, alpha=0.3,
                               zorder=2))
        below = name == 'hairpin'  # the hairpin's box touches the outer wall at the top
        ax.annotate(name.capitalize(), ((x0 + x1) / 2, y0 if below else y1), xytext=(0, -3 if below else 3),
                    textcoords='offset points', ha='center', va='top' if below else 'bottom', fontsize=7.5,
                    color=INK2)
    px, py = np.r_[cx, cx[:1]], np.r_[cy, cy[:1]]
    ax.plot(px, py, color=INK, lw=0.9, ls=(0, (3, 2)), zorder=3)
    ax.plot(px[0], py[0], marker='o', ms=4.5, color=INK, zorder=5)
    ax.annotate('', xy=(px[4], py[4]), xytext=(px[0], py[0]), zorder=5,
                arrowprops=dict(arrowstyle='-|>', color=INK, lw=1.0, mutation_scale=9))
    ax.annotate('Start', (px[0], py[0]), xytext=(-4, -9), textcoords='offset points', ha='right', fontsize=7.5,
                color=INK)
    scale_bar(ax, view, right=True)
    ax.legend(handles=[Line2D([], [], color=INK, lw=0.9, ls=(0, (3, 2)), label=r'Driven line (36.8\,m per lap)'),
                       Patch(fc=INK2, label='Walls')], loc='upper left', fontsize=7)
    save(fig, 'track')


def fig_track_maps(runs, name):
    """Median error along the lap per localizer, and where runs first lost track."""
    good = ok(runs)
    cx, cy = sr.centerline(good)
    px, py = np.r_[cx, cx[:1]], np.r_[cy, cy[:1]]
    seg = np.stack([np.column_stack([px[:-1], py[:-1]]), np.column_stack([px[1:], py[1:]])], axis=1)
    img, extent = walls_rgba(wall='#9a9a9a', track='#f6f6f6')
    occ, free, info = load_occupancy(str(MAPS / 'victoria_outline.yaml'))
    rows, cols = np.nonzero(free)
    res = info['resolution']
    view = (extent[0] + cols.min() * res - 0.3, extent[0] + (cols.max() + 1) * res + 0.3,
            extent[3] - (rows.max() + 1) * res - 0.3, extent[3] - rows.min() * res + 0.3)
    fig, axes = plt.subplots(1, 3, figsize=(W, 2.55))
    for ax, loc in zip(axes, LOCS):
        mine = ok(runs, loc)
        err = sr.nanmedian(np.vstack([sr.binned(r, 'pos') for r in mine]))
        map_panel(ax, view, img, extent)
        line = LineCollection(seg, cmap=sr.ERR_CMAP.with_extremes(bad=FAINT), norm=sr.ERR_NORM, linewidths=3.2,
                              capstyle='round', zorder=2)
        line.set_array(np.ma.masked_invalid(err))
        ax.add_collection(line)
        points = [sr.lost_at(r)[:2] for r in mine if r['lost']]
        for lx, ly in points:
            ax.plot(lx, ly, marker='x', ms=5, mew=1.2, color=INK, ls='none', zorder=5)
        for (lx, ly), n in sr.clusters(points):
            ax.annotate(f'{n}', (lx, ly), xytext=(5, 4), textcoords='offset points', fontsize=7, color=INK, zorder=6)
        lost = sum(r['lost'] for r in mine)
        ax.set_title(NAME[loc], pad=12)
        ax.annotate(f'{lost} of {len(mine)} runs lost track', (0, 1), xycoords='axes fraction', xytext=(0, 1),
                    textcoords='offset points', va='bottom', fontsize=7, color=INK2)
    fig.tight_layout(rect=(0, 0.12, 1, 1), w_pad=0.5)
    cax = fig.add_axes((0.3, 0.115, 0.4, 0.035))
    error_colorbar(fig, cax)
    save(fig, name)


def error_colorbar(fig, cax, label='Position error (median over runs and laps), log scale'):
    cb = fig.colorbar(mpl.cm.ScalarMappable(sr.ERR_NORM, sr.ERR_CMAP), cax=cax, orientation='horizontal',
                      extend='both')
    ticks = [0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5]
    cb.set_ticks(ticks)
    cb.set_ticklabels([rf'{v * 100:g}\,cm' if v < 1 else rf'{v:g}\,m' for v in ticks])
    cax.minorticks_off()
    cax.tick_params(length=0, labelsize=6.5)
    cb.outline.set_visible(False)
    cax.axvline(0.5, color=INK, lw=1.0)
    cb.set_label(label, fontsize=7, color=INK2, labelpad=2)


def fig_heatmap(runs, name):
    """Error along the 3 laps, one row per speed, for each localizer."""
    good = [r for r in ok(runs) if r['n_laps'] == 3]
    cx, cy = sr.centerline(good)
    edges = np.arange(3 * sr.BINS_PER_LAP + 1) / sr.BINS_PER_LAP
    fig, axes = plt.subplots(3, 1, figsize=(W, 6.1), sharex=True)
    for ax, loc in zip(axes, LOCS):
        mine = sorted(ok(good, loc), key=lambda r: float(r['speed']))
        grid = np.vstack([sr.binned(r, 'pos').ravel() for r in mine])
        grid[grid <= 0] = sr.ERR_NORM.vmin / 10
        ax.grid(False)
        ax.pcolormesh(edges, np.arange(len(mine) + 1), np.ma.masked_invalid(grid), shading='flat', rasterized=True,
                      cmap=sr.ERR_CMAP.with_extremes(bad=INK2), norm=sr.ERR_NORM)
        for i, r in enumerate(mine):
            if r['lost']:
                ax.plot(sr.lost_at(r)[2], i + 0.5, marker='x', ms=4, mew=1.1, color='white', ls='none', zorder=4)
        for k in (1, 2):
            ax.axvline(k, color='white', lw=1.6, zorder=3)
        ax.set_yticks(np.arange(len(mine)) + 0.5, [f"{float(r['speed']):.2f}" for r in mine])
        ax.tick_params(axis='y', labelsize=6, length=0)
        ax.set_ylabel('Speed [m/s]')
        ax.set_title(f"{NAME[loc]}: {sum(r['lost'] for r in mine)} of {len(mine)} runs lost track")
        for s in ax.spines.values():
            s.set_visible(False)
    for nm, a, b in corner_spans(cx, cy, 3):
        axes[0].plot([a, b], [1.03, 1.03], transform=axes[0].get_xaxis_transform(), color=INK2, lw=2.2,
                     solid_capstyle='butt', clip_on=False)
        if a < 1:
            axes[0].annotate(nm, ((a + b) / 2, 1.03), xycoords=('data', 'axes fraction'), xytext=(0, 2.5),
                             textcoords='offset points', ha='center', va='bottom', fontsize=6.5, color=INK2)
    axes[0].set_title(axes[0].get_title(loc='left'), pad=19)
    axes[-1].set_xticks(np.arange(0, 3.01, 0.5))
    axes[-1].set_xlim(0, 3)
    axes[-1].set_xlabel('Laps driven (white lines: start of the next lap)')
    fig.tight_layout(rect=(0, 0.075, 1, 1), h_pad=1.2)
    cax = fig.add_axes((0.3, 0.035, 0.4, 0.014))
    error_colorbar(fig, cax, 'Position error (median per 0.37\\,m of track), log scale')
    save(fig, name)


def hit_field(yaml_name):
    """AMCL's likelihood-field hit term for every map cell: exp(-d^2 / 2 sigma^2), d = distance to occupied."""
    occ, _, info = load_occupancy(str(pr.MAP_YAML if yaml_name == 'Victoria.yaml' else MAPS / yaml_name))
    d = np.minimum(distance_transform_edt(~occ) * info['resolution'], AMCL_MAX_DIST)
    h, w = occ.shape
    res, (ox, oy) = info['resolution'], info['origin'][:2]
    return np.exp(-d ** 2 / (2 * AMCL_SIGMA_HIT ** 2)), occ, (ox, ox + w * res, oy, oy + h * res), info


def fig_amcl_maps(cx, cy):
    view = (2.9, 7.7, 0.0, 5.0)
    _, walls, extent, info = hit_field('victoria_outline.yaml')
    res = info['resolution']
    cell = lambda x, y: (int((extent[3] - y) / res), int((x - extent[0]) / res))  # noqa: E731
    # probe: the car at the hairpin's apex (rightmost point of the driven line), a beam straight outwards
    # (+x) that passes the outer wall and ends 0.4 m behind it, in the filled map's black area
    i = int(np.argmax(cx))
    car = (cx[i], cy[i])
    t = 0.0
    while not walls[cell(car[0] + t, car[1])]:
        t += res / 2
    while walls[cell(car[0] + t, car[1])]:
        t += res / 2
    end = (car[0] + t + 0.4, car[1])
    fig, axes = plt.subplots(1, 2, figsize=(W, 3.35))
    for ax, yaml_name, title in [(axes[0], 'Victoria.yaml', r'(a) Filled map (\texttt{Victoria.yaml})'),
                                 (axes[1], 'victoria_outline.yaml', r'(b) Outline map (\texttt{victoria\_outline})')]:
        p, occ, ext, _ = hit_field(yaml_name)
        ax.imshow(p, extent=ext, cmap=HIT_CMAP, vmin=0, vmax=1, zorder=0)
        overlay = np.zeros(walls.shape + (4,))
        overlay[walls] = mpl.colors.to_rgba(INK)
        ax.imshow(overlay, extent=extent, zorder=1)
        ax.set_xlim(view[0], view[1])
        ax.set_ylim(view[2], view[3])
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(False)
        for s_ in ax.spines.values():
            s_.set_visible(False)
        ax.set_title(title)
        ax.plot([car[0], end[0]], [car[1], end[1]], color='#e8a33d', lw=1.1, zorder=3)
        ax.plot(*car, marker='o', ms=3.5, color=INK, zorder=4)
        ax.plot(*end, marker='x', ms=5, mew=1.3, color='#e8a33d', zorder=4)
        ax.annotate(rf'beam end 0.4\,m behind the wall: $p_\mathrm{{hit}} = {p[cell(*end)]:.2f}$', end,
                    xytext=(-118, -22), textcoords='offset points', fontsize=7, color=INK,
                    bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='none', alpha=0.9))
        ax.annotate('car', car, xytext=(-4, 5), textcoords='offset points', ha='right', fontsize=7, color=INK)
    fig.tight_layout(rect=(0, 0.1, 1, 1), w_pad=1.5)
    cax = fig.add_axes((0.3, 0.07, 0.4, 0.025))
    cb = fig.colorbar(mpl.cm.ScalarMappable(mpl.colors.Normalize(0, 1), HIT_CMAP), cax=cax, orientation='horizontal')
    cb.outline.set_visible(False)
    cax.tick_params(length=0, labelsize=6.5)
    cb.set_label(r'Likelihood-field hit term $p_\mathrm{hit} = \exp(-d^2/2\sigma^2)$, $\sigma = 0.2$\,m, '
                 r'$d$ = distance to the nearest occupied cell', fontsize=7, color=INK2, labelpad=2)
    save(fig, 'amcl_maps')


def fig_amcl_lost_run(runs):
    r = next(r for r in ok(runs, 'amcl') if r['speed'] == '0.60')
    d = run_csv(r)
    t = d['t'] - d['t'][0]
    img, extent = pr.load_map()  # the filled map AMCL used in this run
    gray = np.where(img[..., None] > 200, mpl.colors.to_rgba('#f6f6f6'), mpl.colors.to_rgba('#b9b9b9'))
    fig = plt.figure(figsize=(W, 2.75))
    ax = fig.add_axes((0.0, 0.04, 0.5, 0.88))
    view = (-10.2, 8.4, -5.0, 7.6)
    map_panel(ax, view, gray, extent)
    ax.plot(d['gt_x'], d['gt_y'], color=INK, lw=0.7, zorder=2, label='True path')
    ax.plot(d['slam_x'], d['slam_y'], color=COLOR['amcl'], lw=0.8, zorder=3, label='AMCL estimate')
    i = int(np.argmax(d['pos_err'] > pr.LOST))
    ax.annotate('', xy=(d['slam_x'][i + 25], d['slam_y'][i + 25]), xytext=(d['gt_x'][i], d['gt_y'][i]), zorder=5,
                arrowprops=dict(arrowstyle='-|>', color=INK, lw=0.9, mutation_scale=8))
    ax.annotate(rf'jump at $t = {t[i]:.0f}$\,s', (d['gt_x'][i], d['gt_y'][i]), xytext=(-62, 8),
                textcoords='offset points', fontsize=7, color=INK)
    ax.legend(loc='upper left', fontsize=7, handlelength=1.4)
    ax.set_title(r'(a) AMCL on the filled map, 0.60\,m/s, 3 laps')
    scale_bar(ax, view)
    ax2 = fig.add_axes((0.6, 0.17, 0.39, 0.72))
    ax2.plot(t, d['pos_err'] * 100, color=COLOR['amcl'], lw=0.8)
    log_y(ax2)
    lost_line(ax2)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position error [cm]')
    ax2.set_title('(b) Position error over the run')
    ax2.set_xlim(0, t[-1])
    save(fig, 'amcl_lost_run')


def slam_map_pixels(name, dist, gres, gox, goy):
    occ, _, info = load_occupancy(str(MAPS / f'{name}.yaml'))
    occ = occ[::-1]
    iy, ix = np.nonzero(occ)
    r, (ox, oy) = info['resolution'], info['origin'][:2]
    x, y = ox + (ix + 0.5) * r, oy + (iy + 0.5) * r
    d = dist[np.clip(((y - goy) / gres).astype(int), 0, dist.shape[0] - 1),
             np.clip(((x - gox) / gres).astype(int), 0, dist.shape[1] - 1)]
    return x, y, d


def fig_slam_maps(numbers):
    gt, _, gi = load_occupancy(str(MAPS / 'victoria_outline.yaml'))
    gt = gt[::-1]
    gres, (gox, goy) = gi['resolution'], gi['origin'][:2]
    dist = distance_transform_edt(~gt) * gres
    gy, gx = np.nonzero(gt)
    wx, wy = gox + (gx + 0.5) * gres, goy + (gy + 0.5) * gres
    view = (-9.2, -2.3, -3.8, 2.4)
    fig, axes = plt.subplots(1, 2, figsize=(W, 3.15))
    x0, x1, y0, y1 = START_ZONE
    for ax, name, title in [(axes[0], 'victoria_ref', r'(a) \texttt{victoria\_ref}: mapped without loop closure'),
                            (axes[1], 'victoria_ref_lc', r'(b) \texttt{victoria\_ref\_lc}: mapped with loop closure')]:
        x, y, d = slam_map_pixels(name, dist, gres, gox, goy)
        zone = (x > x0) & (x < x1) & (y > y0) & (y < y1)
        share = np.mean(d[zone] < 0.10) * 100
        numbers[f'{name}-start-within10'] = f'{share:.0f}'
        numbers[f'{name}-rest-within10'] = f'{np.mean(d[~zone] < 0.10) * 100:.0f}'
        m = (wx > view[0]) & (wx < view[1]) & (wy > view[2]) & (wy < view[3])
        ax.scatter(wx[m], wy[m], s=1.6, marker='s', lw=0, color=FAINT, zorder=1, rasterized=True)
        m = (x > view[0]) & (x < view[1]) & (y > view[2]) & (y < view[3])
        good = m & (d <= 0.10)
        bad = m & (d > 0.10)
        ax.scatter(x[good], y[good], s=1.6, marker='s', lw=0, color=INK, zorder=2, rasterized=True)
        ax.scatter(x[bad], y[bad], s=1.6, marker='s', lw=0, color=BAD, zorder=3, rasterized=True)
        ax.set_xlim(view[0], view[1])
        ax.set_ylim(view[2], view[3])
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(False)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.annotate(rf'start area: {share:.0f}\,\% of the wall pixels within 10\,cm', (0, 1),
                    xycoords='axes fraction', xytext=(0, 1), textcoords='offset points', va='bottom', fontsize=7,
                    color=INK2)
        ax.set_title(title, pad=12)
        scale_bar(ax, view, 1.0, r'1\,m', right=True)
    fig.legend(handles=[Patch(fc=FAINT, label='True walls'),
                        Patch(fc=INK, label=r'SLAM map: within 10\,cm of a true wall'),
                        Patch(fc=BAD, label=r'SLAM map: more than 10\,cm off')],
               loc='lower center', ncol=3, bbox_to_anchor=(0.5, -0.01))
    fig.tight_layout(rect=(0, 0.07, 1, 1), w_pad=1.5)
    save(fig, 'slam_maps')


def fig_odom_drift(runs):
    good = ok(runs, 'odom')
    img, extent = walls_rgba(wall='#a8a8a8', track='#f6f6f6')
    fig, axes = plt.subplots(2, 2, figsize=(W, 5.0))
    for ax, level in zip(axes.ravel(), ['1', '2', '4', '8']):
        r = next(r for r in good if r['noise'] == level and r['run'] == 1)
        d = run_csv(r)
        allx, ally = np.r_[d['gt_x'], d['slam_x']], np.r_[d['gt_y'], d['slam_y']]
        cxm, cym = (allx.min() + allx.max()) / 2, (ally.min() + ally.max()) / 2
        half = max(allx.max() - allx.min(), (ally.max() - ally.min()) * 1.3) / 2 + 0.8
        view = (cxm - half, cxm + half, cym - half / 1.3, cym + half / 1.3)
        map_panel(ax, view, img, extent)
        ax.plot(d['gt_x'], d['gt_y'], color=INK, lw=0.7, zorder=2)
        ax.plot(d['slam_x'], d['slam_y'], color=INK2, lw=0.8, ls=(0, (3, 1.5)), zorder=3)
        ax.plot(d['gt_x'][0], d['gt_y'][0], marker='o', ms=3.5, color=INK, zorder=4)
        ax.set_title(rf"Noise level {level}: RMSE {r['pos_rmse']:.1f}\,m, worst {r['pos_max']:.1f}\,m")
        ax.set_frame_on(True)
        for s in ax.spines.values():
            s.set_visible(True)
            s.set_color(GRID)
        scale_bar(ax, view, 2.0 if half < 8 else 5.0, r'2\,m' if half < 8 else r'5\,m')
    fig.legend(handles=[Line2D([], [], color=INK, lw=0.7, label='True path (3 laps)'),
                        Line2D([], [], color=INK2, lw=0.8, ls=(0, (3, 1.5)), label='Odometry alone (seed 1)'),
                        Line2D([], [], color=INK, marker='o', ls='none', ms=3.5, label='Start')],
               loc='lower center', ncol=3, bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.04, 1, 1), h_pad=1.0, w_pad=1.0)
    save(fig, 'odom_drift')


def fig_noise_rmse(runs):
    xpos = lambda r: float(LEVELS.index(r['noise']))  # noqa: E731
    series = [dict(label=NAME[k], color=COLOR[k], runs=ok(runs, k), dodge=0.11) for k in LOCS]
    series.append(dict(label=r'Odometry only (no localizer)', color=COLOR['odom'], runs=ok(runs, 'odom'),
                       dashed=True, markers=False, dodge=0.11))
    fig, axes = plt.subplots(1, 2, figsize=(W, 2.6))
    for ax, key, scale, title, unit in [(axes[0], 'pos_rmse', 100, 'Position error', 'RMSE [cm]'),
                                        (axes[1], 'yaw_rmse', 1, 'Heading error', r'RMSE [$^\circ$]')]:
        series_plot(ax, series, key, scale, xpos)
        if key == 'pos_rmse':
            lost_line(ax)
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.set_xticks(range(len(LEVELS)), LEVELS)
        ax.set_xlim(-0.4, len(LEVELS) - 0.6)
        ax.set_xlabel(r'Odometry noise level (0 = perfect, 1 $\approx$ a real car)')
    fig.legend(handles=series_handles(series), loc='lower center', ncol=3, bbox_to_anchor=(0.5, 1.0), fontsize=7.5)
    fig.tight_layout(w_pad=2.5)
    save(fig, 'noise_rmse')


def fig_noise_along_lap(runs):
    good = [r for r in ok(runs) if r['n_laps'] == 3]
    cx, cy = sr.centerline(good)
    x = (np.arange(3 * sr.BINS_PER_LAP) + 0.5) / sr.BINS_PER_LAP
    fig, axes = plt.subplots(2, 1, figsize=(W, 4.1), sharex=True)
    for ax, level in zip(axes, ['4', '8']):
        for nm, a, b in corner_spans(cx, cy, 3):
            ax.axvspan(a, b, color='#eef2f8', lw=0, zorder=0)
        for loc in LOCS + ['odom']:
            mine = [r for r in good if r['localizer'] == loc and r['noise'] == level]
            curve = sr.nanmedian(np.vstack([sr.binned(r, 'pos').ravel() for r in mine])) * 100
            ax.plot(x, curve, color=COLOR[loc], lw=1.0 if loc != 'odom' else 0.9,
                    ls=(0, (4, 2)) if loc == 'odom' else '-', zorder=3)
        log_y(ax)
        lost_line(ax)
        for k in (1, 2):
            ax.axvline(k, color=INK2, lw=0.5, zorder=1)
        ax.set_ylabel('Position error [cm]')
        ax.set_title(f'Noise level {level} (median of the three seeds)', pad=12 if level == '4' else 5)
        ax.set_ylim(1, 3000)
    for nm, a, b in corner_spans(cx, cy, 1):
        axes[0].annotate(nm, ((a + b) / 2, 1), xycoords=('data', 'axes fraction'), xytext=(0, 1),
                         textcoords='offset points', ha='center', va='bottom', fontsize=6.5, color=INK2)
    axes[-1].set_xlim(0, 3)
    axes[-1].set_xticks(np.arange(0, 3.01, 0.5))
    axes[-1].set_xlabel('Laps driven (shaded: hairpin and left-hand corner)')
    handles = [Line2D([], [], color=COLOR[k], lw=1.5, label=NAME[k]) for k in LOCS]
    handles += [Line2D([], [], color=COLOR['odom'], lw=1.2, ls=(0, (4, 2)), label='Odometry only'), LOST_HANDLE]
    fig.legend(handles=handles, loc='lower center', ncol=5, bbox_to_anchor=(0.5, 1.0), fontsize=7.5)
    fig.tight_layout(h_pad=1.2)
    save(fig, 'noise_along_lap')



def fig_fixes(speed, amcl, slam):
    """Short report: both fixes in one figure, position RMSE over speed before and after."""
    fig, axes = plt.subplots(1, 2, figsize=(W, 2.45))
    xpos = lambda r: float(r['speed'])  # noqa: E731
    for ax, loc, before, after, title, lb, la in [
            (axes[0], 'amcl', ok(speed, 'amcl'), ok(amcl, 'amcl'), '(a) AMCL', 'filled map (before)',
             'outline map (after)'),
            (axes[1], 'slam', ok(speed, 'slam'), ok(slam, 'slam'), '(b) SLAM Toolbox', 'loop closure on (before)',
             'loop closure off (after)')]:
        series = [dict(label=lb, color=COLOR[loc], runs=before, dashed=True),
                  dict(label=la, color=COLOR[loc], runs=after)]
        series_plot(ax, series, 'pos_rmse', 100, xpos)
        lost_line(ax)
        ax.set_title(title, pad=15)
        ax.set_ylabel('Position RMSE [cm]')
        ax.set_xlabel('Commanded speed [m/s]')
        ax.set_xticks(np.arange(0.2, 1.01, 0.2))
        ax.set_xlim(0.16, 1.04)
        ax.set_ylim(2.5, 2500)
        ax.legend(handles=series_handles(series, lost=False), loc='lower left', bbox_to_anchor=(0, 1.0), ncol=2,
                  fontsize=7, borderaxespad=0.2, handlelength=1.6, columnspacing=1.0)
    fig.tight_layout(w_pad=2.5)
    save(fig, 'fixes_before_after')


def fig_amcl_maps_compact(cx, cy):
    """Short report: the likelihood-field figure in a lower, narrower layout."""
    view = (2.9, 7.7, 0.0, 5.0)
    _, walls, extent, info = hit_field('victoria_outline.yaml')
    res = info['resolution']
    cell = lambda x, y: (int((extent[3] - y) / res), int((x - extent[0]) / res))  # noqa: E731
    i = int(np.argmax(cx))
    car = (cx[i], cy[i])
    t = 0.0
    while not walls[cell(car[0] + t, car[1])]:
        t += res / 2
    while walls[cell(car[0] + t, car[1])]:
        t += res / 2
    end = (car[0] + t + 0.4, car[1])
    fig = plt.figure(figsize=(W, 2.45))
    panels = [(fig.add_axes((0.0, 0.0, 0.38, 0.88)), 'Victoria.yaml', r'(a) Filled map'),
              (fig.add_axes((0.36, 0.0, 0.38, 0.88)), 'victoria_outline.yaml', r'(b) Outline map')]
    for ax, yaml_name, title in panels:
        p, occ, ext, _ = hit_field(yaml_name)
        ax.imshow(p, extent=ext, cmap=HIT_CMAP, vmin=0, vmax=1, zorder=0)
        overlay = np.zeros(walls.shape + (4,))
        overlay[walls] = mpl.colors.to_rgba(INK)
        ax.imshow(overlay, extent=extent, zorder=1)
        ax.set_xlim(view[0], view[1])
        ax.set_ylim(view[2], view[3])
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(False)
        for s_ in ax.spines.values():
            s_.set_visible(False)
        ax.set_title(title, loc='center')
        ax.plot([car[0], end[0]], [car[1], end[1]], color='#e8a33d', lw=1.1, zorder=3)
        ax.plot(*car, marker='o', ms=3.5, color=INK, zorder=4)
        ax.plot(*end, marker='x', ms=5, mew=1.3, color='#e8a33d', zorder=4)
        ax.annotate(rf'$p_\mathrm{{hit}} = {p[cell(*end)]:.2f}$', end, xytext=(-14, -15), textcoords='offset points',
                    fontsize=7.5, color=INK, bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='none', alpha=0.9))
        ax.annotate('car', car, xytext=(-4, 5), textcoords='offset points', ha='right', fontsize=7, color=INK)
    cax = fig.add_axes((0.74, 0.1, 0.014, 0.7))
    cb = fig.colorbar(mpl.cm.ScalarMappable(mpl.colors.Normalize(0, 1), HIT_CMAP), cax=cax)
    cb.outline.set_visible(False)
    cax.tick_params(length=0, labelsize=6.5)
    cb.set_label(r'$p_\mathrm{hit} = \exp(-d^2/2\sigma^2)$', fontsize=7.5, color=INK2)
    save(fig, 'amcl_maps_compact')



def fig_heatmap_fixes(speed, amcl, slam):
    """Error along the 3 laps for every speed, before and after each fix (AMCL, SLAM Toolbox)."""
    speeds = sorted({r['speed'] for r in ok(speed)}, key=float)
    good = [r for r in ok(amcl) + ok(slam) if r['n_laps'] == 3]
    cx, cy = sr.centerline(good)
    edges = np.arange(3 * sr.BINS_PER_LAP + 1) / sr.BINS_PER_LAP
    cmap = sr.ERR_CMAP.with_extremes(bad='#ececec')
    panels = [('amcl', ok(speed, 'amcl'), 'AMCL, filled map (before)'),
              ('amcl', ok(amcl, 'amcl'), 'AMCL, outline map (after)'),
              ('slam', ok(speed, 'slam'), 'SLAM Toolbox, loop closure on (before)'),
              ('slam', ok(slam, 'slam'), 'SLAM Toolbox, loop closure off (after)')]
    fig, axes = plt.subplots(2, 2, figsize=(W, 4.15), sharex=True, sharey=True)
    for ax, (loc, runs, title) in zip(axes.ravel(), panels):
        by_speed = {r['speed']: r for r in runs if r['localizer'] == loc and r['n_laps'] == 3}
        grid = np.full((len(speeds), 3 * sr.BINS_PER_LAP), np.nan)
        for i, sp in enumerate(speeds):
            if sp in by_speed:
                grid[i] = sr.binned(by_speed[sp], 'pos').ravel()
        grid[grid <= 0] = sr.ERR_NORM.vmin / 10
        ax.grid(False)
        ax.pcolormesh(edges, np.arange(len(speeds) + 1), np.ma.masked_invalid(grid), shading='flat',
                      rasterized=True, cmap=cmap, norm=sr.ERR_NORM)
        for i, sp in enumerate(speeds):
            r = by_speed.get(sp)
            if r is not None and r['lost']:
                ax.plot(sr.lost_at(r)[2], i + 0.5, marker='x', ms=3.6, mew=1.0, color='white', ls='none', zorder=4)
        for k in (1, 2):
            ax.axvline(k, color='white', lw=1.4, zorder=3)
        lost = sum(r['lost'] for r in by_speed.values())
        missing = len(speeds) - len(by_speed)
        ax.set_title(title, pad=12)
        ax.annotate(f'{lost} of {len(by_speed)} runs lost track' + (f', {missing} failed to start' if missing else ''),
                    (0, 1), xycoords='axes fraction', xytext=(0, 2), textcoords='offset points', va='bottom',
                    fontsize=7, color=INK2)
        for s_ in ax.spines.values():
            s_.set_visible(False)
        ax.tick_params(axis='y', labelsize=6, length=0)
    ticks = [i for i, sp in enumerate(speeds) if round(float(sp) * 100) % 20 == 0]
    axes[0, 0].set_yticks([i + 0.5 for i in ticks], [f'{float(speeds[i]):.1f}' for i in ticks])
    for ax in axes[:, 0]:
        ax.set_ylabel('Speed [m/s]')
    for ax in axes[1]:
        ax.set_xticks(np.arange(0, 3.01, 0.5))
        ax.set_xlim(0, 3)
        ax.set_xlabel('Laps driven')
    fig.tight_layout(rect=(0, 0.1, 1, 1), h_pad=1.6, w_pad=1.2)
    cax = fig.add_axes((0.3, 0.045, 0.4, 0.02))
    error_colorbar(fig, cax, r'Position error (median per 0.37\,m of track), log scale; light grey: no run')
    save(fig, 'heatmap_fixes')


# ---------------------------------------------------------------- tables

def write_table(name, colspec, header, rows, midrules=()):
    lines = [rf'\begin{{tabular}}{{{colspec}}}', r'\toprule'] + header + [r'\midrule']
    for i, row in enumerate(rows):
        if i in midrules:
            lines.append(r'\midrule')
        lines.append(' & '.join(row) + r' \\')
    lines += [r'\bottomrule', r'\end{tabular}', '']
    (TAB / f'{name}.tex').write_text('\n'.join(lines))
    written.append(f'tables/{name}.tex')


def metrics_cells(runs_all, loc, noise=None):
    g = ok(runs_all, loc, noise)
    return [fmt(med(g, 'pos_rmse', 100)), fmt(med(g, 'pos_p95', 100)), fmt(med(g, 'yaw_rmse')),
            lost_txt(runs_all, loc, noise)]


def th(*lines, align='r'):
    """Two-line table header cell (makecell)."""
    body = r' \\ '.join('{' + line + '}' for line in lines)  # braces: "\\ [cm]" would be read as \\[length]
    return rf'\makecell[{align}b]{{{body}}}'


HEAD4 = ' & '.join([th('RMSE', '[cm]'), th('p95', '[cm]'), th('Heading', r'RMSE [$^\circ$]'), th('Lost', 'track')])


def tables(speed, amcl, slam, noise, smoke, n):
    # original speed sweep
    maps = {'scan': r'filled \texttt{Victoria.yaml}', 'amcl': r'filled \texttt{Victoria.yaml}',
            'slam': r'\texttt{victoria\_ref}, loop closure on'}
    write_table('speed_sweep', 'llrrrr', [th('Localizer', align='l') + ' & ' + th('Map /', 'configuration', align='l')
                                          + ' & ' + HEAD4 + r' \\'],
                [[NAME[k], maps[k]] + metrics_cells(speed, k) for k in LOCS])
    # short report: every speed sweep in one table, before and after the fixes
    rows = [[NAME['scan'], r'filled map'] + metrics_cells(speed, 'scan'),
            [NAME['amcl'], r'filled map (before)'] + metrics_cells(speed, 'amcl'),
            ['', r'outline map (after)'] + metrics_cells(amcl, 'amcl'),
            [NAME['slam'], r'loop closure on, \texttt{victoria\_ref} (before)'] + metrics_cells(speed, 'slam'),
            ['', r'loop closure off, \texttt{victoria\_ref\_lc} (after)'] + metrics_cells(slam, 'slam')]
    write_table('sweep_overview', 'llrrrr', [th('Localizer', align='l') + ' & ' + th('Configuration', align='l')
                                             + ' & ' + HEAD4 + r' \\'], rows, midrules=(1, 3))
    # AMCL before / after
    write_table('amcl_before_after', 'lrrrrr',
                ['Map & ' + ' & '.join([th('RMSE', '[cm]'), th('p95', '[cm]'), th('Worst run', '[m]'),
                                        th('Heading', r'RMSE [$^\circ$]'), th('Lost', 'track')]) + r' \\'],
                [[r'Filled (\texttt{Victoria.yaml})'] + amcl_cells(speed),
                 [r'Outline (\texttt{victoria\_outline})'] + amcl_cells(amcl)])
    # SLAM Toolbox: map x loop closure, 1.0 m/s
    def one(runs, laps_note):
        r = next(r for r in ok(runs, 'slam') if r['speed'] == '1.0')
        return [fmt(r['pos_rmse'] * 100), fmt(r['pos_p95'] * 100), fmt(r['pos_max'] * 100), fmt(r['yaw_rmse']),
                fmt(r['yaw_max']), laps_note]
    rows = [[r'\texttt{victoria\_ref}', 'on'] + one(speed, '3'),
            [r'\texttt{victoria\_ref\_lc}', 'on'] + one(smoke['slam-lc-smoke'], '1'),
            [r'\texttt{victoria\_ref}', 'off'] + one(smoke['slam-ref-nolc-smoke'], '1'),
            [r'\texttt{victoria\_ref\_lc}', 'off'] + one(smoke['slam-lc-nolc-smoke'], '1')]
    write_table('slam_variants', 'llrrrrrr',
                ['Map & ' + ' & '.join([th('Loop', 'closure', align='l'), th('RMSE', '[cm]'), th('p95', '[cm]'),
                                        th('Max', '[cm]'), th('Heading', r'RMSE [$^\circ$]'),
                                        th('Heading', r'max [$^\circ$]'), 'Laps']) + r' \\'], rows)
    for row, key in zip(rows, ['ref-on', 'lc-on', 'ref-off', 'lc-off']):
        n[f'slam-{key}-rmse'], n[f'slam-{key}-max'] = row[2], row[4]
    # SLAM Toolbox before / after
    write_table('slam_before_after', 'lrrrr', [r'Configuration & ' + HEAD4 + r' \\'],
                [[r'\texttt{victoria\_ref}, loop closure on (before)'] + metrics_cells(speed, 'slam'),
                 [r'\texttt{victoria\_ref\_lc}, loop closure off (after)'] + metrics_cells(slam, 'slam')])
    # final comparison: sweep medians + noise level 0 (all final configurations, 1.0 m/s, 3 runs)
    final = {'scan': speed, 'amcl': amcl, 'slam': slam}
    fmaps = {'scan': r'filled${}^{a}$', 'amcl': 'outline', 'slam': r'\texttt{victoria\_ref\_lc}'}
    rows = []
    for k in LOCS:
        g = ok(noise, k, '0')
        rows.append([NAME[k], fmaps[k]] + metrics_cells(final[k], k)
                    + [fmt(med(g, 'pos_rmse', 100)), fmt(med(g, 'pos_p95', 100)), fmt(med(g, 'yaw_rmse'))])
    write_table('final_comparison', 'llrrrrrrr',
                [r' & & \multicolumn{4}{c}{Speed sweep, 17 speeds} & \multicolumn{3}{c}{1.0\,m/s, 3 runs${}^{b}$} \\',
                 r'\cmidrule(lr){3-6}\cmidrule(lr){7-9}',
                 'Localizer & Map & ' + ' & '.join([th('RMSE', '[cm]'), th('p95', '[cm]'), th('Heading', r'[$^\circ$]'),
                                                    'Lost', th('RMSE', '[cm]'), th('p95', '[cm]'),
                                                    th('Heading', r'[$^\circ$]')]) + r' \\'], rows)
    # noise results
    rows = []
    for lv in LEVELS:
        row = [lv]
        for k in LOCS:
            g = ok(noise, k, lv)
            row += [fmt(med(g, 'pos_rmse', 100)), fmt(med(g, 'pos_p95', 100)), fmt(med(g, 'yaw_rmse')),
                    lost_txt(noise, k, lv)]
        row.append(fmt(med(ok(noise, 'odom', lv), 'pos_rmse', 1), 2) if lv != '0' else '0')
        rows.append(row)
    write_table('noise_results', 'c' + 'rrrr' * 3 + 'r',
                [r' & \multicolumn{4}{c}{Scan matcher} & \multicolumn{4}{c}{AMCL} & \multicolumn{4}{c}{SLAM Toolbox} '
                 r'& Odom. only \\',
                 r'\cmidrule(lr){2-5}\cmidrule(lr){6-9}\cmidrule(lr){10-13}\cmidrule(lr){14-14}',
                 r'Level & RMSE & p95 & Hdg & Lost & RMSE & p95 & Hdg & Lost & RMSE & p95 & Hdg & Lost & RMSE [m] \\'],
                rows)
    # headline table for the summary box
    rows = []
    for k in LOCS:
        row = [NAME[k]]
        row += [fmt(med(ok(final[k], k), 'pos_rmse', 100)), lost_txt(final[k], k)]
        for lv in ('4', '8'):
            row += [fmt(med(ok(noise, k, lv), 'pos_rmse', 100)), lost_txt(noise, k, lv)]
        rows.append(row)
    write_table('headline', 'lrrrrrr',
                [r' & \multicolumn{2}{c}{Perfect odometry${}^{a}$} & \multicolumn{2}{c}{Noise level 4} & '
                 r'\multicolumn{2}{c}{Noise level 8} \\',
                 r'\cmidrule(lr){2-3}\cmidrule(lr){4-5}\cmidrule(lr){6-7}',
                 r'Localizer & RMSE [cm] & Lost & RMSE [cm] & Lost & RMSE [cm] & Lost \\'], rows)


def amcl_cells(runs):
    g = ok(runs, 'amcl')
    return [fmt(med(g, 'pos_rmse', 100)), fmt(med(g, 'pos_p95', 100)), fmt(max(r['pos_max'] for r in g), 1),
            fmt(med(g, 'yaw_rmse')), lost_txt(runs, 'amcl')]


def numbers(speed, amcl, slam, noise, n):
    """Values quoted in the prose, as \\val{key} (report/tables/numbers.tex)."""
    for k in LOCS:
        g = ok(speed, k)
        n[f'{k}-sweep-rmse'] = fmt(med(g, 'pos_rmse', 100))
        n[f'{k}-sweep-yaw'] = fmt(med(g, 'yaw_rmse'))
        n[f'{k}-sweep-lost'] = lost_txt(speed, k)
    n['amcl-outline-rmse'] = fmt(med(ok(amcl, 'amcl'), 'pos_rmse', 100))
    n['amcl-outline-p95'] = fmt(med(ok(amcl, 'amcl'), 'pos_p95', 100))
    n['amcl-outline-yaw'] = fmt(med(ok(amcl, 'amcl'), 'yaw_rmse'))
    n['amcl-outline-lost'] = lost_txt(amcl, 'amcl')
    n['amcl-filled-p95'] = fmt(med(ok(speed, 'amcl'), 'pos_p95', 100))
    n['slam-fixed-rmse'] = fmt(med(ok(slam, 'slam'), 'pos_rmse', 100))
    n['slam-fixed-p95'] = fmt(med(ok(slam, 'slam'), 'pos_p95', 100))
    n['slam-fixed-yaw'] = fmt(med(ok(slam, 'slam'), 'yaw_rmse'))
    n['slam-fixed-lost'] = lost_txt(slam, 'slam')
    rtf = [r['rtf'] for r in ok(slam, 'slam')]
    n['slam-fixed-rtf-min'], n['slam-fixed-rtf-max'] = f'{min(rtf):.2f}', f'{max(rtf):.2f}'
    rtf = [r['rtf'] for r in ok(noise)]
    n['noise-rtf-min'], n['noise-rtf-max'] = f'{min(rtf):.2f}', f'{max(rtf):.2f}'
    for k in LOCS + ['odom']:
        for lv in LEVELS:
            g = ok(noise, k, lv)
            n[f'{k}-n{lv}-rmse'] = fmt(med(g, 'pos_rmse', 100)) if k != 'odom' else fmt(med(g, 'pos_rmse'), 1)
            n[f'{k}-n{lv}-lost'] = lost_txt(noise, k, lv)
            n[f'{k}-n{lv}-yaw'] = fmt(med(g, 'yaw_rmse'))
    n['runs-noise'] = str(len(noise))
    lines = [r'% generated by scripts/report_figures.py: values quoted in the text, used as \val{key}',
             r'\makeatletter']
    lines += [rf'\@namedef{{val@{k}}}{{{v}}}' for k, v in sorted(n.items())]
    lines += [r'\newcommand{\val}[1]{\@ifundefined{val@#1}{\textbf{??}}{\@nameuse{val@#1}}}', r'\makeatother', '']
    (TAB / 'numbers.tex').write_text('\n'.join(lines))
    written.append('tables/numbers.tex')


# ---------------------------------------------------------------- main

def main():
    usetex = style()
    FIG.mkdir(parents=True, exist_ok=True)
    TAB.mkdir(parents=True, exist_ok=True)
    speed = sr.load_runs('speedsweep')
    amcl = sr.load_runs('amcl_outline')
    slam = sr.load_runs('slam_fixed')
    noise = sr.load_runs('odom_noise')
    smoke = {t: sr.load_runs(t) for t in ('slam-lc-smoke', 'slam-lc-nolc-smoke', 'slam-ref-nolc-smoke')}
    n = {}

    cx, cy = sr.centerline(ok(amcl))
    fig_track(cx, cy)
    sweep = [dict(label=NAME[k], color=COLOR[k], runs=ok(speed, k), dodge=0.012) for k in LOCS]
    speed_figure(sweep, 'speed_sweep')
    fig_track_maps(speed, 'speed_sweep_track')
    fig_heatmap(speed, 'speed_sweep_heatmap')
    fig_amcl_maps(cx, cy)
    fig_amcl_lost_run(speed)
    speed_figure([dict(label='Filled map (before)', color=COLOR['amcl'], runs=ok(speed, 'amcl'), dashed=True),
                  dict(label='Outline map (after)', color=COLOR['amcl'], runs=ok(amcl, 'amcl'))],
                 'amcl_before_after', 'AMCL position error', 'AMCL heading error')
    fig_slam_maps(n)
    speed_figure([dict(label='Loop closure on (before)', color=COLOR['slam'], runs=ok(speed, 'slam'), dashed=True),
                  dict(label='Loop closure off (after)', color=COLOR['slam'], runs=ok(slam, 'slam'))],
                 'slam_before_after', 'SLAM Toolbox position error', 'SLAM Toolbox heading error')
    final = [dict(label=NAME['scan'], color=COLOR['scan'], runs=ok(speed, 'scan'), dodge=0.012),
             dict(label=NAME['amcl'], color=COLOR['amcl'], runs=ok(amcl, 'amcl'), dodge=0.012),
             dict(label=NAME['slam'], color=COLOR['slam'], runs=ok(slam, 'slam'), dodge=0.012)]
    speed_figure(final, 'final_comparison', lost=False, log=False)
    fig_fixes(speed, amcl, slam)
    fig_heatmap_fixes(speed, amcl, slam)
    fig_amcl_maps_compact(cx, cy)
    fig_odom_drift(noise)
    fig_noise_rmse(noise)
    fig_noise_along_lap(noise)
    tables(speed, amcl, slam, noise, smoke, n)
    numbers(speed, amcl, slam, noise, n)
    print(f"LaTeX text in figures: {'yes' if usetex else 'no (latex not found, mathtext fallback)'}")
    print('Wrote ' + ', '.join(f'report/{w}' for w in written))


if __name__ == '__main__':
    main()
