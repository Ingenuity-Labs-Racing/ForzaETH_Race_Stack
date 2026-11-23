#!/usr/bin/env python3
"""
Plots the localization benchmark results into results/plots/.

    python3 scripts/plot_results.py [--tag compare]

  rmse_by_run.png       position / heading RMSE of every 1-lap run, per localizer and speed, with how many lost track
  failures_on_track.png where on the track runs went wrong (error > 0.5 m), if any did;
                        runs reset by an outside /initialpose are left out (rmse_by_run.png marks them)
  mean_by_localizer.png mean / median position and heading error of all runs per localizer, over lap progress
                        (tags from --mean-tags), with the corners in CORNERS shaded
  mean_on_track.png     mean estimated path per localizer on the Victoria map, with its 1-sigma spread
  mean_on_track_<corner>.png  the same, zoomed in on each corner in CORNERS

Reads results/summary.csv and the per-sample CSVs in results/<date>_<tag>/.
Works on the host or in the container (needs matplotlib, numpy, pyyaml, pillow).
"""
import argparse
import csv
import statistics
import textwrap
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from PIL import Image  # noqa: E402

SANDBOX = Path(__file__).resolve().parents[1]
RESULTS = SANDBOX / 'results'
MAP_YAML = SANDBOX.parents[1] / 'stack_master' / 'maps' / 'Victoria' / 'Victoria.yaml'
LOST = 0.5  # [m] a run counts as having lost track once its error exceeds this
JUMP = 3.0  # [m] beyond this a run jumped to the wrong place rather than drifted
# Corners where runs struggle, as map boxes [m] (x0, x1, y0, y1): shaded in mean_by_localizer, zoomed in mean_on_track
CORNERS = [('hairpin', (4.3, 6.7, 1.0, 4.2)), ('left-hand corner', (-8.9, -6.3, -1.6, 2.4))]

# Colors follow the localizer, in a fixed order (validated categorical slots 1-3). "odom" (odometry
# only, no localizer) is a reference, not a series: neutral grey, drawn dashed by sweep_report.py.
LOCALIZERS = [
    ('scan', 'Scan matcher (fixed map)', '#2a78d6'),
    ('amcl', 'AMCL (fixed map)', '#eb6834'),
    ('slam', 'SLAM Toolbox (own map)', '#1baf7a'),
    ('odom', 'Odometry only (no localizer)', '#8f8e89'),
]
NAME = {k: n for k, n, _ in LOCALIZERS}
COLOR = {k: c for k, _, c in LOCALIZERS}
SURFACE, TEXT, TEXT2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'axes.edgecolor': GRID, 'axes.labelcolor': TEXT2, 'axes.titlecolor': TEXT,
    'axes.titlesize': 11, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left', 'axes.labelsize': 9,
    'xtick.color': TEXT2, 'ytick.color': TEXT2, 'xtick.labelsize': 9, 'ytick.labelsize': 9,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
    'axes.spines.top': False, 'axes.spines.right': False,
    'legend.frameon': False, 'legend.fontsize': 9, 'font.size': 9, 'text.color': TEXT,
})


def read_summary():
    rows = list(csv.DictReader(open(RESULTS / 'summary.csv')))
    for r in rows:
        log = RESULTS / r.get('run_dir', '') / f"{r['localizer']}_speed_{r['speed']}_run{r['run']}.loc.log"
        r['reset'] = bool(r.get('run_dir')) and log.exists() and 'Re-initialized' in log.read_text()
        for k in ('speed', 'laps', 'pos_rmse', 'pos_p95', 'pos_max', 'yaw_rmse_deg', 'yaw_max_deg'):
            r[k] = float(r[k]) if r[k] else float('nan')
        r['run'] = int(r['run'])
    return rows


def run_csvs(tag):
    """(localizer, speed, results dir, run) -> per-sample columns, from every results dir of the tag."""
    runs = {}
    for d in sorted(RESULTS.glob(f'*_{tag}')):
        for f in d.glob('*speed_*_run*.csv'):
            # runs from before the localizer column (2026-10-07) are all SLAM Toolbox: speed_0.8_run1.csv
            loc, rest = f.stem.split('_speed_') if '_speed_' in f.stem else ('slam', f.stem[len('speed_'):])
            speed, run = rest.split('_run')
            rows = list(csv.DictReader(open(f)))
            if len(rows) < 2:
                continue  # interrupted run
            cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
            cols['t'] -= cols['t'][0]
            # an /initialpose from outside (e.g. Foxglove's pose tool) moved the localizer
            log = f.with_suffix('.loc.log')
            cols['reset'] = log.exists() and 'Re-initialized' in log.read_text()
            runs[(loc, float(speed), d.name, int(run))] = cols
    return runs


def lost_label(ax, x, y, text='lost track', boxed=False):
    bbox = dict(boxstyle='round,pad=0.25', facecolor=SURFACE, edgecolor='none', alpha=0.9) if boxed else None
    ax.annotate(text, (x, y), xytext=(8, 0), textcoords='offset points', va='center', fontsize=8, color=TEXT2,
                bbox=bbox, zorder=5)


def footnote(fig, text, width_in):
    fig.text(0.01, 0.01, textwrap.fill(text, int(width_in * 17)), fontsize=8, color=TEXT2, va='bottom')


def plot_rmse_by_run(rows, out):
    rows = [r for r in rows if r['laps'] == 1 and r['lap_done'] == 'true']
    speeds = sorted({r['speed'] for r in rows})
    present = [k for k, _, _ in LOCALIZERS if any(r['localizer'] == k for r in rows)]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    panels = [('pos_rmse', 'Position error (RMSE per lap)', 'metres, log scale', (0.01, 10)),
              ('yaw_rmse_deg', 'Heading error (RMSE per lap)', 'degrees, log scale', (0.2, 100))]
    width = 0.8 / len(present)
    for ax, (key, title, unit, ylim) in zip(axes, panels):
        for si, speed in enumerate(speeds):
            for li, loc in enumerate(present):
                sel = [r for r in rows if r['localizer'] == loc and r['speed'] == speed]
                vals = [r[key] for r in sel]
                if not vals:
                    continue
                x = si + (li - (len(present) - 1) / 2) * width
                spread = min(0.1, 0.02 * len(vals))
                xs = x + (np.linspace(-spread, spread, len(vals)) if len(vals) > 1 else np.zeros(1))
                normal = [i for i, r in enumerate(sel) if not r['reset']]
                ax.scatter(xs[normal], np.asarray(vals)[normal], s=42, color=COLOR[loc], edgecolor=SURFACE,
                           linewidth=1.5, zorder=3)
                for i, r in enumerate(sel):
                    if r['reset']:  # hollow: not a localization failure
                        ax.scatter(xs[i], vals[i], s=42, facecolor=SURFACE, edgecolor=COLOR[loc], linewidth=1.5,
                                   zorder=3)
                        lost_label(ax, xs[i], vals[i], 'reset by /initialpose')
                lost = sum(r['pos_max'] > LOST and not r['reset'] for r in sel)
                ax.annotate(f'{lost}/{len(sel)}\nlost', (x, 0), xycoords=('data', 'axes fraction'), xytext=(0, 4),
                            textcoords='offset points', ha='center', va='bottom', fontsize=8,
                            color=TEXT if lost else TEXT2, fontweight='bold' if lost else 'normal')
                med = statistics.median(vals)
                ax.hlines(med, x - width * 0.38, x + width * 0.38, color=TEXT, linewidth=1.2, zorder=4)
                fmt = f'{med * 100:.0f} cm' if key == 'pos_rmse' and med < 1 else (
                    f'{med:.2f} m' if key == 'pos_rmse' else f'{med:.1f}°')
                ax.annotate(fmt, (x + width * 0.4, med), xytext=(2, 0), textcoords='offset points',
                            va='center', fontsize=8, color=TEXT2)
        ax.set_yscale('log')
        ax.set_ylim(*ylim)
        ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f'{v:g}'))
        ax.set_xticks(range(len(speeds)), [f'{s:g} m/s' for s in speeds])
        ax.set_xlim(-0.6, len(speeds) - 0.4)
        ax.grid(axis='x', visible=False)
        ax.set_title(title)
        ax.set_ylabel(unit)
    handles = []
    for loc in present:
        sel = [r for r in rows if r['localizer'] == loc and not r['reset']]
        lost = sum(r['pos_max'] > LOST for r in sel)
        handles.append(matplotlib.lines.Line2D([], [], marker='o', linestyle='none', ms=7, color=COLOR[loc],
                                               mec=SURFACE, label=f'{NAME[loc]}: {lost} of {len(sel)} lost track'))
    fig.legend(handles=handles, loc='upper center', ncol=len(present), bbox_to_anchor=(0.5, 0.99))
    footnote(fig, 'Each dot is one 1-lap run with perfect odometry; the bar and label mark the median. Under each '
             f'group: runs that lost track (error > {LOST:g} m). Hollow = run reset from outside, not counted. '
             'SLAM Toolbox uses its own recorded map, the others the ground-truth map.', 12)
    fig.tight_layout(rect=(0, 0.06, 1, 0.92))
    fig.savefig(out, dpi=160)
    plt.close(fig)


def lap_curves(runs, keys, col, grid):
    """Column `col` of each run, resampled at `grid` [% of the lap, by distance driven] -> (runs, len(grid))."""
    curves = []
    for k in keys:
        d = runs[k]
        s = np.concatenate(([0], np.cumsum(np.hypot(np.diff(d['gt_x']), np.diff(d['gt_y'])))))
        curves.append(np.interp(grid, 100 * s / s[-1], np.abs(d[col]) if col.endswith('_err') else d[col]))
    return np.array(curves)


def smooth(a, n=5):
    """Rolling mean over the last axis, n samples wide."""
    pad = np.pad(a, [(0, 0)] * (a.ndim - 1) + [(n // 2, n // 2)], mode='edge')
    return np.apply_along_axis(lambda v: np.convolve(v, np.ones(n) / n, 'valid'), -1, pad)


def corner_spans(runs, grid):
    """(name, first, last) lap progress [%] at which the mean true path is inside each box of CORNERS."""
    x = lap_curves(runs, list(runs), 'gt_x', grid).mean(axis=0)
    y = lap_curves(runs, list(runs), 'gt_y', grid).mean(axis=0)
    spans = []
    for name, (x0, x1, y0, y1) in CORNERS:
        inside = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        if inside.any():
            spans.append((name, grid[inside].min(), grid[inside].max()))
    return spans


def load_map():
    """Victoria ground-truth map image and its imshow extent [m]."""
    info = yaml.safe_load(open(MAP_YAML))
    img = np.array(Image.open(MAP_YAML.parent / info['image']).convert('L'))
    h, w = img.shape
    res, (ox, oy) = info['resolution'], info['origin'][:2]
    return img, (ox, ox + w * res, oy, oy + h * res)


def track_view(img, extent, margin=0.3):
    """The part of `extent` that holds the free track, plus `margin` [m] (image row 0 is the top)."""
    rows, cols = np.nonzero(img > 200)
    h, w = img.shape
    sx, sy = (extent[1] - extent[0]) / w, (extent[3] - extent[2]) / h
    return (extent[0] + cols.min() * sx - margin, extent[0] + (cols.max() + 1) * sx + margin,
            extent[3] - (rows.max() + 1) * sy - margin, extent[3] - rows.min() * sy + margin)


def view_with(view, extent, ds, margin=0.2):
    """`view` grown to also hold the estimates of the runs `ds` (they can leave the track), within `extent`."""
    x = np.concatenate([d['slam_x'] for d in ds] + [view[:2]])
    y = np.concatenate([d['slam_y'] for d in ds] + [view[2:]])
    return (max(extent[0], x.min() - margin), min(extent[1], x.max() + margin),
            max(extent[2], y.min() - margin), min(extent[3], y.max() + margin))


def map_ax(ax, img, extent, view, alpha, ticks=False):
    ax.imshow(img, cmap='gray', extent=extent, vmin=0, vmax=255, alpha=alpha, zorder=0)
    ax.set_aspect('equal')
    ax.set_xlim(view[0], view[1])
    ax.set_ylim(view[2], view[3])
    ax.grid(False)
    if ticks:
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
    else:
        ax.set_xticks([])
        ax.set_yticks([])


def scale_bar(ax, view, length, label):
    x0, y0 = view[0] + 0.04 * (view[1] - view[0]), view[2] + 0.04 * (view[3] - view[2])
    ax.plot([x0, x0 + length], [y0, y0], color=TEXT, linewidth=2, solid_capstyle='butt', zorder=5)
    ax.annotate(label, (x0, y0), xytext=(0, 4), textcoords='offset points', fontsize=8, color=TEXT)


def plot_mean_by_localizer(runs, tags, out):
    """Mean error of all runs per localizer, against lap progress so runs of different lap times line up."""
    runs = {k: d for k, d in runs.items() if not d['reset']}  # an outside reset is not the localizer's error
    speeds = sorted({k[1] for k in runs})
    grid = np.linspace(0, 100, 401)
    spans = corner_spans(runs, grid)
    fig, axes = plt.subplots(len(speeds), 2, figsize=(14, 3.8 * len(speeds)), sharex=True, squeeze=False)
    panels = [('pos_err', 1, 'Position error', 'metres, log scale', (2e-3, 3)),
              ('yaw_err', np.degrees(1), 'Heading error', 'degrees, log scale', (1e-2, 30))]
    for row, speed in zip(axes, speeds):
        handles, labels = [], []
        for loc, name, color in LOCALIZERS:
            keys = [k for k in runs if k[0] == loc and k[1] == speed]
            if not keys:
                continue
            lap_means = []
            for ax, (key, scale, _, _, _) in zip(row, panels):
                curves = smooth(lap_curves(runs, keys, key, grid) * scale)
                mean = curves.mean(axis=0)
                lap_means.append(mean.mean())
                if len(keys) > 1:  # middle half of the runs, and its median
                    lo, med, hi = np.percentile(curves, [25, 50, 75], axis=0)
                    ax.fill_between(grid, lo, hi, color=color, alpha=0.15, linewidth=0)
                    ax.plot(grid, med, color=color, linewidth=1, linestyle=(0, (3, 2)))
                line, = ax.plot(grid, mean, color=color, linewidth=1.6)
            pos, yaw = lap_means
            lost = sum(runs[k]['pos_err'].max() > LOST for k in keys)
            n = f'{len(keys)} run' + ('s' if len(keys) > 1 else '') + (f', {lost} lost track' if lost else '')
            handles.append(line)
            labels.append(f'{name}\n{n}\nlap mean {pos * 100:.0f} cm, {yaw:.1f}°')
        handles += [matplotlib.lines.Line2D([], [], color=TEXT2, linewidth=1.6),
                    matplotlib.lines.Line2D([], [], color=TEXT2, linewidth=1, linestyle=(0, (3, 2))),
                    matplotlib.patches.Patch(color=TEXT2, alpha=0.15, linewidth=0)]
        labels += ['mean', 'median', 'middle half of runs']
        for ax, (key, _, title, unit, ylim) in zip(row, panels):
            for name, p0, p1 in spans:
                ax.axvspan(p0, p1, color=GRID, alpha=0.7, linewidth=0, zorder=0)
                ax.annotate(name, ((p0 + p1) / 2, 0), xycoords=('data', 'axes fraction'), xytext=(0, 3),
                            textcoords='offset points', ha='center', va='bottom', fontsize=8, color=TEXT2,
                            bbox=dict(boxstyle='round,pad=0.2', facecolor=SURFACE, edgecolor='none', alpha=0.85))
            if key == 'pos_err':
                ax.axhline(LOST, color=TEXT2, linewidth=0.8, linestyle=(0, (4, 3)))
                ax.annotate(f'{LOST:g} m', (1, LOST), xycoords=('axes fraction', 'data'), xytext=(-2, 3),
                            textcoords='offset points', ha='right', fontsize=8, color=TEXT2)
            ax.set_yscale('log')
            ax.set_ylim(*ylim)
            ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f'{v:g}'))
            ax.set_xlim(0, 100)
            ax.set_title(f'{title} at {speed:g} m/s')
            ax.set_ylabel(unit)
        row[1].legend(handles, labels, loc='upper left', bbox_to_anchor=(1.02, 1), labelspacing=1.1)
    for ax in axes[-1]:
        ax.set_xlabel('lap progress [%]')
    footnote(fig, 'Each run is resampled by distance driven and smoothed over 1.25 % of the lap. A run that loses '
             'track pulls the mean far above its band; the median follows the typical run. Shaded = the corners '
             f'zoomed in mean_on_track_*.png. Tags: {", ".join(tags)}. Runs reset by an outside /initialpose '
             'are left out.', 14)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out, dpi=160)
    plt.close(fig)


def sigma_cloud(x, y, cov, extent, res=0.02):
    """Mask on a `res` grid over `extent`: the union of the 1-sigma ellipses of the points (x, y) with covariances cov."""
    nx, ny = int((extent[1] - extent[0]) / res), int((extent[3] - extent[2]) / res)
    mask = np.zeros((ny, nx), bool)
    for px, py, c in zip(x, y, cov):
        c = c + np.eye(2) * res ** 2  # at least a pixel wide: all runs start at the true pose, and 2 runs make a line
        r = np.sqrt(np.linalg.eigvalsh(c).max())
        i0, i1 = [int(np.clip((v - extent[0]) / res, 0, nx)) for v in (px - r, px + r + res)]
        j0, j1 = [int(np.clip((v - extent[2]) / res, 0, ny)) for v in (py - r, py + r + res)]
        gx, gy = np.meshgrid(extent[0] + (np.arange(i0, i1) + 0.5) * res - px,
                             extent[2] + (np.arange(j0, j1) + 0.5) * res - py)
        inv = np.linalg.inv(c)
        mask[j0:j1, i0:i1] |= inv[0, 0] * gx ** 2 + 2 * inv[0, 1] * gx * gy + inv[1, 1] * gy ** 2 <= 1
    return mask


def plot_mean_on_track(runs, tags, out, corner=None):
    """Mean estimated path of all runs per localizer and speed, with the 1-sigma spread around it, on the map.
    With `corner` (an entry of CORNERS) zoomed in on that corner, otherwise the whole track."""
    runs = {k: d for k, d in runs.items() if not d['reset']}  # an outside reset is not the localizer's error
    speeds = sorted({k[1] for k in runs})
    present = [(k, n, c) for k, n, c in LOCALIZERS if any(r[0] == k for r in runs)]
    grid = np.linspace(0, 100, 1001)
    img, extent = load_map()
    view = corner[1] if corner else view_with(track_view(img, extent), extent,
                                              [d for d in runs.values() if d['pos_err'].max() > JUMP])
    res = 0.004 if corner else 0.02
    # panels placed by hand [in]: map panels of a fixed aspect leave gaps under tight_layout
    panel_w = 3.6 if corner else 5.4
    panel_h = panel_w * (view[3] - view[2]) / (view[1] - view[0])
    title_h, gap, margin = 0.6, 0.25, 0.15
    legend_rows = 2 if corner else 1
    foot_h = 0.3 * legend_rows + 0.6  # legend and footnote under the panels
    head_h = 0.45 if corner else 0.05
    fig_w = len(present) * (panel_w + gap) - gap + 2 * margin
    fig_h = head_h + len(speeds) * (title_h + panel_h) + (len(speeds) - 1) * gap + foot_h
    fig = plt.figure(figsize=(fig_w, fig_h))
    axes = np.array([[fig.add_axes(((margin + c * (panel_w + gap)) / fig_w,
                                    (fig_h - head_h - (r + 1) * (title_h + panel_h) - r * gap) / fig_h,
                                    panel_w / fig_w, panel_h / fig_h))
                      for c in range(len(present))] for r in range(len(speeds))])
    for row, speed in zip(axes, speeds):
        for ax, (loc, name, color) in zip(row, present):
            map_ax(ax, img, extent, view, alpha=0.15)
            keys = [k for k in runs if k[0] == loc and k[1] == speed]
            # a run that jumped to the wrong place would spread the cloud over the whole map: drawn on its own
            jumped = [k for k in keys if runs[k]['pos_err'].max() > JUMP]
            kept = [k for k in keys if k not in jumped]
            if not keys:
                ax.set_title(f'{name}, {speed:g} m/s', fontsize=10 if corner else 11)
                continue
            sub = f'{len(keys)} run' + ('s' if len(keys) > 1 else '') + (f', {len(jumped)} jumped' if jumped else '')
            gx = lap_curves(runs, keys, 'gt_x', grid).mean(axis=0)
            gy = lap_curves(runs, keys, 'gt_y', grid).mean(axis=0)
            for k in jumped:
                ax.plot(runs[k]['slam_x'], runs[k]['slam_y'], color=color, linewidth=0.8, alpha=0.6, zorder=2)
            if kept:
                ex, ey = lap_curves(runs, kept, 'slam_x', grid), lap_curves(runs, kept, 'slam_y', grid)
                if len(kept) > 1:
                    dx, dy = ex - ex.mean(axis=0), ey - ey.mean(axis=0)
                    cov = np.stack([np.stack([(dx * dx).mean(0), (dx * dy).mean(0)], -1),
                                    np.stack([(dx * dy).mean(0), (dy * dy).mean(0)], -1)], -2)
                    spread = np.sqrt(cov[..., 0, 0] + cov[..., 1, 1])
                    if corner:  # only the part of the lap in view
                        mx, my = ex.mean(axis=0), ey.mean(axis=0)
                        spread = spread[(mx >= view[0]) & (mx <= view[1]) & (my >= view[2]) & (my <= view[3])]
                    sub += (f'; 1σ spread{" here" if corner else ""} median {np.median(spread) * 100:.0f} cm, '
                            f'max {spread.max() * 100:.0f} cm')
                    mask = sigma_cloud(ex.mean(axis=0), ey.mean(axis=0), cov, view, res)
                    ax.imshow(np.ma.masked_where(~mask, mask), cmap=matplotlib.colors.ListedColormap([TEXT2]),
                              extent=view, origin='lower', alpha=0.35, zorder=1, interpolation='nearest')
                ax.plot(ex.mean(axis=0), ey.mean(axis=0), color=color, linewidth=1.6, zorder=3)
            ax.plot(gx, gy, color=TEXT, linewidth=0.8, zorder=2)
            sub = textwrap.fill(sub, int(panel_w * 17))
            ax.annotate(sub, (0, 1), xycoords='axes fraction', xytext=(0, 3), textcoords='offset points',
                        va='bottom', fontsize=8, color=TEXT2)
            ax.set_title(f'{name}, {speed:g} m/s', pad=6 + 11 * (sub.count('\n') + 1), fontsize=10 if corner else 11)
            if corner:
                scale_bar(ax, view, 0.25, '25 cm')
            else:
                ax.plot(gx[0], gy[0], '>', ms=9, color=TEXT2, zorder=4)
                scale_bar(ax, view, 1, '1 m')
                for cname, (x0, x1, y0, y1) in CORNERS:  # the zoomed views
                    ax.add_patch(matplotlib.patches.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                                                              edgecolor=TEXT2, linewidth=0.7,
                                                              linestyle=(0, (3, 2)), zorder=4))
                    ax.annotate(cname, (x0, y1), xytext=(2, -2), textcoords='offset points', va='top',
                                fontsize=7, color=TEXT2, zorder=5)
    handles = [matplotlib.lines.Line2D([], [], color=TEXT, linewidth=0.8, label='true path')]
    handles += [matplotlib.lines.Line2D([], [], color=c, linewidth=1.6, label=f'{n}: mean estimate')
                for _, n, c in present]
    handles.append(matplotlib.patches.Patch(color=TEXT2, alpha=0.35, linewidth=0, label='1 standard deviation'))
    note = ('Mean and spread of the estimated position over the runs, matched by lap progress. Runs that jumped '
            f'to the wrong place (error > {JUMP:g} m) are left out of the mean and drawn as thin lines. ')
    if corner:
        fig.suptitle(f'Close-up: {corner[0]}', x=margin / fig_w, y=1 - 0.1 / fig_h, ha='left', va='top', fontsize=12, fontweight='bold')
    else:
        note += 'Dashed boxes = the close-ups in mean_on_track_*.png. '
    footnote(fig, note + f'Tags: {", ".join(tags)}. Runs reset by an outside /initialpose are left out.', fig_w)
    fig.legend(handles=handles, loc='lower center', ncol=-(-len(handles) // legend_rows),
               bbox_to_anchor=(0.5, 0.45 / fig_h))
    fig.savefig(out, dpi=160)
    plt.close(fig)


def plot_failures(runs, out):
    # an outside /initialpose is not a localization failure (rmse_by_run.png marks those runs)
    failed = [k for k, d in sorted(runs.items()) if d['pos_err'].max() > LOST and not d['reset']]
    if not failed:
        return False
    cols = min(3, len(failed))
    nrows = -(-len(failed) // cols)
    img, extent = load_map()
    view = view_with(track_view(img, extent), extent, [runs[k] for k in failed])
    fig, axes = plt.subplots(nrows, cols, figsize=(6.2 * cols, 4.3 * nrows + 0.6), squeeze=False)
    for ax in axes.flat[len(failed):]:
        ax.set_visible(False)
    for ax, k in zip(axes.flat, failed):
        d = runs[k]
        map_ax(ax, img, extent, view, alpha=0.35, ticks=True)
        ax.plot(d['gt_x'], d['gt_y'], color=TEXT2, linewidth=1.5)
        ax.plot(d['slam_x'], d['slam_y'], color=COLOR[k[0]], linewidth=1.5)
        j = int(np.argmax(d['pos_err'] > LOST))
        ax.plot(d['gt_x'][j], d['gt_y'][j], 'o', ms=8, color=TEXT, mec=SURFACE, mew=1.5, zorder=4)
        lost_label(ax, d['gt_x'][j], d['gt_y'][j], f'error > {LOST:g} m at {d["t"][j]:.0f} s', boxed=True)
        ax.plot(d['gt_x'][0], d['gt_y'][0], '>', ms=8, color=TEXT2, zorder=4)
        ax.set_title(f'{NAME[k[0]]}, {k[1]:g} m/s', pad=14)
        ax.annotate(f'{k[2]}, run {k[3]}', (0, 1), xycoords='axes fraction', xytext=(0, 3),
                    textcoords='offset points', va='bottom', fontsize=8, color=TEXT2)
    shown = [loc for loc, _, _ in LOCALIZERS if any(k[0] == loc for k in failed)]
    handles = [matplotlib.lines.Line2D([], [], color=TEXT2, linewidth=1.5, label='true path')]
    handles += [matplotlib.lines.Line2D([], [], color=COLOR[loc], linewidth=1.5, label=f'{NAME[loc]} estimate')
                for loc in shown]
    handles.append(matplotlib.lines.Line2D([], [], marker='o', linestyle='none', ms=8, color=TEXT, mec=SURFACE,
                                           label=f'first error > {LOST:g} m'))
    fig.legend(handles=handles, loc='lower center', ncol=len(handles), bbox_to_anchor=(0.5, 0))
    fig.tight_layout(rect=(0, 0.6 / (4.3 * nrows + 0.6), 1, 1))
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--tag', default='compare', help='benchmark tag for the per-run plots')
    parser.add_argument('--mean-tags', nargs='+', default=['compare', 'baseline'],
                        help='1-lap benchmark tags averaged in mean_*.png (baseline adds SLAM Toolbox runs)')
    args = parser.parse_args()

    out = RESULTS / 'plots'
    out.mkdir(exist_ok=True)
    plot_rmse_by_run(read_summary(), out / 'rmse_by_run.png')
    written = ['rmse_by_run.png']
    runs = run_csvs(args.tag)
    if runs:
        if plot_failures(runs, out / 'failures_on_track.png'):
            written.append('failures_on_track.png')
    mean_runs = {}
    for tag in args.mean_tags:
        mean_runs.update(run_csvs(tag))
    if mean_runs:
        plot_mean_by_localizer(mean_runs, args.mean_tags, out / 'mean_by_localizer.png')
        plot_mean_on_track(mean_runs, args.mean_tags, out / 'mean_on_track.png')
        written += ['mean_by_localizer.png', 'mean_on_track.png']
        for corner in CORNERS:
            name = f"mean_on_track_{corner[0].replace(' ', '_').replace('-', '_')}.png"
            plot_mean_on_track(mean_runs, args.mean_tags, out / name, corner)
            written.append(name)
    print('Wrote ' + ', '.join(str(out / f) for f in written))


if __name__ == '__main__':
    main()
