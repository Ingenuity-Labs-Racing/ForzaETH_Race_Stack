#!/usr/bin/env python3
"""
Summary table and figure of a sweep (scripts/run_speed_sweep.sh) over speeds or odometry noise
levels, or of any benchmark tag.

    python3 scripts/sweep_report.py [--tag speedsweep]
    python3 scripts/sweep_report.py --tag amcl_outline:amcl --tag speedsweep:scan,slam   # combine sweeps

<tag> below is the tag, or with several --tag the --name (default: the tags joined by "_vs_").

  results/<tag>_table.md, results/<tag>_table.csv   one row per speed (or noise level), the main metrics per
                                                     localizer
  results/<tag>_runs.csv                             one row per run, every metric (also per lap)
  results/plots/<tag>.png                            position / heading error and lap-to-lap change over speed
                                                     (or noise level)
  results/plots/<tag>_track.png                      track map per localizer: median error along the lap, where
                                                     runs lost track
  results/plots/<tag>_offsets.png                    signed error in the car's frame (along, sideways, heading):
                                                     constant part (mean) and random part (SD) per run
  results/plots/<tag>_heatmap.png                    error along the laps, one row per run

When the runs have several odometry noise levels (at one speed), the noise level replaces speed as
the axis of the table and figures. "odom" (odometry only, no localizer) is drawn as a dashed grey
reference: the drift the localizers have to remove.
All metrics cover only the driving window (the car's first to last movement): every run also logs
~10 s with the car parked, a share that changes with speed and would bias a speed comparison.
Reads the per-run files in results/<date>_<tag>/, so it also works on a sweep that is still running.
"""
import argparse
import csv
import math
import re
import statistics
import time
import warnings
from pathlib import Path

import numpy as np

import plot_results as pr  # shared colors, names, chart style (sets the matplotlib backend and rcParams)
import matplotlib.pyplot as plt  # noqa: E402  (after plot_results picked the backend)
from matplotlib.cm import ScalarMappable  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, LogNorm  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402

MOVE = 0.05          # [m] the car counts as driving once it is this far from its start / final position
GT_HZ = 50           # /ground_truth rate. pose_error skips samples it can't match in time (normally 25-50%)
MAX_GAP = 1.0        # [s] flag runs with a longer stretch without an estimate (normal runs: 0.1-0.4 s)
# flag runs where the sim ran below this share of real time: the localizer then gets more CPU per sim
# second than it would on the car (the 6-worker SLAM Toolbox sweep ran at 0.53-0.59x)
MIN_RTF = 0.9
SHORT = {'scan': 'Scan', 'amcl': 'AMCL', 'slam': 'SLAM TB', 'odom': 'Odom only'}
REFERENCE = {'odom'}  # not a localizer: dashed grey line, left out of the lap-change panel, track map and offsets
RUN_FILE = re.compile(r'^(\w+)_speed_([\d.]+)(?:_noise([\d.]+))?_run(\d+)\.sim\.log$')
# per axis: table header, axis label, label of the all-runs row, the word for one step
AXIS = {'speed': ('Speed [m/s]', 'commanded speed [m/s]', 'All speeds', 'speed'),
        'noise': ('Noise level', 'odometry noise level (0 = perfect, 1 ≈ a real car, 2 = every error doubled, ...)',
                  'All levels', 'noise level')}
BINS_PER_LAP = 100   # track map and heatmap: ~0.37 m of the 36.8 m lap per bin
# position error colors (track map, heatmap): one-hue sequential ramp, log scale shared by both figures
ERR_CMAP = LinearSegmentedColormap.from_list(
    'error', ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b'])
ERR_NORM = LogNorm(0.02, 5)
ERR_TICKS = [0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5]


def read_text(path):
    try:
        return path.read_text(errors='replace')
    except OSError:
        return ''


def rms(a):
    return float(np.sqrt(np.mean(np.square(a)))) if len(a) else math.nan


def summary_rows(tag):
    """(run dir, localizer, speed, noise, run) -> summary.csv row, for the laps setting and the commit."""
    try:
        rows = list(csv.DictReader(open(pr.RESULTS / 'summary.csv')))
    except OSError:
        return {}
    # rows from before the odom_noise column have no value there: perfect odometry, level 0
    return {(r.get('run_dir', ''), r['localizer'], r['speed'], r.get('odom_noise') or '0', int(r['run'])): r
            for r in rows if r['tag'] == tag}


def run_metrics(loc, speed, noise, run, base, run_dir, summary):
    """Every metric of one run. base = results dir / '<loc>_speed_<speed>[_noise<noise>]_run<run>'."""
    file = lambda ext: Path(f'{base}.{ext}')  # noqa: E731
    row = summary.get((run_dir, loc, speed, noise, run))
    drive_log = read_text(file('autodrive.log'))
    if row is None:
        # eval_localization.sh adds the summary row when a run ends (also a failed one). Without it the
        # run is either still going (its files keep changing) or from before run_dir was recorded.
        newest = max((p.stat().st_mtime for p in base.parent.glob(f'{base.name}.*')), default=0)
        if time.time() - newest < 600:
            return {'localizer': loc, 'speed': speed, 'noise': noise, 'run': run, 'run_dir': run_dir,
                    'status': 'running'}
        done = re.search(r'Done: ([\d.]+) laps', drive_log)
        row = {'laps': done.group(1) if done else '1'}
    wall = re.search(r'wall_s=(\d+)', drive_log)
    r = {
        'localizer': loc, 'speed': speed, 'noise': noise, 'run': run, 'run_dir': run_dir,
        'commit': row.get('commit', ''),
        'map': row.get('map', ''), 'params': row.get('params', ''),
        'laps': float(row['laps']) if row.get('laps') else 1.0,
        'lap_done': 'Done:' in drive_log,
        'stuck': drive_log.count('Stuck'),
        'retries': len(list(base.parent.glob(f'{base.name}.sim.log.failed*'))),
        'reset': 'Re-initialized' in read_text(file('loc.log')),
        'wall_s': float(wall.group(1)) if wall else math.nan,
        'status': 'failed',
    }
    try:
        rows = list(csv.DictReader(open(file('csv'))))
    except OSError:
        rows = []
    if len(rows) < 2:
        return r  # never started, or no localizer estimate at all
    c = {k: np.array([float(v[k]) for v in rows])
         for k in ('t', 'gt_x', 'gt_y', 'gt_yaw', 'slam_x', 'slam_y', 'slam_yaw', 'pos_err', 'yaw_err')}

    # driving window: from the first sample away from the start to the last sample away from the end
    x, y = c['gt_x'], c['gt_y']
    away_start = np.hypot(x - x[0], y - y[0]) > MOVE
    away_end = np.hypot(x - x[-1], y - y[-1]) > MOVE
    if not away_start.any() or not away_end.any():
        return r  # the car never drove
    w = slice(int(np.argmax(away_start)), len(x) - int(np.argmax(away_end[::-1])))
    t, x, y = c['t'][w], x[w], y[w]
    pos, yaw = c['pos_err'][w], np.degrees(c['yaw_err'][w])
    duration = t[-1] - t[0]
    dist = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])

    # split into laps by distance driven (autodrive stops after exactly `laps` laps)
    n_laps = int(round(r['laps'])) if r['laps'] >= 1 else 1
    lap = np.minimum((dist / max(dist[-1], 1e-9) * n_laps).astype(int), n_laps - 1)
    r.update({
        'status': 'ok', 'n_laps': n_laps, 'samples': len(pos), 'drive_s': duration,
        'speed_actual': dist[-1] / duration if duration > 0 else math.nan,
        'coverage': min(len(pos) / (duration * GT_HZ + 1), 1.0) if duration > 0 else math.nan,
        'max_gap_s': float(np.diff(t).max()) if len(t) > 1 else math.nan,
        'rtf': duration / r['wall_s'] if r['wall_s'] > 0 else math.nan,
        'pos_rmse': rms(pos), 'pos_p95': float(np.percentile(pos, 95)), 'pos_max': float(pos.max()),
        'yaw_rmse': rms(yaw), 'yaw_max': float(yaw.max()),
    })
    for k in range(n_laps):
        r[f'lap{k + 1}_pos_rmse'] = rms(pos[lap == k])
        r[f'lap{k + 1}_yaw_rmse'] = rms(yaw[lap == k])
    r['lap_change'] = r[f'lap{n_laps}_pos_rmse'] - r['lap1_pos_rmse'] if n_laps > 1 else math.nan
    r['lost'] = r['pos_max'] > pr.LOST
    r['lost_lap'] = int(lap[np.argmax(pos > pr.LOST)]) + 1 if r['lost'] else 0

    # signed error in the car's frame (pose_error.py only stores its size): constant part = mean, random part = SD,
    # over the samples while the localizer was on track (error below LOST)
    ex, ey, gyaw = c['slam_x'][w] - x, c['slam_y'][w] - y, c['gt_yaw'][w]
    signed = {
        'along': ex * np.cos(gyaw) + ey * np.sin(gyaw),     # + = estimate ahead of the car
        'side': -ex * np.sin(gyaw) + ey * np.cos(gyaw),     # + = estimate left of the car
        'head': np.degrees(np.arctan2(np.sin(c['slam_yaw'][w] - gyaw), np.cos(c['slam_yaw'][w] - gyaw))),  # + = ccw
    }
    tracked = pos < pr.LOST
    r['tracked_share'] = float(tracked.mean())
    for k, v in signed.items():
        enough = tracked.sum() >= 10
        r[f'{k}_mean'] = float(v[tracked].mean()) if enough else math.nan
        r[f'{k}_sd'] = float(v[tracked].std()) if enough else math.nan
    # per-sample data for the track map and heatmap (not written to any CSV)
    r['_trace'] = {'laps': np.minimum(dist / max(dist[-1], 1e-9) * n_laps, n_laps - 1e-9), 'x': x, 'y': y, 'pos': pos}

    flags = []
    if r['max_gap_s'] > MAX_GAP:
        flags.append(f"no estimate for {r['max_gap_s']:.1f} s at one point")
    if r['stuck']:
        flags.append(f"autodrive got stuck {r['stuck']}x")
    if r['rtf'] < MIN_RTF:
        flags.append(f"sim at {r['rtf']:.2f}x real time")
    if not r['lap_done']:
        flags.append('laps not finished')
    if r['retries']:
        flags.append(f"started on attempt {r['retries'] + 1}")
    if r['reset']:
        flags.append('reset by an outside /initialpose')
    r['flags'] = flags
    return r


def load_runs(tag, only=None):
    """Every run of a tag; only = set of localizers to keep (None: all)."""
    summary = summary_rows(tag)
    runs = []
    for d in sorted(p for p in pr.RESULTS.glob(f'*_{tag}') if p.is_dir()):
        for sim_log in sorted(d.glob('*_speed_*_run*.sim.log')):
            m = RUN_FILE.match(sim_log.name)
            if m and (only is None or m.group(1) in only):
                base = sim_log.with_name(sim_log.name[:-len('.sim.log')])
                runs.append(run_metrics(m.group(1), m.group(2), m.group(3) or '0', int(m.group(4)), base, d.name,
                                        summary))
    return runs


def num(v, digits=1, sign=False):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return '—'
    if not sign and abs(v) >= 10:
        digits = 0
    return f'{v:+.{digits}f}' if sign else f'{v:.{digits}f}'


def median(values):
    values = [v for v in values if not math.isnan(v)]
    return statistics.median(values) if values else math.nan


MAP_DESC = {
    'victoria_ref': 'its own recorded map (`victoria_ref`)',
    'victoria_ref_lc': 'its own recorded map, made with loop closure (`victoria_ref_lc`)',
    'Victoria': 'the filled-in map the world was generated from (`Victoria.yaml`)',
    'victoria_outline': 'the walls of the map the world was generated from, off-track area unknown '
                        '(`victoria_outline`)',
}


PARAMS_DESC = {'localization.yaml': 'loop closure on', 'localization_no_loop_closure.yaml': 'loop closure off'}
DEFAULT_PARAMS = {'amcl': 'amcl.yaml', 'scan': 'scan_match.yaml'}  # not worth a mention
# summary rows from before 2026-11-22 say "default" for the launch file's default parameters at the time
OLD_DEFAULT_PARAMS = {'slam': 'localization.yaml', 'amcl': 'amcl.yaml', 'scan': 'scan_match.yaml'}


def maps_note(runs, present):
    """Which map (and parameter file, where it matters) each localizer used in these runs (summary.csv)."""
    parts = []
    for loc in present:
        mine = [r for r in runs if r['localizer'] == loc]
        used = sorted({r['map'] for r in mine if r.get('map') not in (None, '', 'none')})
        params = sorted({OLD_DEFAULT_PARAMS.get(loc, '') if r.get('params') == 'default' else r.get('params', '')
                         for r in mine} - {'', 'none', DEFAULT_PARAMS.get(loc)})
        if used:
            parts.append(f"{pr.NAME[loc].split(' (')[0]} on " + ' and '.join(MAP_DESC.get(m, f'`{m}`') for m in used)
                         + (' with ' + ', '.join(f'{PARAMS_DESC[p]} (`{p}`)' if p in PARAMS_DESC else f'`{p}`'
                                                 for p in params) if params else ''))
    return '; '.join(parts) or 'unknown'


def noisy(runs):
    return any(float(r['noise']) > 0 for r in runs)


def odometry_note(runs):
    return ('Odometry corrupted by odom_noise at the level shown, correct start.' if noisy(runs)
            else 'Perfect odometry and a correct start.')


def x_positions(values, axis):
    """Axis position of each speed / noise level: speeds as numbers, noise levels evenly spaced (they double)."""
    vals = sorted(set(values), key=float)
    return {v: float(v) if axis == 'speed' else float(i) for i, v in enumerate(vals)}


def x_label(v, axis):
    return f'{float(v):.2f}' if axis == 'speed' else v


def where(r, multi):
    """Where a run sits in the sweep, for the notes: speed, noise level, run number."""
    return (f"at {r['speed']} m/s" + (f", noise {r['noise']}" if float(r['noise']) > 0 else '')
            + (f", run {r['run']}" if multi else ''))


def build_table(runs, present, n_laps, axis):
    header = [AXIS[axis][0], 'Actual [m/s]']
    for loc in present:
        s = SHORT.get(loc, loc)
        header += [f'{s} lost', f'{s} RMSE [cm]', f'{s} p95 [cm]', f'{s} heading [°]']
        if n_laps > 1:
            header.append(f'{s} lap 1→{n_laps} [cm]')

    def cells(sel, all_speeds=False):
        ok = [r for r in sel if r['status'] == 'ok']
        failed = len(sel) - len(ok)
        if not sel:
            return [''] * (5 if n_laps > 1 else 4)
        if not ok:
            return ['failed'] + ['—'] * (4 if n_laps > 1 else 3)
        lost = [r for r in ok if r['lost']]
        if all_speeds or len(ok) > 1:
            lost_txt = f'{len(lost)}/{len(ok)}' + (f' +{failed} failed' if failed else '')
        else:
            lost_txt = (f"lap {lost[0]['lost_lap']}" if n_laps > 1 else 'yes') if lost else '—'
        flagged = ' †' if not all_speeds and any(r['flags'] for r in ok) else ''
        out = [lost_txt,
               num(median([r['pos_rmse'] * 100 for r in ok])) + flagged,
               num(median([r['pos_p95'] * 100 for r in ok])),
               num(median([r['yaw_rmse'] for r in ok]))]
        if n_laps > 1:
            out.append(num(median([r['lap_change'] * 100 for r in ok]), sign=True))
        return out

    rows = []
    for value in sorted({r[axis] for r in runs}, key=float):
        at = [r for r in runs if r[axis] == value]
        row = [value, num(median([r['speed_actual'] for r in at if r['status'] == 'ok']), 2)]
        for loc in present:
            row += cells([r for r in at if r['localizer'] == loc])
        rows.append(row)
    total = [AXIS[axis][2], '']
    for loc in present:
        total += cells([r for r in runs if r['localizer'] == loc], all_speeds=True)
    rows.append(total)
    return header, rows


def write_markdown(path, tag, header, rows, runs, present, n_laps, running, axis):
    commits = sorted({r['commit'] for r in runs if r['commit']})
    ok = [r for r in runs if r['status'] == 'ok']
    word, total_label = AXIS[axis][3], AXIS[axis][2]
    per_point = len(ok) / max(len({(r['localizer'], r[axis]) for r in ok}), 1)
    multi = any(r['run'] > 1 for r in runs)
    title = 'odometry noise sweep' if axis == 'noise' else 'speed sweep'
    lines = [
        f'# Localization {title} (`{tag}`)', '',
        f"{len(runs)} runs: {', '.join(pr.NAME[k] for k in present)}; "
        f"{'one' if per_point <= 1 else f'{per_point:.0f}'} {n_laps}-lap run{'s' if per_point > 1 else ''} "
        f"per {word}"
        + (f" at {', '.join(sorted({r['speed'] for r in runs}, key=float))} m/s" if axis == 'noise' else '')
        + f". Code: {', '.join(commits) or 'unknown'}. Figures: `results/plots/{tag}.png`, "
        f"`{tag}_track.png` (where on the track), `{tag}_offsets.png` (constant vs random error), "
        f"`{tag}_heatmap.png` (error along the laps)."
        + (f' **{running} more runs still in progress**: this table is partial.' if running else ''), '',
        '| ' + ' | '.join(header) + ' |',
        '|' + '|'.join('---' if i == 0 else '---:' for i in range(len(header))) + '|',
    ]
    lines += ['| ' + ' | '.join(f'**{total_label}**' if c == total_label else c for c in row) + ' |' for row in rows]
    lines += ['', f'**Columns**, per localizer (several runs at one {word}: the median):']
    if axis == 'noise':
        lines.append('- **Noise level / Actual:** odometry noise level (`odom_noise.py`: every error term is multiplied '
                     'by it; 0 = perfect odometry, 1 ≈ a real car), and the average speed actually driven.')
    else:
        lines.append('- **Speed / Actual:** commanded cruise speed, and the average actually driven (autodrive slows '
                     'in turns).')
    lines += [
        f'- **lost:** the lap in which the error first exceeded {pr.LOST:g} m, `—` if never. '
        'In the last row: runs that lost track.',
        '- **RMSE:** root-mean-square position error over the whole run: the typical error.',
        '- **p95:** 95% of the time the error was below this: the bad moments, without one spike deciding.',
        '- **heading:** root-mean-square heading error.',
    ]
    if n_laps > 1:
        lines.append(f'- **lap 1→{n_laps}:** RMSE of the last lap minus RMSE of the first. Positive = '
                     'the error grows over the laps.')
    lines.append(f'- **{total_label}:** medians over every run of that localizer.')
    flagged = [r for r in ok if r['flags']]
    failed = [r for r in runs if r['status'] != 'ok']
    if flagged or failed:
        lines += ['', '**† Runs to treat with care:**']
        lines += [f"- {pr.NAME[r['localizer']]} {where(r, multi)}: {'; '.join(r['flags'])}" for r in flagged]
        lines += [f"- {pr.NAME[r['localizer']]} {where(r, multi)}: failed, no usable data "
                  f"({r['run_dir']})" for r in failed]
    lines += ['', '**Read with care:**']
    if noisy(runs):
        lines.append('- The odometry is corrupted by `odom_noise.py` (level 1: distance and turns read 2 % too large, '
                     '0.2 °/s gyro bias, random walk 1 cm and 0.3 ° per √m; higher levels multiply every term). Run r '
                     'of every localizer uses noise seed r, so they all get the same corrupted odometry. The start '
                     'pose is correct.')
        if 'odom' in present:
            lines.append('- Odometry only: no localizer, the corrupted odometry alone. Its error is the drift the '
                         'localizers have to remove; with perfect odometry (level 0) it is 0.')
    else:
        lines.append('- Odometry is perfect and the start pose correct, so this measures whether a localizer '
                     '*disturbs* good odometry, not whether it can *correct* drift. A localizer that does nothing '
                     'would score about 0.')
    lines += [
        '- Errors cover only the driving window, not the car parked before and after the laps.',
        f'- Maps: {maps_note(runs, present)}.',
    ]
    if per_point <= 1:
        lines.append(f'- One run per {word}: results vary from run to run (especially AMCL and SLAM Toolbox), '
                     f'so read the trend over {word}s, not single steps.')
    elif axis == 'noise':
        lines.append('- Several runs per level, each with its own noise seed: the cells are medians over them.')
    path.write_text('\n'.join(lines) + '\n')


def write_runs_csv(path, runs):
    keys = ['localizer', 'speed', 'noise', 'run', 'status', 'lost', 'lost_lap', 'pos_rmse', 'pos_p95', 'pos_max', 'yaw_rmse',
            'yaw_max', 'lap_change', 'speed_actual', 'drive_s', 'samples', 'coverage', 'max_gap_s', 'rtf', 'wall_s',
            'lap_done', 'stuck', 'retries', 'reset', 'laps', 'n_laps', 'tracked_share',
            'along_mean', 'along_sd', 'side_mean', 'side_sd', 'head_mean', 'head_sd']
    max_laps = max((r.get('n_laps', 1) for r in runs), default=1)
    keys += [f'lap{k}_{m}' for k in range(1, max_laps + 1) for m in ('pos_rmse', 'yaw_rmse')]
    keys += ['flags', 'commit', 'run_dir']
    with open(path, 'w', newline='') as f:
        out = csv.writer(f)
        out.writerow(keys)
        for r in sorted(runs, key=lambda r: (r['localizer'], float(r['speed']), float(r['noise']), r['run'])):
            row = []
            for k in keys:
                v = r.get(k, '')
                if k == 'flags':
                    v = '; '.join(v) if v else ''
                elif isinstance(v, float):
                    v = '' if math.isnan(v) else f'{v:.4f}'
                row.append(v)
            out.writerow(row)


def plot(runs, present, n_laps, tag, out, axis):
    ok = [r for r in runs if r['status'] == 'ok']
    pos_of = x_positions([r[axis] for r in ok], axis)
    xs = sorted(pos_of.values())
    panels = [('pos_rmse', 'Position error', 'RMSE over the run [cm], log scale', 100, True),
              ('yaw_rmse', 'Heading error', 'RMSE over the run [°], log scale', 1, True)]
    if n_laps > 1:
        panels.append(('lap_change', f'Does the error grow? Lap {n_laps} minus lap 1',
                       'position RMSE change [cm]', 100, False))
    fig_h = 2.9 * len(panels) + 1.4
    fig, axes = plt.subplots(len(panels), 1, figsize=(10, fig_h), sharex=True)
    step = min(np.diff(xs)) if len(xs) > 1 else 0.1
    spread = step * min(0.18, 0.6 / len(present))  # dodge the series sideways, at most ~1/3 of a step in all
    offset = {loc: (i - (len(present) - 1) / 2) * spread for i, loc in enumerate(present)}

    for ax, (key, title, unit, scale, log) in zip(axes, panels):
        end_labels = []
        for loc in present:
            if key == 'lap_change' and loc in REFERENCE:
                continue  # the odometry's drift grows by metres: it would flatten the localizers' lines
            # a log axis can't show 0 (the odometry alone at noise level 0)
            mine = [r for r in ok if r['localizer'] == loc and not math.isnan(r[key]) and (r[key] > 0 or not log)]
            line_x, line_y = [], []
            for v, xv in sorted(pos_of.items(), key=lambda e: e[1]):
                sel = [r for r in mine if r[axis] == v]
                if sel:
                    line_x.append(xv + offset[loc])
                    line_y.append(median([r[key] for r in sel]) * scale)
            ax.plot(line_x, line_y, color=pr.COLOR[loc], lw=1.0, ls='--' if loc in REFERENCE else '-',
                    solid_joinstyle='round', solid_capstyle='round', zorder=2)
            for r in mine:
                x, y = pos_of[r[axis]] + offset[loc], r[key] * scale
                if log and r['n_laps'] > 1:  # whisker: range of the per-lap values
                    laps = [r[f"lap{k}_{key}"] * scale for k in range(1, r['n_laps'] + 1)]
                    ax.vlines(x, min(laps), max(laps), color=pr.COLOR[loc], lw=0.9, alpha=0.55, zorder=2)
                ax.plot(x, y, ls='none', marker='X' if r['lost'] else 'o', ms=7 if r['lost'] else 4.6,
                        color=pr.COLOR[loc], mec=pr.SURFACE, mew=0.9, zorder=3)
            if key == 'pos_rmse' and line_x:  # direct label at the line end (text in ink, not series color)
                end_labels.append((line_x[-1], line_y[-1], pr.NAME[loc].split(' (')[0]))
        if log:
            ax.set_yscale('log')
            ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:g}'))
        else:
            ax.axhline(0, color=pr.TEXT2, lw=0.8, zorder=1)
        # place the labels bottom to top, each at least 11 pt above the previous one so they don't overlap
        ax.get_ylim()  # settles the autoscaled limits, so transData is final
        last = -math.inf
        for x, y, text in sorted(end_labels, key=lambda e: e[1]):
            y_pt = ax.transData.transform((x, y))[1] * 72 / fig.dpi
            placed = max(y_pt, last + 11)
            ax.annotate(text, (x, y), xytext=(9, placed - y_pt), textcoords='offset points', va='center',
                        fontsize=8.5, color=pr.TEXT2)
            last = placed
        ax.set_title(title)
        ax.set_ylabel(unit)
    ordered = sorted(pos_of, key=pos_of.get)
    axes[-1].set_xticks([pos_of[v] for v in ordered], [x_label(v, axis) for v in ordered])
    axes[-1].set_xlabel(AXIS[axis][1])
    axes[-1].set_xlim(xs[0] - step, xs[-1] + step * 3.2)  # room for the direct labels
    for ax in axes:
        ax.grid(axis='x', visible=False)

    handles = []
    for loc in present:
        mine = [r for r in ok if r['localizer'] == loc]
        lost = sum(r['lost'] for r in mine)
        handles.append(Line2D([], [], color=pr.COLOR[loc], lw=1.0, ls='--' if loc in REFERENCE else '-', marker='o',
                              ms=5, mec=pr.SURFACE, label=f'{pr.NAME[loc]}: {lost} of {len(mine)} runs lost track'))
    handles.append(Line2D([], [], ls='none', marker='X', ms=7, color=pr.TEXT2, mec=pr.SURFACE,
                          label=f'lost track (error > {pr.LOST:g} m)'))
    fig.legend(handles=handles, loc='upper center', ncol=2, bbox_to_anchor=(0.5, 0.995))
    legend_top = 1 - (0.12 + 0.24 * math.ceil(len(handles) / 2)) / fig_h  # ~0.24 in per legend row
    whisker = ' Whiskers: lowest to highest per-lap RMSE.' if n_laps > 1 else ''
    if noisy(runs):
        odometry = ('The odometry is corrupted by odom_noise at the level on the x-axis (every error term multiplied by '
                    'it; 1 ≈ a real car), with a correct start.')
        if 'odom' in present:
            odometry += (' Dashed grey: the odometry alone, no localizer: the drift the localizers have to remove '
                         '(0 at level 0, so not drawn on the log axes; left out of the lap-change panel).')
    else:
        odometry = ('Perfect odometry and a correct start: this shows whether a localizer disturbs good odometry, not '
                    'whether it corrects drift.')
    pr.footnote(fig, f'Each dot is one {n_laps}-lap run (lines join the medians per {AXIS[axis][3]}).{whisker} '
                f'{odometry} Errors cover only the driving window. SLAM Toolbox uses its own recorded map, the others '
                f'the ground-truth map. Numbers: results/{tag}_table.md.', 10)
    fig.tight_layout(rect=(0, 0.06, 1, legend_top))
    fig.savefig(out, dpi=160)
    plt.close(fig)


def nanmedian(a, axis=0):
    with warnings.catch_warnings():  # all-NaN columns (no sample in that bin) stay NaN, quietly
        warnings.simplefilter('ignore', RuntimeWarning)
        return np.nanmedian(a, axis=axis)


def binned(r, key):
    """Median of the run's trace[key] in each 1/BINS_PER_LAP of a lap -> (n_laps, BINS_PER_LAP), NaN if no sample."""
    tr, n_laps = r['_trace'], r['n_laps']
    edges = np.arange(n_laps * BINS_PER_LAP + 1) / BINS_PER_LAP
    cut = np.searchsorted(tr['laps'], edges)  # lap progress only grows, so every bin is one slice
    v = tr[key]
    return np.array([np.median(v[a:b]) if b > a else np.nan
                     for a, b in zip(cut[:-1], cut[1:])]).reshape(n_laps, BINS_PER_LAP)


def centerline(ok):
    """Median true path of all runs, one point per bin of the lap: x, y."""
    return (nanmedian(np.vstack([binned(r, 'x') for r in ok])),
            nanmedian(np.vstack([binned(r, 'y') for r in ok])))


def lost_at(r):
    """True position and lap progress where the run's error first exceeded LOST."""
    tr = r['_trace']
    i = int(np.argmax(tr['pos'] > pr.LOST))
    return tr['x'][i], tr['y'][i], tr['laps'][i]


def clusters(points, radius=1.0):
    """Groups points closer than `radius` [m] to a group's centre -> [(centre, count)]."""
    groups = []
    for p in points:
        for g in groups:
            if np.hypot(*(np.mean(g, axis=0) - p)) < radius:
                g.append(p)
                break
        else:
            groups.append([p])
    return [(np.mean(g, axis=0), len(g)) for g in groups]


def error_colorbar(fig, rect):
    cax = fig.add_axes(rect)
    cax.grid(False)  # before colorbar(): matplotlib 3.5 warns otherwise
    cb = fig.colorbar(ScalarMappable(ERR_NORM, ERR_CMAP), cax=cax, orientation='horizontal', extend='both')
    cb.set_ticks(ERR_TICKS)
    cb.set_ticklabels([f'{v * 100:g} cm' if v < 0.5 else f'{v:g} m' for v in ERR_TICKS])
    cax.minorticks_off()
    cax.tick_params(length=0, labelsize=8)
    cb.outline.set_visible(False)
    cax.axvline(pr.LOST, color=pr.TEXT, lw=1.4)
    cax.annotate(f'lost track (> {pr.LOST:g} m)', (pr.LOST, 1), xycoords=('data', 'axes fraction'), xytext=(3, 3),
                 textcoords='offset points', ha='left', va='bottom', fontsize=8, color=pr.TEXT)
    cb.set_label('position error, log scale', fontsize=9, color=pr.TEXT2)


def subtitle(ax, text):
    ax.annotate(text, (0, 1), xycoords='axes fraction', xytext=(0, 3), textcoords='offset points', va='bottom',
                fontsize=8.5, color=pr.TEXT2)


def plot_track_map(runs, present, tag, out):
    """Where on the track each localizer is off: median error along the lap, and where runs lost track."""
    ok = [r for r in runs if r['status'] == 'ok']
    present = [loc for loc in present if loc not in REFERENCE]
    cx, cy = centerline(ok)
    px, py = np.r_[cx, cx[:1]], np.r_[cy, cy[:1]]  # close the loop
    seg = np.stack([np.column_stack([px[:-1], py[:-1]]), np.column_stack([px[1:], py[1:]])], axis=1)
    img, extent = pr.load_map()
    view = pr.track_view(img, extent)
    fig, axes = plt.subplots(1, len(present), figsize=(5.6 * len(present), 5.7), squeeze=False)
    for ax, loc in zip(axes[0], present):
        mine = [r for r in ok if r['localizer'] == loc]
        err = nanmedian(np.vstack([binned(r, 'pos') for r in mine]))
        pr.map_ax(ax, img, extent, view, alpha=0.22)
        ax.add_collection(LineCollection(seg, colors=pr.TEXT2, linewidths=7.5, alpha=0.55, capstyle='round', zorder=1))
        line = LineCollection(seg, cmap=ERR_CMAP.with_extremes(bad=pr.GRID), norm=ERR_NORM, linewidths=5.5,
                              capstyle='round', zorder=2)
        line.set_array(np.ma.masked_invalid(err))
        ax.add_collection(line)
        ax.annotate('', xy=(px[5], py[5]), xytext=(px[0], py[0]), zorder=4,
                    arrowprops=dict(arrowstyle='-|>', color=pr.TEXT, lw=1.3, mutation_scale=13))
        ax.annotate('start', (px[0], py[0]), xytext=(8, -10), textcoords='offset points', fontsize=8, color=pr.TEXT)
        points = [lost_at(r)[:2] for r in mine if r['lost']]
        for lx, ly in points:
            ax.plot(lx, ly, ls='none', marker='X', ms=8.5, color=pr.TEXT, mec=pr.SURFACE, mew=1.1, zorder=5)
        for (lx, ly), n in clusters(points):
            ax.annotate(f'{n} run{"s" if n > 1 else ""}', (lx, ly), xytext=(9, 8), textcoords='offset points',
                        fontsize=8, color=pr.TEXT, zorder=6,
                        bbox=dict(boxstyle='round,pad=0.25', facecolor=pr.SURFACE, edgecolor='none', alpha=0.9))
        lost = sum(r['lost'] for r in mine)
        ax.set_title(pr.NAME[loc], pad=16)
        subtitle(ax, f"typical error {median([r['pos_rmse'] for r in mine]) * 100:.0f} cm  ·  "
                     f"{lost} of {len(mine)} runs lost track")
    fig.legend(handles=[Line2D([], [], ls='none', marker='X', ms=8.5, color=pr.TEXT, mec=pr.SURFACE,
                               label=f'where a run first went over {pr.LOST:g} m (label: how many runs)')],
               loc='upper center', bbox_to_anchor=(0.5, 0.995))
    fig.tight_layout(rect=(0, 0.21, 1, 0.92))
    error_colorbar(fig, (0.33, 0.135, 0.34, 0.03))
    pr.footnote(fig, 'Line colour: median position error at each point of the lap (bins of ~0.37 m), over every run, '
                'speed, noise level and lap of that localizer. A run that lost track counts there too, but the median '
                'follows the typical run. Typical error = median of the runs\' RMSE. '
                f'{odometry_note(runs)} Numbers: results/{tag}_table.md.', 5.6 * len(present))
    fig.savefig(out, dpi=160)
    plt.close(fig)


def plot_offsets(runs, present, tag, out, axis):
    """Signed error in the car's frame per run: the constant part (mean) and the random part (SD)."""
    ok = [r for r in runs if r['status'] == 'ok']
    present = [loc for loc in present if loc not in REFERENCE]
    pos_of = x_positions([r[axis] for r in ok], axis)
    xs = sorted(pos_of.values())
    step = min(np.diff(xs)) if len(xs) > 1 else 0.1
    offset = {loc: (i - (len(present) - 1) / 2) * step * 0.22 for i, loc in enumerate(present)}
    panels = [('along', 'Along the car', '+ = estimate ahead of the car, − = behind it', 'cm', 100),
              ('side', 'Sideways', '+ = estimate left of the car, − = right of it', 'cm', 100),
              ('head', 'Heading', '+ = estimate turned counter-clockwise, − = clockwise', '°', 1)]
    fig, axes = plt.subplots(len(panels), 1, figsize=(11.5, 10.2), sharex=True)
    for ax, (key, title, sign, unit, scale) in zip(axes, panels):
        handles, lims = [], [0.0]
        for loc in present:
            mine = [r for r in ok if r['localizer'] == loc and not math.isnan(r[f'{key}_mean'])]
            for r in mine:
                xx, m, sd = pos_of[r[axis]] + offset[loc], r[f'{key}_mean'] * scale, r[f'{key}_sd'] * scale
                ax.vlines(xx, m - sd, m + sd, color=pr.COLOR[loc], lw=1.1, alpha=0.55, zorder=2)
                ax.plot(xx, m, ls='none', marker='X' if r['lost'] else 'o', ms=7 if r['lost'] else 4.8,
                        color=pr.COLOR[loc], mec=pr.SURFACE, mew=0.9, zorder=3)
                lims += [m] if r['lost'] else [m - sd, m + sd]  # a lost run's bar may run off the axis
            const = median([r[f'{key}_mean'] * scale for r in mine])
            rand = median([r[f'{key}_sd'] * scale for r in mine])
            handles.append(Line2D([], [], ls='none', marker='o', ms=5, color=pr.COLOR[loc], mec=pr.SURFACE,
                                  label=f'{SHORT.get(loc, loc)}: {const:+.1f} ± {rand:.1f} {unit}'))
        lo, hi = min(lims), max(lims)
        ax.set_ylim(lo - 0.1 * (hi - lo), hi + 0.1 * (hi - lo))
        ax.axhline(0, color=pr.TEXT2, lw=0.8, zorder=1)
        ax.set_title(title, pad=16)
        subtitle(ax, sign)
        ax.set_ylabel(f'error [{unit}]')
        ax.grid(axis='x', visible=False)
        leg = ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(1.01, 1.0),
                        title='median over runs:\nconstant ± random', title_fontsize=8.5)
        leg._legend_box.align = 'left'
    ordered = sorted(pos_of, key=pos_of.get)
    axes[-1].set_xticks([pos_of[v] for v in ordered], [x_label(v, axis) for v in ordered])
    axes[-1].set_xlabel(AXIS[axis][1])
    axes[-1].set_xlim(xs[0] - step, xs[-1] + step)

    handles = [Line2D([], [], ls='none', marker='o', ms=5.5, color=pr.COLOR[loc], mec=pr.SURFACE,
                      label=pr.NAME[loc]) for loc in present]
    handles += [Line2D([], [], ls='none', marker='o', ms=5.5, color=pr.TEXT2, mec=pr.SURFACE,
                       label='dot: constant part (mean of the run)'),
                Line2D([], [], color=pr.TEXT2, lw=1.1, alpha=0.55, label='bar: ± 1 SD, the random part'),
                Line2D([], [], ls='none', marker='X', ms=7, color=pr.TEXT2, mec=pr.SURFACE,
                       label=f'run lost track (only its samples under {pr.LOST:g} m count)')]
    fig.legend(handles=handles, loc='upper center', ncol=3, bbox_to_anchor=(0.45, 0.995))
    pr.footnote(fig, 'Signed error of the estimate against the true pose, turned into the car\'s frame, over the samples '
                f'while the error was below {pr.LOST:g} m. A constant part points to the setup (laser position, map '
                'alignment); the random part is the localizer\'s jitter. A constant part that grew with speed would '
                'mean a time delay. RMSE² ≈ constant² + random². The y-range fits the runs that kept track, so bars '
                f'of lost runs can run off it. Driving window only. {odometry_note(runs)}', 11.5)
    fig.tight_layout(rect=(0, 0.05, 1, 0.93))
    fig.savefig(out, dpi=160)
    plt.close(fig)


def plot_heatmap(runs, present, n_laps, tag, out, axis):
    """Error along the laps for every run: rows = runs (by speed or noise level), columns = position on the track."""
    ok = [r for r in runs if r['status'] == 'ok' and r['n_laps'] == n_laps]
    pos_of = x_positions([r[axis] for r in ok], axis)
    cx, cy = centerline(ok)
    edges = np.arange(n_laps * BINS_PER_LAP + 1) / BINS_PER_LAP
    multi = any(r['run'] > 1 for r in ok)
    present = [loc for loc in present if any(r['localizer'] == loc for r in ok)]  # only runs with all n_laps laps
    fig, axes = plt.subplots(len(present), 1, figsize=(12, 2.9 * len(present) + 2.2), sharex=True, squeeze=False)
    axes = axes[:, 0]
    for ax, loc in zip(axes, present):
        mine = sorted((r for r in ok if r['localizer'] == loc), key=lambda r: (pos_of[r[axis]], r['run']))
        # an error of exactly 0 (the odometry alone at noise level 0) can't go on the log scale: draw it as the
        # lowest colour, not as missing data
        grid = np.vstack([binned(r, 'pos').ravel() for r in mine])
        grid[grid <= 0] = ERR_NORM.vmin / 10
        ax.grid(False)  # before pcolormesh(): matplotlib 3.5 warns otherwise
        ax.pcolormesh(edges, np.arange(len(mine) + 1), np.ma.masked_invalid(grid), shading='flat', rasterized=True,
                      cmap=ERR_CMAP.with_extremes(bad=pr.TEXT2), norm=ERR_NORM)
        for i, r in enumerate(mine):
            if r['lost']:
                ax.plot(lost_at(r)[2], i + 0.5, ls='none', marker='X', ms=6.5, color=pr.TEXT, mec=pr.SURFACE,
                        mew=0.9, zorder=4)
        for k in range(1, n_laps):
            ax.axvline(k, color=pr.SURFACE, lw=2, zorder=3)
        ax.set_yticks(np.arange(len(mine)) + 0.5,
                      [r[axis] + (f" #{r['run']}" if multi else '') for r in mine])
        ax.tick_params(axis='y', labelsize=7, length=0)
        ax.set_ylabel('commanded speed [m/s]' if axis == 'speed' else 'odometry noise level')
        lost = sum(r['lost'] for r in mine)
        ax.set_title(pr.NAME[loc], pad=30 if ax is axes[0] else 16)  # top panel: room for the corner band
        subtitle(ax, f'{lost} of {len(mine)} runs lost track')

    # corners of plot_results.CORNERS as a band above the top panel, for orientation
    top = axes[0]
    for name, (x0, x1, y0, y1) in pr.CORNERS:
        inside = np.tile((cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1), n_laps).astype(int)
        change = np.diff(np.r_[0, inside, 0])
        segs = [(a / BINS_PER_LAP, b / BINS_PER_LAP) for a, b in zip(np.nonzero(change == 1)[0],
                                                                      np.nonzero(change == -1)[0])]
        for a, b in segs:
            top.plot([a, b], [1.035, 1.035], transform=top.get_xaxis_transform(), color=pr.TEXT2, lw=3,
                     solid_capstyle='butt', clip_on=False)
        if segs:  # label the first segment that isn't cut short by the start of the run
            widest = max(b - a for a, b in segs)
            a, b = next(s for s in segs if s[1] - s[0] > 0.8 * widest)
            top.annotate(name, ((a + b) / 2, 1.035), xycoords=('data', 'axes fraction'), xytext=(0, 4),
                         textcoords='offset points', ha='center', va='bottom', fontsize=8, color=pr.TEXT2)

    ticks = np.arange(0, n_laps + 0.01, 0.25)
    axes[-1].set_xticks(ticks, [f'{t:g}' for t in ticks])
    axes[-1].set_xlim(0, n_laps)
    axes[-1].set_xlabel('laps driven (white lines: start of the next lap)')
    fig.legend(handles=[Line2D([], [], ls='none', marker='X', ms=7, color=pr.TEXT, mec=pr.SURFACE,
                               label=f'where the run first went over {pr.LOST:g} m')],
               loc='upper center', bbox_to_anchor=(0.5, 0.995))
    fig.tight_layout(rect=(0, 0.13, 1, 0.955))
    error_colorbar(fig, (0.33, 0.07, 0.34, 0.016))
    pr.footnote(fig, 'One row per run, one cell per ~0.37 m of track (median error in that stretch). Dark grey cells: '
                'no estimate there. The same column in every lap is the same place on the track, so an error that '
                f'repeats at one column is tied to that place, not to the {AXIS[axis][3]}. {odometry_note(runs)}', 12)
    fig.savefig(out, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--tag', action='append', metavar='TAG[:LOCALIZERS]',
                        help='benchmark tag (default speedsweep). Repeat it to combine sweeps; TAG:amcl,scan takes '
                             'only those localizers from that tag')
    parser.add_argument('--name', help='name of the output files (default: the tag, or the tags joined by "_vs_")')
    args = parser.parse_args()
    specs = [(t.split(':')[0], set(t.split(':')[1].split(',')) if ':' in t else None)
             for t in (args.tag or ['speedsweep'])]
    tag = args.name or '_vs_'.join(t for t, _ in specs)
    runs = [r for t, only in specs for r in load_runs(t, only)]
    running = sum(r['status'] == 'running' for r in runs)
    runs = [r for r in runs if r['status'] != 'running']
    if not runs:
        raise SystemExit(f"No finished runs for {', '.join(t for t, _ in specs)} in results/"
                         + (f' ({running} still running)' if running else ''))
    for loc in sorted({r['localizer'] for r in runs}):
        maps = sorted({r['map'] for r in runs if r['localizer'] == loc and r.get('map') not in (None, '', 'none')})
        if len(maps) > 1:
            raise SystemExit(f"{pr.NAME[loc]} ran on several maps here ({', '.join(maps)}). Take each localizer "
                             'from one map, e.g. --tag <new>:amcl --tag <old>:scan,slam')
    if len({r['speed'] for r in runs}) > 1 and len({r['noise'] for r in runs}) > 1:
        raise SystemExit('These runs vary both the speed and the odometry noise level. Report one at a time, e.g. a '
                         'noise sweep at one speed.')
    axis = 'noise' if len({r['noise'] for r in runs}) > 1 else 'speed'
    present = [k for k, _, _ in pr.LOCALIZERS if any(r['localizer'] == k for r in runs)]
    ok = [r for r in runs if r['status'] == 'ok']
    n_laps = max((r['n_laps'] for r in ok), default=1)

    header, rows = build_table(runs, present, n_laps, axis)
    with open(pr.RESULTS / f'{tag}_table.csv', 'w', newline='') as f:
        csv.writer(f).writerows([header] + rows)
    write_markdown(pr.RESULTS / f'{tag}_table.md', tag, header, rows, runs, present, n_laps, running, axis)
    write_runs_csv(pr.RESULTS / f'{tag}_runs.csv', runs)
    written = [f'results/{tag}_table.md', f'results/{tag}_table.csv', f'results/{tag}_runs.csv']
    if ok:
        (pr.RESULTS / 'plots').mkdir(exist_ok=True)
        plot(runs, present, n_laps, tag, pr.RESULTS / 'plots' / f'{tag}.png', axis)
        plot_track_map(runs, present, tag, pr.RESULTS / 'plots' / f'{tag}_track.png')
        plot_offsets(runs, present, tag, pr.RESULTS / 'plots' / f'{tag}_offsets.png', axis)
        plot_heatmap(runs, present, n_laps, tag, pr.RESULTS / 'plots' / f'{tag}_heatmap.png', axis)
        written += [f'results/plots/{tag}{s}.png' for s in ('', '_track', '_offsets', '_heatmap')]
    print(f'{len(runs)} finished runs ({len(runs) - len(ok)} failed'
          + (f', {running} still running' if running else '') + '). Wrote: ' + ', '.join(written))


if __name__ == '__main__':
    main()
