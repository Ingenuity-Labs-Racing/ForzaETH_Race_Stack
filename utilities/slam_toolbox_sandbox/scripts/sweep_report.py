#!/usr/bin/env python3
"""
Summary table and figure of a speed sweep (scripts/run_speed_sweep.sh), or of any benchmark tag.

    python3 scripts/sweep_report.py [--tag speedsweep]

  results/<tag>_table.md, results/<tag>_table.csv   one row per speed, the main metrics per localizer
  results/<tag>_runs.csv                             one row per run, every metric (also per lap)
  results/plots/<tag>.png                            position / heading error and lap-to-lap change over speed

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
from pathlib import Path

import numpy as np

import plot_results as pr  # shared colors, names, chart style (sets the matplotlib backend and rcParams)
import matplotlib.pyplot as plt  # noqa: E402  (after plot_results picked the backend)
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402

MOVE = 0.05          # [m] the car counts as driving once it is this far from its start / final position
GT_HZ = 50           # /ground_truth rate. pose_error skips samples it can't match in time (normally 25-50%)
MAX_GAP = 1.0        # [s] flag runs with a longer stretch without an estimate (normal runs: 0.1-0.4 s)
MIN_RTF = 0.5        # flag runs where the sim ran slower than half real time
SHORT = {'scan': 'Scan', 'amcl': 'AMCL', 'slam': 'SLAM TB'}
RUN_FILE = re.compile(r'^(\w+)_speed_([\d.]+)_run(\d+)\.sim\.log$')


def read_text(path):
    try:
        return path.read_text(errors='replace')
    except OSError:
        return ''


def rms(a):
    return float(np.sqrt(np.mean(np.square(a)))) if len(a) else math.nan


def summary_rows(tag):
    """(run dir, localizer, speed, run) -> summary.csv row, for the laps setting and the commit."""
    try:
        rows = list(csv.DictReader(open(pr.RESULTS / 'summary.csv')))
    except OSError:
        return {}
    return {(r.get('run_dir', ''), r['localizer'], r['speed'], int(r['run'])): r for r in rows if r['tag'] == tag}


def run_metrics(loc, speed, run, base, run_dir, summary):
    """Every metric of one run. base = results dir / '<loc>_speed_<speed>_run<run>'."""
    file = lambda ext: Path(f'{base}.{ext}')  # noqa: E731
    row = summary.get((run_dir, loc, speed, run))
    drive_log = read_text(file('autodrive.log'))
    if row is None:
        # eval_localization.sh adds the summary row when a run ends (also a failed one). Without it the
        # run is either still going (its files keep changing) or from before run_dir was recorded.
        newest = max((p.stat().st_mtime for p in base.parent.glob(f'{base.name}.*')), default=0)
        if time.time() - newest < 600:
            return {'localizer': loc, 'speed': speed, 'run': run, 'run_dir': run_dir, 'status': 'running'}
        done = re.search(r'Done: ([\d.]+) laps', drive_log)
        row = {'laps': done.group(1) if done else '1'}
    wall = re.search(r'wall_s=(\d+)', drive_log)
    r = {
        'localizer': loc, 'speed': speed, 'run': run, 'run_dir': run_dir, 'commit': row.get('commit', ''),
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
    c = {k: np.array([float(v[k]) for v in rows]) for k in ('t', 'gt_x', 'gt_y', 'pos_err', 'yaw_err')}

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


def load_runs(tag):
    summary = summary_rows(tag)
    runs = []
    for d in sorted(p for p in pr.RESULTS.glob(f'*_{tag}') if p.is_dir()):
        for sim_log in sorted(d.glob('*_speed_*_run*.sim.log')):
            m = RUN_FILE.match(sim_log.name)
            if m:
                base = sim_log.with_name(sim_log.name[:-len('.sim.log')])
                runs.append(run_metrics(m.group(1), m.group(2), int(m.group(3)), base, d.name, summary))
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


def build_table(runs, present, n_laps):
    header = ['Speed [m/s]', 'Actual [m/s]']
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
    for speed in sorted({r['speed'] for r in runs}, key=float):
        at = [r for r in runs if r['speed'] == speed]
        row = [speed, num(median([r['speed_actual'] for r in at if r['status'] == 'ok']), 2)]
        for loc in present:
            row += cells([r for r in at if r['localizer'] == loc])
        rows.append(row)
    total = ['All speeds', '']
    for loc in present:
        total += cells([r for r in runs if r['localizer'] == loc], all_speeds=True)
    rows.append(total)
    return header, rows


def write_markdown(path, tag, header, rows, runs, present, n_laps, running):
    commits = sorted({r['commit'] for r in runs if r['commit']})
    ok = [r for r in runs if r['status'] == 'ok']
    per_speed = len(ok) / max(len({(r['localizer'], r['speed']) for r in ok}), 1)
    lines = [
        f'# Localization speed sweep (`{tag}`)', '',
        f"{len(runs)} runs: {', '.join(pr.NAME[k] for k in present)}; "
        f"{'one' if per_speed <= 1 else f'{per_speed:.0f}'} {n_laps}-lap run{'s' if per_speed > 1 else ''} "
        f"per speed. Code: {', '.join(commits) or 'unknown'}. Figure: `results/plots/{tag}.png`."
        + (f' **{running} more runs still in progress**: this table is partial.' if running else ''), '',
        '| ' + ' | '.join(header) + ' |',
        '|' + '|'.join('---' if i == 0 else '---:' for i in range(len(header))) + '|',
    ]
    lines += ['| ' + ' | '.join('**All speeds**' if c == 'All speeds' else c for c in row) + ' |' for row in rows]
    lines += [
        '', '**Columns**, per localizer (several runs at one speed: the median):',
        '- **Speed / Actual:** commanded cruise speed, and the average actually driven (autodrive slows in turns).',
        f'- **lost:** the lap in which the error first exceeded {pr.LOST:g} m, `—` if never. '
        'In the last row: runs that lost track.',
        '- **RMSE:** root-mean-square position error over the whole run: the typical error.',
        '- **p95:** 95% of the time the error was below this: the bad moments, without one spike deciding.',
        '- **heading:** root-mean-square heading error.',
    ]
    if n_laps > 1:
        lines.append(f'- **lap 1→{n_laps}:** RMSE of the last lap minus RMSE of the first. Positive = '
                     'the error grows over the laps.')
    lines.append('- **All speeds:** medians over every run of that localizer.')
    flagged = [r for r in ok if r['flags']]
    failed = [r for r in runs if r['status'] != 'ok']
    if flagged or failed:
        lines += ['', '**† Runs to treat with care:**']
        lines += [f"- {pr.NAME[r['localizer']]} at {r['speed']} m/s: {'; '.join(r['flags'])}" for r in flagged]
        lines += [f"- {pr.NAME[r['localizer']]} at {r['speed']} m/s: failed, no usable data "
                  f"({r['run_dir']})" for r in failed]
    lines += [
        '', '**Read with care:**',
        '- Odometry is perfect and the start pose correct, so this measures whether a localizer *disturbs* '
        'good odometry, not whether it can *correct* drift. A localizer that does nothing would score about 0.',
        '- Errors cover only the driving window, not the car parked before and after the laps.',
        '- SLAM Toolbox localizes on its own recorded map (`victoria_ref`); AMCL and the scan matcher on the '
        'map the world was generated from (`Victoria.yaml`).',
    ]
    if per_speed <= 1:
        lines.append('- One run per speed: results vary from run to run (especially AMCL and SLAM Toolbox), '
                     'so read the trend over speeds, not single steps.')
    path.write_text('\n'.join(lines) + '\n')


def write_runs_csv(path, runs):
    keys = ['localizer', 'speed', 'run', 'status', 'lost', 'lost_lap', 'pos_rmse', 'pos_p95', 'pos_max', 'yaw_rmse',
            'yaw_max', 'lap_change', 'speed_actual', 'drive_s', 'samples', 'coverage', 'max_gap_s', 'rtf', 'wall_s',
            'lap_done',
            'stuck', 'retries', 'reset', 'laps', 'n_laps']
    max_laps = max((r.get('n_laps', 1) for r in runs), default=1)
    keys += [f'lap{k}_{m}' for k in range(1, max_laps + 1) for m in ('pos_rmse', 'yaw_rmse')]
    keys += ['flags', 'commit', 'run_dir']
    with open(path, 'w', newline='') as f:
        out = csv.writer(f)
        out.writerow(keys)
        for r in sorted(runs, key=lambda r: (r['localizer'], float(r['speed']), r['run'])):
            row = []
            for k in keys:
                v = r.get(k, '')
                if k == 'flags':
                    v = '; '.join(v) if v else ''
                elif isinstance(v, float):
                    v = '' if math.isnan(v) else f'{v:.4f}'
                row.append(v)
            out.writerow(row)


def plot(runs, present, n_laps, tag, out):
    ok = [r for r in runs if r['status'] == 'ok']
    speeds = sorted({float(r['speed']) for r in ok})
    panels = [('pos_rmse', 'Position error', 'RMSE over the run [cm], log scale', 100, True),
              ('yaw_rmse', 'Heading error', 'RMSE over the run [°], log scale', 1, True)]
    if n_laps > 1:
        panels.append(('lap_change', f'Does the error grow? Lap {n_laps} minus lap 1',
                       'position RMSE change [cm]', 100, False))
    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 2.9 * len(panels) + 1.4), sharex=True)
    step = min(np.diff(speeds)) if len(speeds) > 1 else 0.1
    offset = {loc: (i - (len(present) - 1) / 2) * step * 0.18 for i, loc in enumerate(present)}

    for ax, (key, title, unit, scale, log) in zip(axes, panels):
        for loc in present:
            mine = [r for r in ok if r['localizer'] == loc and not math.isnan(r[key])]
            line_x, line_y = [], []
            for s in speeds:
                sel = [r for r in mine if float(r['speed']) == s]
                if sel:
                    line_x.append(s + offset[loc])
                    line_y.append(median([r[key] for r in sel]) * scale)
            ax.plot(line_x, line_y, color=pr.COLOR[loc], lw=1.0, solid_joinstyle='round', solid_capstyle='round',
                    zorder=2)
            for r in mine:
                x, y = float(r['speed']) + offset[loc], r[key] * scale
                if log and r['n_laps'] > 1:  # whisker: range of the per-lap values
                    laps = [r[f"lap{k}_{key}"] * scale for k in range(1, r['n_laps'] + 1)]
                    ax.vlines(x, min(laps), max(laps), color=pr.COLOR[loc], lw=0.9, alpha=0.55, zorder=2)
                ax.plot(x, y, ls='none', marker='X' if r['lost'] else 'o', ms=7 if r['lost'] else 4.6,
                        color=pr.COLOR[loc], mec=pr.SURFACE, mew=0.9, zorder=3)
            if key == 'pos_rmse' and line_x:  # direct label at the line end (text in ink, not series color)
                ax.annotate(pr.NAME[loc].split(' (')[0], (line_x[-1], line_y[-1]), xytext=(9, 0),
                            textcoords='offset points', va='center', fontsize=8.5, color=pr.TEXT2)
        if log:
            ax.set_yscale('log')
            ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:g}'))
        else:
            ax.axhline(0, color=pr.TEXT2, lw=0.8, zorder=1)
        ax.set_title(title)
        ax.set_ylabel(unit)
    axes[-1].set_xticks(speeds, [f'{s:.2f}' for s in speeds])
    axes[-1].set_xlabel('commanded speed [m/s]')
    axes[-1].set_xlim(speeds[0] - step, speeds[-1] + step * 3.2)  # room for the direct labels
    for ax in axes:
        ax.grid(axis='x', visible=False)

    handles = []
    for loc in present:
        mine = [r for r in ok if r['localizer'] == loc]
        lost = sum(r['lost'] for r in mine)
        handles.append(Line2D([], [], color=pr.COLOR[loc], lw=1.0, marker='o', ms=5, mec=pr.SURFACE,
                              label=f'{pr.NAME[loc]}: {lost} of {len(mine)} runs lost track'))
    handles.append(Line2D([], [], ls='none', marker='X', ms=7, color=pr.TEXT2, mec=pr.SURFACE,
                          label=f'lost track (error > {pr.LOST:g} m)'))
    fig.legend(handles=handles, loc='upper center', ncol=2, bbox_to_anchor=(0.5, 0.995))
    whisker = ' Whiskers: lowest to highest per-lap RMSE.' if n_laps > 1 else ''
    pr.footnote(fig, f'Each dot is one {n_laps}-lap run (lines join the medians per speed).{whisker} Perfect odometry '
                'and a correct start: this shows whether a localizer disturbs good odometry, not whether it corrects '
                'drift. Errors cover only the driving window. SLAM Toolbox uses its own recorded map, the others the '
                f'ground-truth map. Numbers: results/{tag}_table.md.', 10)
    fig.tight_layout(rect=(0, 0.06, 1, 0.945))
    fig.savefig(out, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--tag', default='speedsweep')
    tag = parser.parse_args().tag
    runs = load_runs(tag)
    running = sum(r['status'] == 'running' for r in runs)
    runs = [r for r in runs if r['status'] != 'running']
    if not runs:
        raise SystemExit(f'No finished runs for tag "{tag}" in results/*_{tag}/'
                         + (f' ({running} still running)' if running else ''))
    present = [k for k, _, _ in pr.LOCALIZERS if any(r['localizer'] == k for r in runs)]
    ok = [r for r in runs if r['status'] == 'ok']
    n_laps = max((r['n_laps'] for r in ok), default=1)

    header, rows = build_table(runs, present, n_laps)
    with open(pr.RESULTS / f'{tag}_table.csv', 'w', newline='') as f:
        csv.writer(f).writerows([header] + rows)
    write_markdown(pr.RESULTS / f'{tag}_table.md', tag, header, rows, runs, present, n_laps, running)
    write_runs_csv(pr.RESULTS / f'{tag}_runs.csv', runs)
    written = [f'results/{tag}_table.md', f'results/{tag}_table.csv', f'results/{tag}_runs.csv']
    if ok:
        (pr.RESULTS / 'plots').mkdir(exist_ok=True)
        plot(runs, present, n_laps, tag, pr.RESULTS / 'plots' / f'{tag}.png')
        written.append(f'results/plots/{tag}.png')
    print(f'{len(runs)} finished runs ({len(runs) - len(ok)} failed'
          + (f', {running} still running' if running else '') + '). Wrote: ' + ', '.join(written))


if __name__ == '__main__':
    main()
