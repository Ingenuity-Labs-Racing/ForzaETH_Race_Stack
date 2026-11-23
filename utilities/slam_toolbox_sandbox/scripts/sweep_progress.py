#!/usr/bin/env python3
"""
Live progress of a sweep started by run_speed_sweep.sh (speeds, odometry noise levels, repeats):
one overall bar and one line per worker, refreshed every 2 s.

    python3 scripts/sweep_progress.py [--tag speedsweep]    (or: ./scripts/run_speed_sweep.sh watch)

Only reads files - results/<tag>_plan.json, the worker logs results/<tag>_worker<N>.log and the
autodrive log of each worker's current run - so Ctrl+C stops watching, not the sweep.
The overall percentage is weighted by expected driving time, so a 0.20 m/s run counts 5x a 1.0 m/s one.
"""
import argparse
import json
import re
import sys
import time
from pathlib import Path

try:
    from tqdm import tqdm
except ImportError:  # pip-installed in the container, not part of the Docker image
    tqdm = None

RESULTS = Path(__file__).resolve().parents[1] / 'results'
LAP_M = 36.8      # [m] Victoria centerline lap
OVERHEAD_S = 30   # [s] per run: sim start, localizer start, shutdown
REFRESH_S = 2

# "[tag] slam, speed 1.0 m/s, noise 2, run 3 ..." (before 2026-11-22 without the noise part and run number)
RUN_START = re.compile(r'^\[[^\]]*\] (\w+), speed ([\d.]+) m/s(?:, noise ([\d.]+))?, run (\d+)?')
LAPS_DONE = re.compile(r'([\d.]+) laps')


def run_work(speed, laps):
    """Expected seconds for one run."""
    return laps * LAP_M / float(speed) + OVERHEAD_S


def run_name(loc, speed, noise, run):
    """Base name of a run's files, as eval_localization.sh writes them."""
    return f"{loc}_speed_{speed}{f'_noise{noise}' if float(noise) > 0 else ''}_run{run}"


def describe(loc, speed, noise, run):
    return f"{loc:<4} {speed:>4} m/s" + (f" noise {noise}" if float(noise) > 0 else '') + (f" #{run}" if run != '1' else '')


def hms(s):
    s = int(max(s, 0))
    return f'{s // 3600}:{s // 60 % 60:02d}:{s % 60:02d}'


def latest_laps(log):
    """Laps driven so far, from the newest "X.XX laps" line of an autodrive log."""
    try:
        text = log.read_text(errors='replace')
    except OSError:
        return None
    found = LAPS_DONE.findall(text)
    return float(found[-1]) if found else None


class Worker:
    def __init__(self, num, cfg, plan):
        self.num, self.delay, self.laps = num, cfg['delay_s'], plan['laps']
        # runs as localizer:speed:noise:run (plans before 2026-11-22: every localizer at each of 'speeds')
        jobs = cfg.get('jobs') or [f'{loc}:{s}:0:1' for loc in plan['localizers'] for s in cfg['speeds']]
        self.log = RESULTS / f"{plan['tag']}_worker{num}.log"
        self.total = sum(run_work(j.split(':')[1], self.laps) for j in jobs)
        self.runs_total = len(jobs)

    def update(self):
        """Parses the worker log. Sets done work, finished runs, failures and the current run."""
        self.out, self.current, self.finished = None, None, False
        self.done_runs, self.failed, self.retries, self.work = [], [], 0, 0.0
        try:
            lines = self.log.read_text(errors='replace').splitlines()
        except OSError:
            lines = []
        for line in lines:
            if line.startswith('Output: '):
                self.out = Path(line[len('Output: '):].strip())
            elif m := RUN_START.match(line):
                self.current = (m.group(1), m.group(2), m.group(3) or '0', m.group(4) or '1')
            elif line.startswith('  RETRY'):
                self.retries += 1
            elif self.current and (line.startswith('  lap done:') or line.startswith('  FAIL:')):
                ok = line.startswith('  lap done: true')
                self.done_runs.append(self.current)
                if not ok:
                    self.failed.append((*self.current, line.strip()))
                self.work += run_work(self.current[1], self.laps)
                self.current = None
            elif line.startswith('All runs:'):
                self.finished = True
        self.lap = None
        if self.current and self.out:
            self.lap = latest_laps(self.out / f'{run_name(*self.current)}.autodrive.log')
            self.work += OVERHEAD_S / 2 + (self.lap or 0) * LAP_M / float(self.current[1])

    def eta(self, elapsed):
        """Seconds left for this worker, from its own pace so far (None until it has some)."""
        busy = elapsed - self.delay
        if self.finished or self.work >= self.total:
            return 0.0
        if busy < 60 or self.work <= 0:
            return None
        return (self.total - self.work) * busy / self.work

    def status(self, elapsed, alive):
        if self.finished:
            return f'w{self.num}  finished', 1.0
        if not alive:
            return f'w{self.num}  stopped (worker not running)', 0.0
        if elapsed < self.delay and not self.out:
            return f'w{self.num}  starts in {int(self.delay - elapsed)} s', 0.0
        if not self.current:
            return f'w{self.num}  between runs', 0.0
        run = describe(*self.current)
        if self.lap is None:
            return f'w{self.num}  {run}  starting sim + localizer', 0.0
        return f'w{self.num}  {run}  lap {self.lap:.2f}/{self.laps:g}', self.lap / self.laps


def alive_pids(tag):
    try:
        pids = [int(p) for p in (RESULTS / f'{tag}.pids').read_text().split()]
    except OSError:
        return set()
    alive = set()
    for p in pids:
        try:
            Path(f'/proc/{p}').stat()
            alive.add(p)
        except OSError:
            pass
    return alive


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--tag', default='speedsweep')
    tag = parser.parse_args().tag
    try:
        plan = json.loads((RESULTS / f'{tag}_plan.json').read_text())
    except OSError:
        sys.exit(f'No sweep "{tag}" found (results/{tag}_plan.json). Start one: ./scripts/run_speed_sweep.sh')

    workers = [Worker(n, cfg, plan) for n, cfg in sorted(plan['workers'].items())]
    pids = [int(p) for p in (RESULTS / f'{tag}.pids').read_text().split()] if (RESULTS / f'{tag}.pids').exists() else []
    total = sum(w.total for w in workers)
    runs_total = sum(w.runs_total for w in workers)

    bars = []
    if tqdm:
        # the summary goes in desc: tqdm would put ", " in front of a postfix
        bars.append(tqdm(total=1000, position=0, dynamic_ncols=True,
                         bar_format=f'Sweep {tag} {{percentage:3.0f}}%|{{bar:30}}| {{desc}}'))
        for i, _ in enumerate(workers):
            bars.append(tqdm(total=1000, position=i + 1, dynamic_ncols=True, bar_format='  {desc:<56} |{bar:24}|'))
    last_plain = 0.0
    try:
        while True:
            elapsed = time.time() - plan['started']
            alive = alive_pids(tag)
            for w in workers:
                w.update()
            done = sum(w.work for w in workers)
            runs_done = sum(len(w.done_runs) for w in workers)
            failed = sum(len(w.failed) for w in workers)
            retries = sum(w.retries for w in workers)
            etas = [w.eta(elapsed) for w in workers]
            eta = '?' if any(e is None for e in etas) else hms(max(etas))
            summary = (f'{runs_done}/{runs_total} runs | {failed} failed | {retries} retr{"y" if retries == 1 else "ies"} | '
                       f'running {hms(elapsed)} | ETA ~{eta}')
            lines = [w.status(elapsed, pid in alive) for w, pid in zip(workers, pids or [None] * len(workers))]
            if bars:
                bars[0].n = int(1000 * min(done / total, 1.0))
                bars[0].set_description_str(summary, refresh=False)
                bars[0].refresh()
                for bar, (text, frac) in zip(bars[1:], lines):
                    bar.n = int(1000 * frac)
                    bar.set_description_str(text, refresh=False)
                    bar.refresh()
            elif time.time() - last_plain >= 30:
                last_plain = time.time()
                print(f'[{time.strftime("%H:%M:%S")}] {100 * done / total:3.0f}% | {summary} | '
                      + ' | '.join(t for t, _ in lines), flush=True)
            all_stopped = pids and not alive
            if all(w.finished for w in workers) or all_stopped:
                break
            time.sleep(REFRESH_S)
    except KeyboardInterrupt:
        for bar in bars:
            bar.close()
        print('\nStopped watching - the sweep keeps running. Re-attach: ./scripts/run_speed_sweep.sh watch')
        return

    for bar in bars:
        bar.close()
    if not all(w.finished for w in workers):
        print(f'\nSweep "{tag}" is no longer running, but not every worker finished (stopped?).')
    else:
        print(f'\nSweep "{tag}" finished in {hms(elapsed)}: {runs_done}/{runs_total} runs, {failed} failed, '
              f'{retries} retr{"y" if retries == 1 else "ies"}.')
    for w in workers:
        for loc, speed, noise, run, why in w.failed:
            print(f'  failed: {describe(loc, speed, noise, run)} - {why}')
    print(f'Table and figure: python3 scripts/sweep_report.py --tag {tag}')


if __name__ == '__main__':
    main()
