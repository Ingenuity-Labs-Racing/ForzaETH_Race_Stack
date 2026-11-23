#! /bin/bash
# Speed sweep: every localizer at 17 speeds (0.20-1.00 m/s), one 3-lap run per speed, split over
# parallel eval_localization.sh workers (each in its own Gazebo partition and ROS domain, -i 1..N).
# Also sweeps odometry noise levels (NOISE) and repeats (REPEATS). The workers run detached:
# Ctrl+C or closing the terminal only stops the progress view.
#
#   ./scripts/run_speed_sweep.sh          start the sweep and show its progress
#   ./scripts/run_speed_sweep.sh watch    show the progress again (from any container terminal)
#   ./scripts/run_speed_sweep.sh stop     abort the sweep
#
# Results are tagged $TAG (default speedsweep): rows in results/summary.csv, logs and CSVs in
# results/<date>_i<N>_<tag>/. Afterwards: python3 scripts/sweep_report.py --tag <tag>
# Overrides (environment variables): TAG, LAPS, LOCALIZERS (scan amcl slam, + odom = odometry
# only), SPEEDS, NOISE (odometry noise levels, default 0), REPEATS (runs per speed and noise
# level, default 1), WORKERS (default 3), STAGGER, and MAP / PARAMS (passed to
# eval_localization.sh as -m / -p; need a single localizer).
# Each worker takes ~1-2 CPU cores and ~0.8 GB while driving; 5-6 workers fit the 10-core / 8 GB
# container. Runs are balanced over the workers by expected run time.
#   e.g. TAG=amcl_outline LOCALIZERS=amcl WORKERS=6 ./scripts/run_speed_sweep.sh
#        TAG=odom_noise LOCALIZERS="scan amcl slam odom" SPEEDS=1.0 NOISE="0 1 2 4 8" REPEATS=3 WORKERS=5 ./scripts/run_speed_sweep.sh

SANDBOX=$(cd "$(dirname "$0")/.." && pwd)
RESULTS=$SANDBOX/results
TAG=${TAG:-speedsweep}
LAPS=${LAPS:-3}
LOCALIZERS=${LOCALIZERS:-"scan amcl slam"}
SPEEDS=${SPEEDS:-"0.20 0.25 0.30 0.35 0.40 0.45 0.50 0.55 0.60 0.65 0.70 0.75 0.80 0.85 0.90 0.95 1.0"}
NOISE=${NOISE:-0}
REPEATS=${REPEATS:-1}
WORKERS=${WORKERS:-3}
STAGGER=${STAGGER:-15}  # [s] between worker starts, so fewer localizers start at the same moment
MAP=${MAP:-}            # e.g. maps/victoria_ref_lc (SLAM Toolbox: no extension)
PARAMS=${PARAMS:-}      # e.g. config/localization_no_loop_closure.yaml
PIDS=$RESULTS/$TAG.pids
PLAN=$RESULTS/${TAG}_plan.json

# Prints one line of runs (localizer:speed:noise:run) per worker, in the order the worker drives
# them. Longest run first, each to the worker that would be done earliest (worker i starts
# (i-1)*STAGGER late). Run time ~ LAPS * 37 m / speed + ~20 s start/stop. Among runs of equal
# length the localizers take turns, so the workers rarely run the same (heavy) localizer at once.
split_jobs() {
    python3 - "$WORKERS" "$STAGGER" "$LAPS" "$REPEATS" "$LOCALIZERS" "$SPEEDS" "$NOISE" <<'EOF'
import sys
workers, stagger, laps, repeats = int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), int(sys.argv[4])
locs, speeds, noise = (a.split() for a in sys.argv[5:8])
cost = lambda s: laps * 37.0 / float(s) + 20.0
jobs = [(l, s, n, r) for s in speeds for n in noise for r in range(1, repeats + 1) for l in locs]
jobs.sort(key=lambda j: -cost(j[1]))  # stable: equal costs keep the localizers interleaved
workers = min(workers, len(jobs))
load, plan = [i * stagger for i in range(workers)], [[] for _ in range(workers)]
for j in jobs:
    i = min(range(workers), key=lambda i: load[i])
    load[i] += cost(j[1])
    plan[i].append(':'.join(map(str, j)))
for p in plan:
    print(' '.join(p))
EOF
}

# PIDs of this sweep's workers that are still running
running() {
    [ -f "$PIDS" ] || return 0
    for p in $(cat "$PIDS"); do kill -0 "$p" 2>/dev/null && echo "$p"; done
}

watch() {
    exec python3 "$SANDBOX/scripts/sweep_progress.py" --tag "$TAG"
}

stop() {
    for p in $(running); do kill -9 "$p" 2>/dev/null; done
    # Then everything the workers started. Like eval_localization.sh's cleanup, this goes by the
    # Gazebo partition in each process's environment (eval<N> of this sweep's workers, from the
    # plan), so the user's own sim and other benchmarks are never touched.
    local ids
    ids=$(python3 -c "import json, sys; print('|'.join(json.load(open(sys.argv[1]))['workers']))" "$PLAN" 2>/dev/null)
    [ -n "$ids" ] && for p in $(pgrep -u "$(id -u)"); do
        [ "$p" = $$ ] && continue
        tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -qxE "IGN_PARTITION=eval($ids)" && kill -9 "$p" 2>/dev/null
    done
    echo "Sweep '$TAG' stopped. Finished runs are kept in results/."
}

start() {
    if [ -n "$(running)" ]; then
        echo "Sweep '$TAG' is already running. Watch it: $0 watch   Abort it: $0 stop"; exit 1
    fi
    if compgen -G "$RESULTS/*_$TAG" > /dev/null; then
        echo "Results tagged '$TAG' already exist (results/*_$TAG); a second sweep would mix with them."
        echo "Start it under a new name instead: TAG=<new name> $0"; exit 1
    fi
    [[ "$WORKERS" =~ ^([1-9]|1[0-9]|20)$ ]] || { echo "WORKERS must be 1-20 (eval_localization.sh -i)"; exit 1; }
    [[ "$REPEATS" =~ ^[1-9][0-9]*$ ]] || { echo "REPEATS must be a whole number >= 1"; exit 1; }
    # MAP / PARAMS: absolute paths (workers start elsewhere), checked here, since a worker that
    # fails on them would only say so in its own log
    local extra=""
    if [ -n "$MAP$PARAMS" ] && { [ "$(wc -w <<< "$LOCALIZERS")" -ne 1 ] || [ "$LOCALIZERS" = odom ]; }; then
        echo "MAP and PARAMS need a single localizer (LOCALIZERS=slam, amcl or scan)"; exit 1
    fi
    if [ -n "$MAP" ]; then
        MAP=$(realpath -m "$MAP")
        [ -e "$MAP" ] || [ -e "$MAP.posegraph" ] || { echo "map not found: $MAP"; exit 1; }
        extra="$extra -m '$MAP'"
    fi
    if [ -n "$PARAMS" ]; then
        PARAMS=$(realpath -m "$PARAMS")
        [ -f "$PARAMS" ] || { echo "params not found: $PARAMS"; exit 1; }
        extra="$extra -p '$PARAMS'"
    fi
    mkdir -p "$RESULTS"

    # what the progress view needs to know: workers, their runs and start delays
    local delay=0 workers="" i=0 jobs
    : > "$PIDS"
    while read -r jobs; do
        i=$((i + 1))
        setsid nohup bash -c "sleep $delay; exec '$SANDBOX/scripts/eval_localization.sh' -j '$jobs' \
            -t '$TAG' -l $LAPS -i $i$extra" > "$RESULTS/${TAG}_worker$i.log" 2>&1 < /dev/null &
        echo $! >> "$PIDS"
        workers="$workers $i/$delay/${jobs// /,}"
        delay=$((delay + STAGGER))
    done < <(split_jobs)
    python3 - "$PLAN" "$TAG" "$LAPS" "$LOCALIZERS" $workers <<'EOF'
import json, sys, time
plan, tag, laps, localizers, *workers = sys.argv[1:]
json.dump({
    'tag': tag, 'laps': float(laps), 'localizers': localizers.split(), 'started': time.time(),
    'workers': {w.split('/')[0]: {'delay_s': int(w.split('/')[1]), 'jobs': w.split('/')[2].split(',')}
                for w in workers},
}, open(plan, 'w'), indent=1)
EOF
    echo "Started $(wc -l < "$PIDS") workers for sweep '$TAG' (they keep running if you close this view)."
    watch
}

case "${1:-start}" in
    start) start ;;
    watch) watch ;;
    stop) stop ;;
    *) sed -n '2,20p' "$0"; exit 1 ;;
esac
