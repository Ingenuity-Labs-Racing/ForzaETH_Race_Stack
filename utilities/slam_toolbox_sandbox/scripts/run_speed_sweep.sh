#! /bin/bash
# Speed sweep: every localizer at 17 speeds (0.20-1.00 m/s), one 3-lap run per speed, split over
# 3 parallel eval_localization.sh workers (each in its own Gazebo partition and ROS domain, -i 1-3).
# The workers run detached: Ctrl+C or closing the terminal only stops the progress view.
#
#   ./scripts/run_speed_sweep.sh          start the sweep and show its progress
#   ./scripts/run_speed_sweep.sh watch    show the progress again (from any container terminal)
#   ./scripts/run_speed_sweep.sh stop     abort the sweep
#
# Results are tagged $TAG (default speedsweep): rows in results/summary.csv, logs and CSVs in
# results/<date>_i<N>_<tag>/. Afterwards: python3 scripts/sweep_report.py --tag <tag>
# Overrides, e.g. to test the tooling: TAG, LAPS, LOCALIZERS, SPEEDS_1, SPEEDS_2, SPEEDS_3
# (an empty SPEEDS_<N> leaves that worker out).

SANDBOX=$(cd "$(dirname "$0")/.." && pwd)
RESULTS=$SANDBOX/results
TAG=${TAG:-speedsweep}
LAPS=${LAPS:-3}
LOCALIZERS=${LOCALIZERS:-"scan amcl slam"}
# split so each worker gets about the same driving time (sum of 1/speed)
SPEEDS_1=${SPEEDS_1-"0.20 0.45 0.60 0.85 1.0"}
SPEEDS_2=${SPEEDS_2-"0.25 0.40 0.55 0.75 0.90 0.95"}
SPEEDS_3=${SPEEDS_3-"0.30 0.35 0.50 0.65 0.70 0.80"}
STAGGER=30  # [s] between worker starts, so fewer localizers start at the same moment
PIDS=$RESULTS/$TAG.pids
PLAN=$RESULTS/${TAG}_plan.json

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
    # Gazebo partition in each process's environment, so the user's own sim is never touched.
    for p in $(pgrep -u "$(id -u)"); do
        [ "$p" = $$ ] && continue
        tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -qx "IGN_PARTITION=eval[123]" && kill -9 "$p" 2>/dev/null
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
    mkdir -p "$RESULTS"

    # what the progress view needs to know: workers, their speeds and start delays
    local delay=0 workers=""
    : > "$PIDS"
    for i in 1 2 3; do
        local speeds_var=SPEEDS_$i
        local speeds=${!speeds_var}
        [ -n "$speeds" ] || continue
        setsid nohup bash -c "sleep $delay; exec '$SANDBOX/scripts/eval_localization.sh' -L '$LOCALIZERS' \
            -t '$TAG' -s '$speeds' -l $LAPS -r 1 -i $i" > "$RESULTS/${TAG}_worker$i.log" 2>&1 < /dev/null &
        echo $! >> "$PIDS"
        workers="$workers $i:$delay:${speeds// /,}"
        delay=$((delay + STAGGER))
    done
    python3 - "$PLAN" "$TAG" "$LAPS" "$LOCALIZERS" $workers <<'EOF'
import json, sys, time
plan, tag, laps, localizers, *workers = sys.argv[1:]
json.dump({
    'tag': tag, 'laps': float(laps), 'localizers': localizers.split(), 'started': time.time(),
    'workers': {w.split(':')[0]: {'delay_s': int(w.split(':')[1]), 'speeds': w.split(':')[2].split(',')}
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
    *) sed -n '2,14p' "$0"; exit 1 ;;
esac
