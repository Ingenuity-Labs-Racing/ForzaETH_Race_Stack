#! /bin/bash
# Standard localization benchmark for the sandbox. For each speed it starts a fresh headless
# sim, runs SLAM Toolbox localization on the reference map, drives autopilot laps and
# measures the error against Gazebo's ground truth.
# Runs in its own Gazebo partition / ROS domain, so it doesn't interfere with an open session.
#
#   usage: eval_localization.sh [-t tag] [-s "0.8 1.5"] [-r repeats] [-l laps] [-p params.yaml] [-m map] [-v] [-b]
#     -t  name for this run (default: "run")
#     -s  speeds in m/s (default: "0.8 1.5")
#     -r  runs per speed, each in a fresh sim (default: 1). Results vary run to run, use >= 3 to compare settings
#     -l  laps per run, driven back to back (default: 1)
#     -p  SLAM Toolbox params (default: config/localization.yaml)
#     -m  serialized map without extension (default: maps/victoria_ref)
#     -v  watch it in the Foxglove app (ws://localhost:8765, layout foxglove/sandbox_layout.json).
#         Close your own sim first: only one Foxglove bridge can use the port.
#     -b  record every run as a bag (MCAP, open it in Foxglove). Needs ros-humble-rosbag2-storage-mcap
#
# Output: results/<date>_<tag>/ (logs + per-sample CSV per run), and one row per run
# appended to results/summary.csv. Exit code is non-zero if any lap failed.

SANDBOX=~/ws/src/race_stack/utilities/slam_toolbox_sandbox
TAG=run
SPEEDS="0.8 1.5"
REPEATS=1
LAPS=1
BAG=false
FOXGLOVE=false
PARAMS=default
MAP=$SANDBOX/maps/victoria_ref
while getopts "t:s:r:l:p:m:vbh" opt; do
    case $opt in
        t) TAG=$OPTARG ;;
        s) SPEEDS=$OPTARG ;;
        r) REPEATS=$OPTARG ;;
        l) LAPS=$OPTARG ;;
        v) FOXGLOVE=true ;;
        b) BAG=true ;;
        p) PARAMS=$(realpath "$OPTARG") ;;
        m) MAP=$(realpath -m "$OPTARG") ;;
        *) sed -n '2,19p' "$0"; exit 1 ;;
    esac
done

export IGN_PARTITION=eval ROS_DOMAIN_ID=77
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

[ -f "$MAP.posegraph" ] || { echo "map not found: $MAP.posegraph"; exit 1; }
PARAMS_ARG=""; [ "$PARAMS" != "default" ] && PARAMS_ARG="params:=$PARAMS"
OUT=$SANDBOX/results/$(date +%Y-%m-%d_%H%M%S)_$TAG
SUMMARY=$SANDBOX/results/summary.csv
mkdir -p "$OUT"
[ -f "$SUMMARY" ] || echo "date,tag,commit,params,map,speed,run,laps,lap_done,samples,pos_rmse,pos_p95,pos_max,yaw_rmse_deg,yaw_max_deg" > "$SUMMARY"
# safe.directory: inside the container the repo belongs to the host user
GIT="git -c safe.directory=* -C $SANDBOX"
COMMIT=$($GIT rev-parse --short HEAD)$($GIT diff --quiet HEAD -- . || echo "+dirty")

# Kills only processes started in the eval partition (pkill -f could hit the user's session).
cleanup() {
    for p in $(pgrep -f "ign gazebo|parameter_bridge|static_transform|foxglove_bridge|ros2 launch|autodrive|slam_toolbox_node|pose_error|ros2 bag"); do
        tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -q IGN_PARTITION=eval && kill -9 $p 2>/dev/null
    done
    sleep 1
}
trap cleanup EXIT

# wait_for <timeout s> <command>: polls the command every second
wait_for() {
    local end=$((SECONDS + $1)); shift
    until "$@" > /dev/null 2>&1; do
        [ $SECONDS -ge $end ] && return 1
        sleep 1
    done
}

# autodrive declares speed/laps as doubles: "3" would be rejected as an integer
float() { python3 -c "print(float($1))"; }

FAILED=0
cleanup
for SPEED in $SPEEDS; do for RUN in $(seq $REPEATS); do
    LOG=$OUT/speed_${SPEED}_run$RUN
    echo "[$TAG] speed $SPEED m/s, run $RUN/$REPEATS ..."

    ros2 launch slam_toolbox_sandbox sim.launch.py foxglove:=$FOXGLOVE > $LOG.sim.log 2>&1 < /dev/null &
    disown  # no "Killed" messages when cleanup stops it
    if ! wait_for 90 timeout 5 ros2 topic echo --once /ground_truth; then
        echo "  FAIL: sim did not start (see $LOG.sim.log)"; FAILED=1; cleanup; continue
    fi

    ros2 launch slam_toolbox_sandbox localization.launch.py map:=$MAP $PARAMS_ARG csv:=$LOG.csv \
        > $LOG.loc.log 2>&1 < /dev/null &
    LOC_PID=$!
    disown
    if ! wait_for 60 grep -q "position: now" $LOG.loc.log; then
        echo "  FAIL: localization did not start (see $LOG.loc.log)"; FAILED=1; cleanup; continue
    fi

    if [ "$BAG" = true ]; then
        ros2 bag record -a -s mcap -o $LOG.bag > $LOG.bag.log 2>&1 < /dev/null &
        BAG_PID=$!
        sleep 2
    fi

    # generous timeout: the Victoria lap is ~80 m and the sim may run slower than real time
    LAP_TIMEOUT=$(python3 -c "print(int($LAPS * 240 / $SPEED + 60))")
    timeout $LAP_TIMEOUT ros2 run slam_toolbox_sandbox autodrive --ros-args -p speed:=$(float $SPEED) -p laps:=$(float $LAPS) \
        > $LOG.autodrive.log 2>&1
    LAP_DONE=false; grep -q "Done:" $LOG.autodrive.log && LAP_DONE=true

    if [ "$BAG" = true ]; then
        kill -INT $BAG_PID; wait_for 15 bash -c "! kill -0 $BAG_PID"
        echo "  bag: $LOG.bag"
    fi
    kill -INT $LOC_PID; wait_for 10 bash -c "! kill -0 $LOC_PID"
    cleanup

    # metrics from the per-sample CSV
    STATS=$(python3 - "$LOG.csv" <<'EOF'
import csv, math, sys
rows = list(csv.DictReader(open(sys.argv[1])))
if not rows:
    print('0,,,,,'); sys.exit()
pos = sorted(float(r['pos_err']) for r in rows)
yaw = [math.degrees(float(r['yaw_err'])) for r in rows]
rms = lambda v: math.sqrt(sum(e * e for e in v) / len(v))
print(f'{len(pos)},{rms(pos):.3f},{pos[int(0.95 * (len(pos) - 1))]:.3f},{pos[-1]:.3f},{rms(yaw):.2f},{max(yaw):.2f}')
EOF
)
    echo "$(date +%F),$TAG,$COMMIT,$(basename $PARAMS),$(basename $MAP),$SPEED,$RUN,$LAPS,$LAP_DONE,$STATS" >> "$SUMMARY"
    IFS=, read N RMSE P95 MAX YRMSE YMAX <<< "$STATS"
    echo "  lap done: $LAP_DONE | samples $N | position RMSE $RMSE m, p95 $P95 m, max $MAX m | yaw RMSE $YRMSE deg, max $YMAX deg"
    [ "$LAP_DONE" = true ] && [ "$N" -gt 0 ] || FAILED=1
done; done

echo "Logs: $OUT"
echo "All runs: $SUMMARY"
exit $FAILED
