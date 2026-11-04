#! /bin/bash
# Standard localization benchmark for the sandbox. For each localizer and speed it starts a
# fresh headless sim, starts the localizer, drives autopilot laps and measures the error
# against Gazebo's ground truth.
#   slam: SLAM Toolbox on its serialized reference map (maps/victoria_ref)
#   amcl: Nav2 AMCL on the ground-truth map (stack_master/maps/Victoria)
#   scan: scan_match_localizer, scan-to-map matching on the same fixed ground-truth map
# Runs in its own Gazebo partition / ROS domain, so it doesn't interfere with an open session.
# Several can run at once with different -i (each takes ~1 CPU core and ~1 GB; don't combine with -v).
#
#   usage: eval_localization.sh [-L "slam amcl"] [-t tag] [-s "0.8 1.5"] [-r repeats] [-l laps] [-p params.yaml] [-m map] [-v] [-b]
#     -L  localizers to test, each under the same conditions (default: "slam")
#     -t  name for this run (default: "run")
#     -s  speeds in m/s (default: "0.8 1.5")
#     -r  runs per speed, each in a fresh sim (default: 1). Results vary run to run, use >= 3 to compare settings
#     -l  laps per run, driven back to back (default: 1)
#     -p  localizer params (default: config/localization.yaml, amcl.yaml or scan_match.yaml)
#     -m  map (default: maps/victoria_ref for slam - serialized, no extension;
#         stack_master/maps/Victoria/Victoria.yaml for amcl and scan). -p and -m need a single localizer
#     -v  watch it in the Foxglove app (ws://localhost:8765, layout foxglove/sandbox_layout.json).
#         Close your own sim first: only one Foxglove bridge can use the port.
#     -b  record every run as a bag (MCAP, open it in Foxglove). Needs ros-humble-rosbag2-storage-mcap
#     -i  instance number 1-20 for parallel benchmarks: own Gazebo partition and ROS domain
#
# Output: results/<date>_<tag>/ (logs + per-sample CSV per run), and one row per run
# appended to results/summary.csv. Exit code is non-zero if any lap failed.

SANDBOX=~/ws/src/race_stack/utilities/slam_toolbox_sandbox
LOCALIZERS=slam
TAG=run
SPEEDS="0.8 1.5"
REPEATS=1
LAPS=1
BAG=false
FOXGLOVE=false
PARAMS=default
MAP=default
INSTANCE=""
while getopts "L:t:s:r:l:p:m:i:vbh" opt; do
    case $opt in
        L) LOCALIZERS=$OPTARG ;;
        t) TAG=$OPTARG ;;
        s) SPEEDS=$OPTARG ;;
        r) REPEATS=$OPTARG ;;
        l) LAPS=$OPTARG ;;
        v) FOXGLOVE=true ;;
        b) BAG=true ;;
        i) INSTANCE=$OPTARG ;;
        p) PARAMS=$(realpath "$OPTARG") ;;
        m) MAP=$(realpath -m "$OPTARG") ;;
        *) sed -n '2,26p' "$0"; exit 1 ;;
    esac
done

if [ -n "$INSTANCE" ]; then
    [[ "$INSTANCE" =~ ^([1-9]|1[0-9]|20)$ ]] || { echo "-i must be 1-20"; exit 1; }
    export IGN_PARTITION=eval$INSTANCE ROS_DOMAIN_ID=$((77 + INSTANCE))
else
    export IGN_PARTITION=eval ROS_DOMAIN_ID=77
fi
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

for L in $LOCALIZERS; do
    [ "$L" = slam ] || [ "$L" = amcl ] || [ "$L" = scan ] || { echo "unknown localizer: $L (use slam, amcl or scan)"; exit 1; }
done
if [ $(wc -w <<< "$LOCALIZERS") -gt 1 ] && { [ "$PARAMS" != default ] || [ "$MAP" != default ]; }; then
    echo "-p and -m need a single localizer (-L slam, -L amcl or -L scan)"; exit 1
fi
OUT=$SANDBOX/results/$(date +%Y-%m-%d_%H%M%S)${INSTANCE:+_i$INSTANCE}_$TAG
SUMMARY=$SANDBOX/results/summary.csv
mkdir -p "$OUT"
[ -f "$SUMMARY" ] || echo "date,tag,localizer,commit,params,map,speed,run,laps,lap_done,samples,pos_rmse,pos_p95,pos_max,yaw_rmse_deg,yaw_max_deg,run_dir" > "$SUMMARY"
# safe.directory: inside the container the repo belongs to the host user
GIT="git -c safe.directory=* -C $SANDBOX"
COMMIT=$($GIT rev-parse --short HEAD)$($GIT diff --quiet HEAD -- . || echo "+dirty")

# Kills only processes started in this partition (pkill -f could hit the user's session or
# another parallel benchmark).
cleanup() {
    for p in $(pgrep -f "ign gazebo|parameter_bridge|static_transform|foxglove_bridge|ros2 launch|autodrive|slam_toolbox_node|nav2_map_server/map_server|nav2_amcl/amcl|lifecycle_manager|scan_match_localizer|pose_error|ros2 bag"); do
        [ $p = $$ ] && continue
        tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -qx "IGN_PARTITION=$IGN_PARTITION" && kill -9 $p 2>/dev/null
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
for LOCALIZER in $LOCALIZERS; do
if [ "$LOCALIZER" = slam ]; then
    LAUNCH=localization.launch.py
    L_MAP=$MAP; [ "$L_MAP" = default ] && L_MAP=$SANDBOX/maps/victoria_ref
    [ -f "$L_MAP.posegraph" ] || { echo "map not found: $L_MAP.posegraph"; exit 1; }
else
    LAUNCH=amcl.launch.py; [ "$LOCALIZER" = scan ] && LAUNCH=scan_match.launch.py
    L_MAP=$MAP; [ "$L_MAP" = default ] && L_MAP=~/ws/src/race_stack/stack_master/maps/Victoria/Victoria.yaml
    [ -f "$L_MAP" ] || { echo "map not found: $L_MAP"; exit 1; }
fi
PARAMS_ARG=""; [ "$PARAMS" != "default" ] && PARAMS_ARG="params:=$PARAMS"
for SPEED in $SPEEDS; do for RUN in $(seq $REPEATS); do
    LOG=$OUT/${LOCALIZER}_speed_${SPEED}_run$RUN
    echo "[$TAG] $LOCALIZER, speed $SPEED m/s, run $RUN/$REPEATS ..."

    ros2 launch slam_toolbox_sandbox sim.launch.py foxglove:=$FOXGLOVE > $LOG.sim.log 2>&1 < /dev/null &
    disown  # no "Killed" messages when cleanup stops it
    if ! wait_for 90 timeout 5 ros2 topic echo --once /ground_truth; then
        echo "  FAIL: sim did not start (see $LOG.sim.log)"; FAILED=1; cleanup; continue
    fi

    ros2 launch slam_toolbox_sandbox $LAUNCH map:=$L_MAP $PARAMS_ARG csv:=$LOG.csv \
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
    echo "$(date +%F),$TAG,$LOCALIZER,$COMMIT,$(basename $PARAMS),$(basename $L_MAP .yaml),$SPEED,$RUN,$LAPS,$LAP_DONE,$STATS,$(basename $OUT)" >> "$SUMMARY"
    IFS=, read N RMSE P95 MAX YRMSE YMAX <<< "$STATS"
    echo "  lap done: $LAP_DONE | samples $N | position RMSE $RMSE m, p95 $P95 m, max $MAX m | yaw RMSE $YRMSE deg, max $YMAX deg"
    [ "$LAP_DONE" = true ] && [ "$N" -gt 0 ] || FAILED=1
done; done
done

echo "Logs: $OUT"
echo "All runs: $SUMMARY"
exit $FAILED
