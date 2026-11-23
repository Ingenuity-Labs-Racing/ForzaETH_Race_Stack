#! /bin/bash
# Standard localization benchmark for the sandbox. For each localizer, speed and odometry noise
# level it starts a fresh headless sim, starts the localizer, drives autopilot laps and measures
# the error against Gazebo's ground truth.
#   slam: SLAM Toolbox on its own recorded map (maps/victoria_ref_lc), loop closure off
#   amcl: Nav2 AMCL on the fixed ground-truth map's walls (maps/victoria_outline)
#   scan: scan_match_localizer, scan-to-map matching on the same fixed map
#   odom: no localizer, the odometry alone: how far it drifts (about 0 without -n). The reference
#         for the odometry noise test
# Runs in its own Gazebo partition / ROS domain, so it doesn't interfere with an open session.
# Several can run at once with different -i (each takes ~1 CPU core and ~1 GB; don't combine with -v).
#
#   usage: eval_localization.sh [-L "slam amcl"] [-t tag] [-s "0.8 1.5"] [-n "0 2"] [-r repeats] [-l laps] [-p params.yaml] [-m map] [-v] [-b]
#     -L  localizers to test, each under the same conditions (default: "slam")
#     -t  name for this run (default: "run")
#     -s  speeds in m/s (default: "0.8 1.5")
#     -n  odometry noise levels (default: 0 = perfect odometry). 1 is roughly a real car, 2 doubles
#         every error term, ... (slam_toolbox_sandbox/odom_noise.py). Run r uses noise seed r, so
#         every localizer gets the same corrupted odometry in its run r
#     -r  runs per speed and noise level, each in a fresh sim (default: 1). Results vary run to run,
#         use >= 3 to compare settings
#     -j  explicit runs "localizer:speed:noise:run ..." instead of -L/-s/-n/-r (run_speed_sweep.sh)
#     -l  laps per run, driven back to back (default: 1)
#     -p  localizer params (default: config/localization_no_loop_closure.yaml, amcl.yaml or scan_match.yaml)
#     -m  map (default: maps/victoria_ref_lc for slam - serialized, no extension;
#         maps/victoria_outline.yaml for amcl and scan). -p and -m need a single localizer (not odom)
#     -v  watch it in the Foxglove app (ws://localhost:8765, layout foxglove/sandbox_layout.json).
#         Close your own sim first: only one Foxglove bridge can use the port.
#     -b  record every run as a bag (MCAP, open it in Foxglove). Needs ros-humble-rosbag2-storage-mcap
#     -i  instance number 1-20 for parallel benchmarks: own Gazebo partition and ROS domain
#
# Output: results/<date>_<tag>/ (logs + per-sample CSV per run, named <localizer>_speed_<s>_run<r>,
# with _noise<n> before _run when the noise level isn't 0), and one row per run appended to
# results/summary.csv. Exit code is non-zero if any lap failed.

SANDBOX=~/ws/src/race_stack/utilities/slam_toolbox_sandbox
LOCALIZERS=slam
TAG=run
SPEEDS="0.8 1.5"
NOISE=0
REPEATS=1
JOBS=""
LAPS=1
BAG=false
FOXGLOVE=false
PARAMS=default
MAP=default
INSTANCE=""
while getopts "L:t:s:n:r:j:l:p:m:i:vbh" opt; do
    case $opt in
        L) LOCALIZERS=$OPTARG ;;
        t) TAG=$OPTARG ;;
        s) SPEEDS=$OPTARG ;;
        n) NOISE=$OPTARG ;;
        r) REPEATS=$OPTARG ;;
        j) JOBS=$OPTARG ;;
        l) LAPS=$OPTARG ;;
        v) FOXGLOVE=true ;;
        b) BAG=true ;;
        i) INSTANCE=$OPTARG ;;
        p) PARAMS=$(realpath "$OPTARG") ;;
        m) MAP=$(realpath -m "$OPTARG") ;;
        *) sed -n '2,35p' "$0"; exit 1 ;;
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

# the runs, as localizer:speed:noise:run
if [ -z "$JOBS" ]; then
    for L in $LOCALIZERS; do for S in $SPEEDS; do for N in $NOISE; do for R in $(seq $REPEATS); do
        JOBS="$JOBS $L:$S:$N:$R"
    done; done; done; done
fi
NUM='^[0-9]+([.][0-9]+)?$'
for J in $JOBS; do
    IFS=: read L S N R <<< "$J"
    [[ "$L" =~ ^(slam|amcl|scan|odom)$ ]] || { echo "unknown localizer: $L (use slam, amcl, scan or odom)"; exit 1; }
    [[ "$S" =~ $NUM && "$N" =~ $NUM && "$R" =~ ^[1-9][0-9]*$ ]] || { echo "bad run '$J' (localizer:speed:noise:run)"; exit 1; }
done
USED=$(for J in $JOBS; do echo "${J%%:*}"; done | sort -u)
if [ "$PARAMS" != default ] || [ "$MAP" != default ]; then
    [ $(wc -w <<< "$USED") -eq 1 ] && [ "$USED" != odom ] || {
        echo "-p and -m need a single localizer (-L slam, -L amcl or -L scan)"; exit 1; }
fi

# Sets LAUNCH, LAUNCH_ARGS, MAP_NAME and PARAMS_NAME (as recorded in summary.csv) for localizer $1
setup_localizer() {
    case $1 in
        slam) LAUNCH=localization.launch.py; L_MAP=$SANDBOX/maps/victoria_ref_lc
              L_PARAMS=$SANDBOX/config/localization_no_loop_closure.yaml ;;
        amcl) LAUNCH=amcl.launch.py; L_MAP=$SANDBOX/maps/victoria_outline.yaml; L_PARAMS=$SANDBOX/config/amcl.yaml ;;
        scan) LAUNCH=scan_match.launch.py; L_MAP=$SANDBOX/maps/victoria_outline.yaml
              L_PARAMS=$SANDBOX/config/scan_match.yaml ;;
        odom) LAUNCH=odom_only.launch.py; LAUNCH_ARGS=""; MAP_NAME=none; PARAMS_NAME=none; return 0 ;;
    esac
    [ "$MAP" != default ] && L_MAP=$MAP
    [ "$PARAMS" != default ] && L_PARAMS=$PARAMS
    if [ "$1" = slam ]; then
        [ -f "$L_MAP.posegraph" ] || { echo "map not found: $L_MAP.posegraph"; return 1; }
    else
        [ -f "$L_MAP" ] || { echo "map not found: $L_MAP"; return 1; }
    fi
    LAUNCH_ARGS="map:=$L_MAP params:=$L_PARAMS"
    MAP_NAME=$(basename "$L_MAP" .yaml)
    PARAMS_NAME=$(basename "$L_PARAMS")
}
for L in $USED; do setup_localizer $L || exit 1; done

OUT=$SANDBOX/results/$(date +%Y-%m-%d_%H%M%S)${INSTANCE:+_i$INSTANCE}_$TAG
SUMMARY=$SANDBOX/results/summary.csv
HEADER="date,tag,localizer,commit,params,map,speed,run,laps,lap_done,samples,pos_rmse,pos_p95,pos_max,yaw_rmse_deg,yaw_max_deg,run_dir,odom_noise"
mkdir -p "$OUT"
echo "Output: $OUT"  # read by sweep_progress.py
if [ ! -f "$SUMMARY" ]; then
    echo "$HEADER" > "$SUMMARY"
elif [ "$(head -1 "$SUMMARY"),odom_noise" = "$HEADER" ]; then
    sed -i '1s/$/,odom_noise/' "$SUMMARY"  # older file without the noise column: its rows are level 0
fi
# safe.directory: inside the container the repo belongs to the host user
GIT="git -c safe.directory=* -C $SANDBOX"
COMMIT=$($GIT rev-parse --short HEAD)$($GIT diff --quiet HEAD -- . || echo "+dirty")

# Kills only processes started in this partition (pkill -f could hit the user's session or
# another parallel benchmark).
cleanup() {
    for p in $(pgrep -f "ign gazebo|parameter_bridge|static_transform|foxglove_bridge|ros2 launch|autodrive|car_marker|slam_toolbox_node|nav2_map_server/map_server|nav2_amcl/amcl|lifecycle_manager|scan_match_localizer|pose_error|odom_noise|ros2 bag"); do
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

# Starts the sim and the localizer of one run (uses LOG, LAUNCH, LAUNCH_ARGS, NOISE_LEVEL, RUN). On
# failure it cleans up, sets START_ERR and returns non-zero, so the run can be retried: in parallel
# benchmarks AMCL sometimes never publishes map->odom.
start_run() {
    START_ERR=""
    ros2 launch slam_toolbox_sandbox sim.launch.py foxglove:=$FOXGLOVE odom_noise:=$NOISE_LEVEL odom_seed:=$RUN \
        > $LOG.sim.log 2>&1 < /dev/null &
    disown  # no "Killed" messages when cleanup stops it
    # --no-daemon: a ros2 daemon whose ROS context has died stays "running" and fails every CLI call
    # in its domain; that made worker 1 report "sim did not start" on every attempt
    if ! wait_for 90 timeout 5 ros2 topic echo --once --no-daemon /ground_truth; then
        START_ERR="sim did not start (see $LOG.sim.log)"; cleanup; return 1
    fi
    ros2 launch slam_toolbox_sandbox $LAUNCH $LAUNCH_ARGS csv:=$LOG.csv > $LOG.loc.log 2>&1 < /dev/null &
    LOC_PID=$!
    disown
    if ! wait_for 60 grep -q "position: now" $LOG.loc.log; then
        START_ERR="localization did not start (see $LOG.loc.log)"; cleanup; return 1
    fi
}

FAILED=0
cleanup
for J in $JOBS; do
    IFS=: read LOCALIZER SPEED NOISE_LEVEL RUN <<< "$J"
    setup_localizer $LOCALIZER
    NOISE_PART=""; [ "$(awk "BEGIN { print ($NOISE_LEVEL > 0) }")" = 1 ] && NOISE_PART=_noise$NOISE_LEVEL
    LOG=$OUT/${LOCALIZER}_speed_${SPEED}${NOISE_PART}_run$RUN
    echo "[$TAG] $LOCALIZER, speed $SPEED m/s, noise $NOISE_LEVEL, run $RUN ..."  # parsed by sweep_progress.py

    # up to 3 attempts; the logs of a failed attempt are kept as *.failed<N>
    for TRY in 1 2 3; do
        start_run && break
        [ $TRY = 3 ] && break
        echo "  RETRY $TRY: $START_ERR"
        for f in $LOG.sim.log $LOG.loc.log $LOG.csv; do [ -f $f ] && mv $f $f.failed$TRY; done
    done
    if [ -n "$START_ERR" ]; then
        echo "  FAIL: $START_ERR"; FAILED=1
        # a row without metrics, so failed runs show up in the results too
        echo "$(date +%F),$TAG,$LOCALIZER,$COMMIT,$PARAMS_NAME,$MAP_NAME,$SPEED,$RUN,$LAPS,false,0,,,,,,$(basename $OUT),$NOISE_LEVEL" >> "$SUMMARY"
        continue
    fi

    if [ "$BAG" = true ]; then
        ros2 bag record -a -s mcap -o $LOG.bag > $LOG.bag.log 2>&1 < /dev/null &
        BAG_PID=$!
        sleep 2
    fi

    # generous timeout: the Victoria lap is ~37 m and the sim may run slower than real time.
    # Sim time keeps autodrive's 3 s stuck detector honest when the sim is slow. Line-buffered
    # logging so the "X.XX laps" progress lines reach the log right away (sweep_progress.py).
    LAP_TIMEOUT=$(python3 -c "print(int($LAPS * 240 / $SPEED + 60))")
    AUTODRIVE_START=$SECONDS
    RCUTILS_LOGGING_BUFFERED_STREAM=1 timeout $LAP_TIMEOUT ros2 run slam_toolbox_sandbox autodrive --ros-args \
        -p speed:=$(float $SPEED) -p laps:=$(float $LAPS) -p use_sim_time:=true > $LOG.autodrive.log 2>&1
    echo "wall_s=$((SECONDS - AUTODRIVE_START))" >> $LOG.autodrive.log  # for the real-time factor
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
    echo "$(date +%F),$TAG,$LOCALIZER,$COMMIT,$PARAMS_NAME,$MAP_NAME,$SPEED,$RUN,$LAPS,$LAP_DONE,$STATS,$(basename $OUT),$NOISE_LEVEL" >> "$SUMMARY"
    IFS=, read N RMSE P95 MAX YRMSE YMAX <<< "$STATS"
    echo "  lap done: $LAP_DONE | samples $N | position RMSE $RMSE m, p95 $P95 m, max $MAX m | yaw RMSE $YRMSE deg, max $YMAX deg"
    [ "$LAP_DONE" = true ] && [ "$N" -gt 0 ] || FAILED=1
done

echo "Logs: $OUT"
echo "All runs: $SUMMARY"
exit $FAILED
