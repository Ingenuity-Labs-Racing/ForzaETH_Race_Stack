"""
Stops a launch file from starting a second copy of the sim or of a localizer.

Two sims in the same Gazebo partition and ROS domain mix their topics: /scan, /tf and
/ground_truth then carry two cars, autodrive steers on a pose that jumps between them, and the
localizers and error plots become meaningless. Nothing crashes, so it's easy to miss.

Only processes with the same IGN_PARTITION and ROS_DOMAIN_ID as this launch count, so isolated
benchmark runs (scripts/eval_localization.sh) and the user's own session never block each other.
"""
import os
import time

SIM = ['ign gazebo', 'gz sim']
LOCALIZERS = [
    'scan_match_localizer', 'amcl', 'localization_slam_toolbox_node',
    'async_slam_toolbox_node', 'sync_slam_toolbox_node', 'pose_error',
]


def _read(path):
    try:
        with open(path, 'rb') as f:
            return f.read()
    except OSError:
        return None


def _env(pid):
    raw = _read(f'/proc/{pid}/environ')
    if raw is None:  # not ours / gone: can't tell, so don't count it
        return None
    env = dict(kv.split('=', 1) for kv in raw.decode(errors='replace').split('\0') if '=' in kv)
    return env.get('IGN_PARTITION', ''), env.get('ROS_DOMAIN_ID', '0') or '0'


def _argv(pid):
    raw = _read(f'/proc/{pid}/cmdline')
    return [a for a in raw.decode(errors='replace').split('\0') if a] if raw else []


def _ppid(pid):
    stat = _read(f'/proc/{pid}/stat')
    # the process name (field 2) may contain spaces: parse after the closing ')'
    return int(stat.decode().rsplit(')', 1)[1].split()[1]) if stat else 0


def _age(pid):
    stat, uptime = _read(f'/proc/{pid}/stat'), _read('/proc/uptime')
    if not stat or not uptime:
        return '?'
    start = int(stat.decode().rsplit(')', 1)[1].split()[19]) / os.sysconf('SC_CLK_TCK')
    s = int(float(uptime.split()[0]) - start)
    return time.strftime('%H:%M:%S', time.gmtime(s)) if s < 86400 else f'{s // 86400} d'


def _matches(argv, patterns):
    # compare executable names, e.g. "ign gazebo -s ..." or "python3 pose_error --ros-args ...".
    # Split on spaces too: `ign` (a Ruby script) rewrites its cmdline into one space-separated string.
    head = [os.path.basename(a) for a in ' '.join(argv[:3]).split()[:3]]
    for p in patterns:
        words = p.split()
        if any(head[i:i + len(words)] == words for i in range(len(head))):
            return p
    return None


def _launcher(pid):
    """The `ros2 launch` / `ros2 run` process that started pid (the one to Ctrl+C), else pid."""
    p = pid
    while p > 1:
        argv = _argv(p)
        if len(argv) > 2 and os.path.basename(argv[1]) == 'ros2' and argv[2] in ('launch', 'run'):
            return p, ' '.join(argv[2:5])
        p = _ppid(p)
    return pid, None


def running_instances(patterns):
    """[(pid, matched pattern)] of processes in this partition and domain, outside our own tree."""
    mine = (os.environ.get('IGN_PARTITION', ''), os.environ.get('ROS_DOMAIN_ID', '0') or '0')
    me = os.getpid()
    found = []
    for name in os.listdir('/proc'):
        if not name.isdigit() or int(name) == me:
            continue
        pid = int(name)
        hit = _matches(_argv(pid), patterns)
        if not hit or _env(pid) != mine:
            continue
        p = pid
        while p > 1 and p != me:
            p = _ppid(p)
        if p != me:
            found.append((pid, hit))
    return found


def fail_if_running(what, patterns):
    """Raise RuntimeError (aborts the launch) if a matching process is already running."""
    found = running_instances(patterns)
    if not found:
        return
    part = os.environ.get('IGN_PARTITION') or '(default)'
    domain = os.environ.get('ROS_DOMAIN_ID', '0') or '0'
    lines, launchers = [], set()
    for pid, hit in found:
        lpid, lcmd = _launcher(pid)
        launchers.add(lpid)
        lines.append(f'  pid {pid} ({hit}, running {_age(pid)})'
                     + (f', started by pid {lpid}: {lcmd}' if lcmd else ''))
    raise RuntimeError(
        f'A {what} is already running in Gazebo partition {part}, ROS domain {domain}:\n'
        + '\n'.join(lines)
        + '\nTwo copies would mix their topics and TF. Stop the old one first: Ctrl+C in its '
        f'terminal, or\n  kill -INT {" ".join(map(str, sorted(launchers)))}\n'
        'To run a second copy on purpose, give it its own IGN_PARTITION and ROS_DOMAIN_ID.')
