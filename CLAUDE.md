# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ForzaETH Race Stack is an autonomous racing system for 1/10 scale F1TENTH racecars developed by ETH Zurich's Center for Project Based Learning. The stack supports both time-trials and head-to-head racing scenarios, running on ROS Noetic with both simulation and real hardware deployments.

## Build and Development Commands

### Building the System

```bash
# Build entire workspace
catkin build

# After building, source the environment
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
```

### Docker Deployment

```bash
# Build base image (x86)
docker compose build base_x86

# Build simulator container
export UID=$(id -u)
export GID=$(id -g)
docker compose build sim_x86

# Ensure cache directories exist at ../race_stack_cache/noetic/{build,devel,logs}
```

### Running the Stack

**Simulator with base system:**
```bash
roslaunch stack_master base_system.launch sim:=true map_name:=test_map
```

**Time trials:**
```bash
# In terminal 1: Launch base system
roslaunch stack_master base_system.launch map_name:=<map_name> racecar_version:=<NUCX>

# In terminal 2: Launch time trials
roslaunch stack_master time_trials.launch racecar_version:=<NUCX> ctrl_algo:=<MAP|PP|KMPC|STMPC> LU_table:=<NUCX_map_pacejka>
```

**Head-to-head racing:**
```bash
roslaunch stack_master headtohead.launch racecar_version:=<NUCX> ctrl_algo:=<MAP|PP|KMPC|STMPC> LU_table:=<NUCX_map_pacejka> planner:=<frenet|graph_based|spliner|predictive_spliner>
```

**Mapping (on real car):**
```bash
roslaunch stack_master mapping.launch map_name:=<map_name> racecar_version:=<NUCX>
```

### Testing

```bash
# BayesOpt4ROS unit tests
cd f110_utils/nodes/bayesopt4ros
pytest test/unit/
```

## High-Level Architecture

### System Structure

The race stack is organized into modular ROS packages across several key domains:

**Core System Layers:**
- `stack_master/` - Main interface and launch configurations. All high-level operations start here.
- `base_system/` - Contains f1tenth_system (hardware drivers, VESC interface) and f110-simulator
- `f110_utils/` - Utility nodes and tools (Bayesian optimization, lap analysis, map editing, etc.)

**Perception-Planning-Control Pipeline:**
1. **State Estimation** (`state_estimation/`)
   - Cartographer SLAM or SynPF particle filter for localization
   - EKF sensor fusion from `robot_localization` package
   - Fuses VESC odometry, IMU, and localization data

2. **Perception** (`perception/`)
   - `abd_tracker` - Adversarial Blend Detector for opponent tracking
   - `TinyCenterSpeed` - ML-based perception for cone detection

3. **Planning** (`planner/`)
   - `frenet-planner` - Frenet frame based overtaking
   - `graph_based_planner` - Graph search overtaking
   - `spliner` - Spline-based overtaking
   - `predictive-spliner` - Spatiotemporal overtaking with opponent trajectory prediction
   - `gb_optimizer` - Global trajectory optimization

4. **Control** (`controller/`)
   - `controller_manager.py` - Main control node that coordinates controllers
   - `pp/` - Pure Pursuit controller
   - `map/` - Model and Acceleration-based Pursuit controller
   - `mpc/` - Model Predictive Control (kinematic KMPC and single-track STMPC versions)
   - `ftg/` - Follow the Gap controller

5. **State Machine** (`state_machine/`)
   - Manages racing states (ready, driving, emergency stop, etc.)
   - `state_indicator/` - Visual state feedback

### Configuration Management

**Car-Specific Configurations** (`stack_master/config/<RACECAR_VERSION>/`):
- Each physical car has its own configuration directory (e.g., NUC2, JET1)
- Contains VESC parameters, sensor configs, static transforms, controller tuning
- `DEFAULT/` contains shared baseline parameters

**Map Configurations** (`stack_master/maps/<MAP_NAME>/`):
- Each map has YAML metadata, PGM occupancy grid, speed scaling, and overtaking sectors
- Maps are created via the mapping procedure and stored here

**Global Parameters** (used throughout nodes):
- `/sim` - Running in simulation (false means real car)
- `/measure` - Enable additional measurements/logging
- `/from_bag` - Running from rosbag playback
- `/racecar_version` - Which car configuration to use (NUC2, JET1, etc.)

### Key Data Flow

1. **Localization**: LiDAR scan → SLAM/PF → pose → EKF fusion with VESC odom + IMU → `/car_state/odom`
2. **Planning**: Global raceline + obstacles → overtaking planner → `/local_waypoints`
3. **Control**: `/local_waypoints` + `/car_state/odom` → controller → `/vesc/high_level/ackermann_cmd_mux/input/nav_1`

### Controller Selection

The `ctrl_algo` argument selects the controller:
- `PP` - Pure Pursuit (simplest, good baseline)
- `MAP` - Model and Acceleration-based Pursuit (requires `LU_table` steering lookup)
- `KMPC` - Kinematic MPC (no tire dynamics)
- `STMPC` - Single Track MPC (with Pacejka tire model, most advanced)

### Planner Selection

The `planner` argument selects the overtaking planner:
- `frenet` - Frenet-based planner
- `graph_based` - Graph search-based
- `spliner` - Reactive spline-based
- `predictive_spliner` - Spatiotemporal prediction-based

### Localization Modes

Specify `localization:=<slam|pf>` in base_system.launch:
- `slam` (default) - More accurate, faster, smoother. Works well in structured environments.
- `pf` - Better on slippery floors, poorly reflecting environments, or long featureless straights.

## Development Guidelines

### Code Style

**Python:**
- Add docstrings (use Python Docstring Generator extension for VS Code)
- Use type hints for non-obvious function parameters and returns
- Remove all commented code, unused imports, and variables
- Add TODO comments for code smells/hard-coded values

**ROS Nodes:**
- Use global context parameters (`/sim`, `/measure`, `/from_bag`, `/racecar_version`) instead of hard-coding
- Use ROS logging functions with default level `INFO`
- Reference example: `planner/spliner/src/spliner_node.py`

**Launch Files:**
- Document arguments with `doc` attribute
- Keep formatting consistent using VS Code XML extension

**Package READMEs:**
Must contain:
- Description
- Parameters
- Input/Output Topic Signature

Reference example: `planner/spliner/README.md`

### System Identification

The `system_identification/` package provides tools for on-track system identification:
- `id_controller` - System ID controller
- `id_analyser` - Analysis tools
- `steering_lookup` - Steering lookup table generation
- `on_track_sys_id` - Learning-based on-track system ID

After generating lookup tables, rebuild: `catkin build steering_lookup` and re-source workspace.

### Checklists and Procedures

Detailed operational procedures are in `stack_master/checklists/`:
- `TimeTrials.md` - Time trial setup and execution
- `HeadToHead.md` - Head-to-head racing setup
- `Mapping.md` - Creating new maps

### Connecting to the Car (Pit Setup)

```bash
cd <race_stack_folder>/f110_utils/scripts/pit_starter
source pit_starter.sh <YOUR_ZEROTIER_IP> <NUC3|NUC4> rviz
```

### Installation Locations

**Dependencies:**
- Ubuntu packages: `.devcontainer/.install_utils/linux_req_car.txt` (car), `linux_req_sim.txt` (sim)
- Python packages: `.devcontainer/.install_utils/requirements.txt`
- Custom packages: `f110_utils/libs/ccma`, `planner/graph_based_planner/src/GraphBasedPlanner`

**Modified Submodules:**
- Cartographer SLAM (install via `.devcontainer/.install_utils/cartographer_setup.sh`)
- SynPF particle filter (install via `.devcontainer/.install_utils/pf_setup.sh`)

## Important Notes

- The repository has a ROS2 version in other branches
- Cache directories for Docker are at `../race_stack_cache/noetic/{build,devel,logs}` (outside repo)
- After mapping, sector tuning triggers automatic package rebuilds - re-source workspace afterwards
- Main branch is `main`
- See INSTALLATION.md for detailed setup, TROUBLESHOOTING.md for common issues
- The stack is published in Journal of Field Robotics with additional publications linked in README.md
