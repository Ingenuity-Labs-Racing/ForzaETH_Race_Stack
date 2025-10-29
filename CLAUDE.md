# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ForzaETH Race Stack is an autonomous racing system for F1TENTH vehicles developed at ETH Zurich's D-ITET Center for Project Based Learning. The stack supports both time-trials and head-to-head racing in simulation and on real hardware.

This is a ROS Noetic catkin workspace with both Python and C++ packages.

## Build Commands

### Initial Setup (Docker - Recommended)
```bash
# Build base docker image
docker compose build base_x86

# Build simulator container
export UID=$(id -u)
export GID=$(id -g)
docker compose build sim_x86

# Create cache directories
mkdir -p ../race_stack_cache/noetic/build ../race_stack_cache/noetic/devel ../race_stack_cache/noetic/logs
```

### Initial Setup (Native)
```bash
# Install dependencies (car)
xargs sudo apt-get install -y < ./.install_utils/linux_req_car.txt
pip install -r ./.devcontainer/.install_utils/requirements.txt
pip install ~/catkin_ws/src/race_stack/f110_utils/libs/ccma
pip install -e ~/catkin_ws/src/race_stack/planner/graph_based_planner/src/GraphBasedPlanner

# Install cartographer and particle filter
chmod +x ./.devcontainer/.install_utils/cartographer_setup.sh
sudo ./.devcontainer/.install_utils/cartographer_setup.sh
chmod +x ./.devcontainer/.install_utils/pf_setup.sh
sudo ./.devcontainer/.install_utils/pf_setup.sh
```

### Build
```bash
catkin build
```

### Re-source after build
```bash
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
```

## Running the System

### Simulation Test
```bash
# Terminal 1: Launch base system with simulator
roslaunch stack_master base_system.launch sim:=true map_name:=test_map

# Terminal 2: Launch time trials
roslaunch stack_master time_trials.launch racecar_version:=NUC2
```

### Real Car - Time Trials
```bash
# Terminal 1: Launch roscore (optional but recommended)
roscore

# Terminal 2: Launch base system
roslaunch stack_master base_system.launch map_name:=<map_name> racecar_version:=<NUC2|JET1>

# Terminal 3: Launch time trials
roslaunch stack_master time_trials.launch ctrl_algo:=<PP|MAP|STMPC|KMPC> LU_table:=<LU_table_name>
# Note: LU_table only required for MAP controller
```

### Real Car - Head-to-Head Racing
```bash
roslaunch stack_master headtohead.launch ctrl_algo:=<PP|MAP|STMPC|KMPC> LU_table:=<table> planner:=<frenet|graph_based|spliner|predictive_spliner>
```

### Mapping
```bash
roslaunch stack_master mapping.launch map_name:=<map_name> racecar_version:=<NUCX>
# After completing a lap, use GUI to select sectors and overtaking sectors
# Then re-source the workspace
```

### Localization Options
```bash
# Choose between SLAM (default) or particle filter
roslaunch stack_master base_system.launch map_name:=<map> racecar_version:=<NUC> localization:=<slam|pf>
```

## Architecture

The race stack follows a modular architecture with clear separation of concerns:

### Core Modules

**base_system**: Hardware interfaces and simulator
- `f1tenth_system`: Real car interfaces (VESC, sensors, joystick)
- `f110-simulator`: Modified F1TENTH simulator

**state_estimation**: Localization and velocity estimation
- Supports both Cartographer SLAM (default) and SynPF particle filter
- EKF-based sensor fusion using `robot_localization` package
- Frenet frame conversion for planning/control

**perception**: Obstacle detection and tracking
- `abd_tracker`: Adaptive breakpoint detector with Kalman filtering
- `TinyCenterSpeed`: ML-based opponent detection

**planner**: Global and local trajectory planning
- Global: `gb_optimizer` (minimum curvature trajectory generation)
- Local: `spliner`, `frenet-planner`, `graph_based_planner`, `predictive-spliner`
- Planners handle overtaking and collision avoidance

**controller**: Low-level trajectory tracking
- Pure Pursuit (PP)
- Model and Acceleration-based Pursuit (MAP) - requires lookup table
- Kinematic MPC (KMPC)
- Single Track MPC (STMPC) - includes tire dynamics

**state_machine**: Behavioral state management
- Coordinates time trials vs head-to-head behaviors
- Manages state transitions (GBTRACK, OVERTAKE, FTG)
- Generates local waypoints for controller based on current state

**stack_master**: Main launch interface
- Central launchfiles for all operational modes
- Configuration files per racecar (`config/<racecar_version>/`)
- Checklists for procedures in `checklists/`

**f110_utils**: Shared utilities, messages, and tools
- Custom message definitions (`f110_msgs`)
- Frenet coordinate conversion library and server
- Trajectory publishers, lap analyzers, parameter optimizers
- Pit connection scripts

**sensors**: VESC motor controller interface

**system_identification**: Vehicle modeling and calibration
- Data collection automation (`id_controller`)
- Parameter estimation and lookup table generation (`id_analyser`)
- Steering lookup tables (`steering_lookup`)

### Key Architectural Patterns

**Frenet Frame**: The system extensively uses Frenet coordinates (along-track distance s, lateral offset d) for planning and control. The `frenet_conversion` library and server handle coordinate transformations.

**Sector-based Trajectory Scaling**: Racelines are divided into sectors, each with scaling factors for velocity profiles. Sectors are defined during mapping and stored per map.

**State Machine Flow**: The state machine coordinates planner/controller behavior:
- GBTRACK: Follow global raceline
- OVERTAKE: Execute local planner trajectory
- FTG: Follow-the-gap for recovery
- Emergency states for low battery/obstacles

**Lookup Tables (MAP Controller)**: Pre-computed steering angles as a function of speed and desired acceleration, generated via system identification. Tables are racecar and track-surface specific (e.g., `NUC2_hangar_pacejka`).

**Configuration per Racecar**: Each physical car has unique parameters in `stack_master/config/<racecar_version>/` for transforms, VESC tuning, sensor calibration.

## Development Guidelines

### ROS Nodes
- Use global parameters for context: `/sim`, `/measure`, `/from_bag`, `/racecar_version`
- Use ROS logging functions with INFO as default level
- Add type hints in Python for non-obvious parameters/returns
- Add docstrings (use Python Docstring Generator extension)

### Launch Files
- Document arguments with `doc` attribute
- Use consistent XML formatting

### README Structure
Each package/node should have a README with:
- Description
- Parameters
- Input/Output Topic Signature

### Code Quality
- Add TODO comments for code smells
- Remove all commented code and unused imports
- Add comments for non-obvious code or design choices

## Important Files and Locations

- Racecar configs: `stack_master/config/<racecar_version>/`
- Checklists/procedures: `stack_master/checklists/`
- Maps and trajectories: Generated during mapping, stored per map name
- Docker setup: `.docker_utils/` and `docker-compose.yaml`
- Installation scripts: `.devcontainer/.install_utils/`
- Pit connection: `f110_utils/scripts/pit_starter/`

## Common Workflows

**After modifying sector configuration**: The mapping procedure rebuilds `sector_tuner` and `overtaking_sector_tuner` packages. You must re-source the workspace afterward.

**Connecting from pit laptop**:
```bash
cd <race_stack>/f110_utils/scripts/pit_starter
source pit_starter.sh <YOUR_ZEROTIER_IP> <NUC3|NUC4> rviz
```

**Testing controller changes**: Use simulation first with `sim:=true`, then test on real car in safe environment.

**Adding a new racecar**: Create new config directory in `stack_master/config/<new_car_name>/` with transforms, VESC parameters, and sensor calibration.