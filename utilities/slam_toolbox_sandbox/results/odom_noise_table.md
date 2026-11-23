# Localization odometry noise sweep (`odom_noise`)

60 runs: Scan matcher (fixed map), AMCL (fixed map), SLAM Toolbox (own map), Odometry only (no localizer); 3 3-lap runs per noise level at 1.0 m/s. Code: 85289b5+dirty. Figures: `results/plots/odom_noise.png`, `odom_noise_track.png` (where on the track), `odom_noise_offsets.png` (constant vs random error), `odom_noise_heatmap.png` (error along the laps).

| Noise level | Actual [m/s] | Scan lost | Scan RMSE [cm] | Scan p95 [cm] | Scan heading [°] | Scan lap 1→3 [cm] | AMCL lost | AMCL RMSE [cm] | AMCL p95 [cm] | AMCL heading [°] | AMCL lap 1→3 [cm] | SLAM TB lost | SLAM TB RMSE [cm] | SLAM TB p95 [cm] | SLAM TB heading [°] | SLAM TB lap 1→3 [cm] | Odom only lost | Odom only RMSE [cm] | Odom only p95 [cm] | Odom only heading [°] | Odom only lap 1→3 [cm] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0.99 | 0/3 | 4.0 | 9.9 | 0.5 | +0.0 | 0/3 | 6.2 † | 12 | 1.2 | +1.5 | 0/3 | 5.2 † | 7.9 | 0.8 | +0.1 | 0/3 | 0.0 | 0.0 | 0.0 | +0.0 |
| 1 | 0.99 | 0/3 | 4.3 † | 11 | 0.6 | -0.1 | 0/3 | 9.4 † | 16 | 1.9 | -3.6 | 0/3 | 5.0 † | 7.9 | 0.6 | +0.2 | 3/3 | 220 | 426 | 24 | +248.7 |
| 2 | 0.99 | 0/3 | 4.6 † | 11 | 0.6 | +0.0 | 0/3 | 13 † | 23 | 2.7 | -2.5 | 0/3 | 5.0 † | 7.8 | 0.6 | +0.3 | 3/3 | 419 † | 814 | 49 | +459.0 |
| 4 | 0.99 | 0/3 | 5.3 † | 12 | 0.9 | -0.2 | 1/3 | 24 † | 37 | 4.4 | -1.6 | 0/3 | 5.3 † | 8.4 | 0.8 | +0.1 | 3/3 | 690 | 1333 | 98 | +618.3 |
| 8 | 0.99 | 0/3 | 12 | 20 | 2.6 | -1.2 | 3/3 | 37 | 61 | 6.7 | +1.8 | 0/3 | 7.3 | 12 | 1.6 | +0.4 | 3/3 | 762 | 1414 | 101 | +142.5 |
| **All levels** |  | 0/15 | 4.6 | 11 | 0.7 | -0.1 | 4/15 | 13 | 23 | 2.7 | +0.4 | 0/15 | 5.2 | 7.9 | 0.8 | +0.1 | 12/15 | 419 | 814 | 49 | +265.9 |

**Columns**, per localizer (several runs at one noise level: the median):
- **Noise level / Actual:** odometry noise level (`odom_noise.py`: every error term is multiplied by it; 0 = perfect odometry, 1 ≈ a real car), and the average speed actually driven.
- **lost:** the lap in which the error first exceeded 0.5 m, `—` if never. In the last row: runs that lost track.
- **RMSE:** root-mean-square position error over the whole run: the typical error.
- **p95:** 95% of the time the error was below this: the bad moments, without one spike deciding.
- **heading:** root-mean-square heading error.
- **lap 1→3:** RMSE of the last lap minus RMSE of the first. Positive = the error grows over the laps.
- **All levels:** medians over every run of that localizer.

**† Runs to treat with care:**
- AMCL (fixed map) at 1.0 m/s, noise 2, run 1: sim at 0.89x real time
- AMCL (fixed map) at 1.0 m/s, run 2: started on attempt 2
- Scan matcher (fixed map) at 1.0 m/s, noise 4, run 2: sim at 0.89x real time
- SLAM Toolbox (own map) at 1.0 m/s, noise 2, run 2: sim at 0.90x real time
- SLAM Toolbox (own map) at 1.0 m/s, run 3: sim at 0.90x real time
- AMCL (fixed map) at 1.0 m/s, noise 4, run 2: sim at 0.89x real time
- AMCL (fixed map) at 1.0 m/s, run 1: started on attempt 2
- Odometry only (no localizer) at 1.0 m/s, noise 2, run 2: sim at 0.87x real time
- SLAM Toolbox (own map) at 1.0 m/s, noise 2, run 1: sim at 0.89x real time
- SLAM Toolbox (own map) at 1.0 m/s, noise 4, run 3: started on attempt 3
- AMCL (fixed map) at 1.0 m/s, noise 4, run 1: sim at 0.86x real time
- Scan matcher (fixed map) at 1.0 m/s, noise 1, run 1: sim at 0.90x real time
- SLAM Toolbox (own map) at 1.0 m/s, noise 4, run 2: sim at 0.88x real time
- AMCL (fixed map) at 1.0 m/s, noise 1, run 1: sim at 0.90x real time; started on attempt 2
- AMCL (fixed map) at 1.0 m/s, noise 2, run 3: sim at 0.85x real time
- Scan matcher (fixed map) at 1.0 m/s, noise 2, run 2: sim at 0.89x real time
- SLAM Toolbox (own map) at 1.0 m/s, noise 1, run 2: sim at 0.90x real time
- SLAM Toolbox (own map) at 1.0 m/s, noise 4, run 1: sim at 0.89x real time; started on attempt 2
- AMCL (fixed map) at 1.0 m/s, noise 2, run 2: sim at 0.89x real time; started on attempt 2
- SLAM Toolbox (own map) at 1.0 m/s, noise 1, run 1: sim at 0.89x real time
- SLAM Toolbox (own map) at 1.0 m/s, noise 2, run 3: sim at 0.85x real time

**Read with care:**
- The odometry is corrupted by `odom_noise.py` (level 1: distance and turns read 2 % too large, 0.2 °/s gyro bias, random walk 1 cm and 0.3 ° per √m; higher levels multiply every term). Run r of every localizer uses noise seed r, so they all get the same corrupted odometry. The start pose is correct.
- Odometry only: no localizer, the corrupted odometry alone. Its error is the drift the localizers have to remove; with perfect odometry (level 0) it is 0.
- Errors cover only the driving window, not the car parked before and after the laps.
- Maps: Scan matcher on the walls of the map the world was generated from, off-track area unknown (`victoria_outline`); AMCL on the walls of the map the world was generated from, off-track area unknown (`victoria_outline`); SLAM Toolbox on its own recorded map, made with loop closure (`victoria_ref_lc`) with loop closure off (`localization_no_loop_closure.yaml`).
- Several runs per level, each with its own noise seed: the cells are medians over them.
