# Localization speed sweep (`amcl_outline_vs_speedsweep`)

51 runs: Scan matcher (fixed map), AMCL (fixed map), SLAM Toolbox (own map); one 3-lap run per speed. Code: 85289b5, 85289b5+dirty. Figures: `results/plots/amcl_outline_vs_speedsweep.png`, `amcl_outline_vs_speedsweep_track.png` (where on the track), `amcl_outline_vs_speedsweep_offsets.png` (constant vs random error), `amcl_outline_vs_speedsweep_heatmap.png` (error along the laps).

| Speed [m/s] | Actual [m/s] | Scan lost | Scan RMSE [cm] | Scan p95 [cm] | Scan heading [°] | Scan lap 1→3 [cm] | AMCL lost | AMCL RMSE [cm] | AMCL p95 [cm] | AMCL heading [°] | AMCL lap 1→3 [cm] | SLAM TB lost | SLAM TB RMSE [cm] | SLAM TB p95 [cm] | SLAM TB heading [°] | SLAM TB lap 1→3 [cm] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.20 | 0.20 | — | 5.2 | 14 | 0.6 | -0.1 | — | 6.1 | 11 | 1.1 | +0.7 | lap 1 | 90 | 149 | 7.6 | -13.6 |
| 0.25 | 0.25 | — | 5.1 | 14 | 0.6 | -0.2 | — | 6.3 | 11 | 1.2 | +0.7 | lap 1 | 70 † | 151 | 6.2 | -64.2 |
| 0.30 | 0.30 | — | 5.0 | 14 | 0.6 | +0.1 | — | 6.6 | 13 | 1.2 | +2.5 | lap 1 | 48 † | 127 | 4.7 | -67.9 |
| 0.35 | 0.35 | — | 5.2 | 14 | 0.6 | -0.1 | — | 6.3 | 11 | 1.1 | +0.7 | lap 1 | 93 | 181 | 7.9 | -2.5 |
| 0.40 | 0.40 | — | 5.1 | 14 | 0.6 | -0.0 | — | 6.3 | 11 | 1.2 | +0.6 | lap 1 | 102 | 180 | 8.4 | -36.0 |
| 0.45 | 0.45 | — | 5.1 | 14 | 0.6 | -0.0 | — | 6.1 | 11 | 1.2 | +0.4 | lap 1 | 101 † | 189 | 8.2 | -16.5 |
| 0.50 | 0.50 | — | 5.1 | 14 | 0.6 | -0.2 | — | 6.7 | 13 | 1.1 | -2.3 | lap 1 | 775 | 1540 | 86 | +1076.0 |
| 0.55 | 0.55 | — | 5.1 | 14 | 0.6 | -0.1 | — | 5.7 | 11 | 1.2 | -0.6 | lap 1 | 92 | 161 | 7.8 | -48.1 |
| 0.60 | 0.60 | — | 5.1 | 14 | 0.6 | -0.0 | — | 6.1 | 11 | 1.1 | -0.0 | lap 1 | 914 | 1527 | 95 | -13.1 |
| 0.65 | 0.65 | — | 5.3 | 14 | 0.6 | +0.1 | — | 5.4 | 10 | 1.0 | -0.8 | lap 1 | 88 | 165 | 7.1 | -20.6 |
| 0.70 | 0.69 | — | 5.2 | 14 | 0.6 | -0.2 | — | 6.3 | 11 | 1.1 | +2.4 | lap 1 | 698 | 1313 | 102 | +776.1 |
| 0.75 | 0.74 | — | 5.0 | 13 | 0.6 | -0.1 | — | 5.5 | 10 | 1.1 | -0.8 | lap 1 | 104 | 189 | 8.4 | +27.7 |
| 0.80 | 0.79 | — | 4.8 | 13 | 0.6 | -0.1 | — | 6.3 | 11 | 1.2 | +2.7 | lap 1 | 62 | 131 | 4.7 | -2.2 |
| 0.85 | 0.84 | — | 5.1 | 14 | 0.6 | -0.1 | — | 6.6 | 12 | 1.2 | +0.7 | lap 1 | 568 † | 1531 | 57 | +863.5 |
| 0.90 | 0.89 | — | 4.9 | 14 | 0.6 | -0.1 | — | 6.4 | 11 | 1.1 | +0.3 | lap 1 | 101 † | 219 | 8.4 | -6.4 |
| 0.95 | 0.94 | — | 5.1 | 14 | 0.6 | +0.0 | — | 5.9 | 11 | 1.2 | +2.7 | lap 1 | 66 | 168 | 6.6 | -91.6 |
| 1.0 | 0.99 | — | 5.0 | 14 | 0.6 | -0.2 | — | 6.0 | 11 | 1.1 | +2.1 | lap 1 | 87 † | 180 | 7.3 | -35.9 |
| **All speeds** |  | 0/17 | 5.1 | 14 | 0.6 | -0.1 | 0/17 | 6.3 | 11 | 1.1 | +0.7 | 17/17 | 93 | 180 | 7.9 | -13.6 |

**Columns**, per localizer (several runs at one speed: the median):
- **Speed / Actual:** commanded cruise speed, and the average actually driven (autodrive slows in turns).
- **lost:** the lap in which the error first exceeded 0.5 m, `—` if never. In the last row: runs that lost track.
- **RMSE:** root-mean-square position error over the whole run: the typical error.
- **p95:** 95% of the time the error was below this: the bad moments, without one spike deciding.
- **heading:** root-mean-square heading error.
- **lap 1→3:** RMSE of the last lap minus RMSE of the first. Positive = the error grows over the laps.
- **All speeds:** medians over every run of that localizer.

**† Runs to treat with care:**
- SLAM Toolbox (own map) at 0.45 m/s: sim at 0.70x real time
- SLAM Toolbox (own map) at 0.85 m/s: sim at 0.89x real time
- SLAM Toolbox (own map) at 1.0 m/s: started on attempt 2
- SLAM Toolbox (own map) at 0.25 m/s: sim at 0.83x real time
- SLAM Toolbox (own map) at 0.90 m/s: started on attempt 2
- SLAM Toolbox (own map) at 0.30 m/s: sim at 0.77x real time

**Read with care:**
- Odometry is perfect and the start pose correct, so this measures whether a localizer *disturbs* good odometry, not whether it can *correct* drift. A localizer that does nothing would score about 0.
- Errors cover only the driving window, not the car parked before and after the laps.
- Maps: Scan matcher on the filled-in map the world was generated from (`Victoria.yaml`); AMCL on the walls of the map the world was generated from, off-track area unknown (`victoria_outline`); SLAM Toolbox on its own recorded map (`victoria_ref`) with loop closure on (`localization.yaml`).
- One run per speed: results vary from run to run (especially AMCL and SLAM Toolbox), so read the trend over speeds, not single steps.
