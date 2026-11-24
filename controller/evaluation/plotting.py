"""Generate static Matplotlib reports; importing this module needs no display."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def generate_plots(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    directory = Path(directory)
    with (directory / 'planned_path.csv').open(newline='') as stream:
        path = list(csv.DictReader(stream))
    with (directory / 'samples.csv').open(newline='') as stream:
        samples = list(csv.DictReader(stream))
    summary = json.loads((directory / 'summary.json').read_text())
    def values(rows, key):
        return np.array([float(r[key]) if r.get(key) not in (None, '') else np.nan for r in rows])
    def decorate(ax):
        ax.set_xlabel('Map x (m)')
        ax.set_ylabel('Map y (m)')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.25)
    planned = np.column_stack([values(path, 'x'), values(path, 'y')]) if path else np.empty((0, 2))
    if len(planned):
        closed = np.vstack([planned, planned[0]])
        segments = np.stack([closed[:-1], closed[1:]], axis=1)
        fig, ax = plt.subplots(figsize=(9, 7))
        collection = LineCollection(segments, cmap='viridis', linewidth=2)
        collection.set_array(values(path, 'reference_speed'))
        ax.add_collection(collection)
        ax.scatter(planned[:, 0], planned[:, 1], c=values(path, 'reference_speed'),
                   cmap='viridis', s=8)
        ax.autoscale()
        ax.plot(planned[0, 0], planned[0, 1], 'kx', label='Reference waypoint 0')
        fig.colorbar(collection, ax=ax, label='Desired waypoint speed (m/s)')
        ax.set_title('Planned path and scaled reference speed')
        ax.legend()
        decorate(ax)
        fig.tight_layout()
        fig.savefig(directory / 'planned_path.png', dpi=170)
        plt.close(fig)
    else:
        fig, ax = plt.subplots(figsize=(9, 7))
        ax.text(.5, .5, 'No planned path received before trial termination',
                transform=ax.transAxes, ha='center', va='center', wrap=True)
        ax.set_title('Planned path unavailable')
        decorate(ax)
        fig.tight_layout()
        fig.savefig(directory / 'planned_path.png', dpi=170)
        plt.close(fig)

    colors = {1: 'tab:blue', 2: 'tab:orange'}
    fig, axes = plt.subplots(3, 1, figsize=(9, 15), sharex=True, sharey=True)
    all_x, all_y = [planned[:, 0]], [planned[:, 1]]
    if samples:
        all_x.append(values(samples, 'pose_x'))
        all_y.append(values(samples, 'pose_y'))
    finite_x = np.concatenate(all_x)
    finite_y = np.concatenate(all_y)
    finite_x, finite_y = finite_x[np.isfinite(finite_x)], finite_y[np.isfinite(finite_y)]
    for panel, ax in enumerate(axes):
        if len(planned):
            ax.plot(closed[:, 0], closed[:, 1], '--', color='.45', lw=1.2, label='Planned path')
        selected_laps = [1, 2] if panel == 0 else [panel]
        for lap in selected_laps:
            rows = [r for r in samples if int(r['lap']) == lap]
            if rows:
                ax.plot(values(rows, 'pose_x'), values(rows, 'pose_y'),
                        color=colors[lap], lw=1.2, label=f'Lap {lap}')
        title = 'Complete trajectory: both laps' if panel == 0 else f'Lap {panel} trajectory'
        if not summary['success']:
            title += ' — incomplete trial'
        ax.set_title(title)
        decorate(ax)
        if ax.get_legend_handles_labels()[0]:
            ax.legend()
    if len(finite_x):
        span = max(np.ptp(finite_x), np.ptp(finite_y), 1)
        axes[0].set_xlim(np.min(finite_x) - .05 * span, np.max(finite_x) + .05 * span)
        axes[0].set_ylim(np.min(finite_y) - .05 * span, np.max(finite_y) + .05 * span)
    fig.tight_layout()
    fig.savefig(directory / 'trajectory_comparison.png', dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    experiment = summary['experiment']
    capabilities = experiment.get('report_capabilities', ['constraints'] if 'solver_plugin' in experiment['controller_parameters'] else [])
    speed_label = experiment.get('reference_speed_label', 'Reference')
    steering_label = experiment.get('reference_steering_label', 'Reference')
    t = values(samples, 'elapsed_s')
    axes[0].plot(t, values(samples, 'position_error_m'), color='tab:blue')
    axes[0].set_ylabel('Position error (m)')
    heading_error = values(samples, 'heading_error_deg')
    if samples and 'heading_error_deg' not in samples[0]:
        # Older recordings stored the wrapped error in radians.
        heading_error = np.rad2deg(values(samples, 'heading_error_rad'))
    axes[1].plot(t, heading_error, color='tab:blue')
    axes[1].set_ylabel('Heading error (deg)')
    axes[2].plot(t, values(samples, 'reference_speed'), '--', color='.35', label=speed_label)
    axes[2].plot(t, values(samples, 'command_speed'), color='tab:blue', label='Command')
    axes[2].plot(t, values(samples, 'measured_speed'), color='tab:orange', label='Measured')
    axes[2].set_ylabel('Speed (m/s)')
    axes[2].legend()
    axes[3].plot(t, values(samples, 'reference_steering'), '--', color='.35', label=steering_label)
    axes[3].plot(t, values(samples, 'command_steering'), color='tab:blue', label='Command')
    cfg = summary['experiment']['controller_parameters']
    for bound in ('steering_min', 'steering_max'):
        if 'constraints' in capabilities and cfg.get(bound) is not None:
            axes[3].axhline(cfg[bound], color='tab:red', ls=':', lw=1)
    axes[3].set_ylabel('Steering angle (rad)')
    axes[3].set_xlabel('Elapsed time (s)')
    axes[3].legend()
    for ax in axes:
        ax.grid(alpha=.25)
        if summary['lap_boundaries_s']:
            ax.axvline(summary['lap_boundaries_s'][0], color='tab:green', ls=':', label='Lap boundary')
    if summary['lap_boundaries_s']:
        axes[0].annotate('Lap 1 → Lap 2', (summary['lap_boundaries_s'][0], 1),
                         xycoords=('data', 'axes fraction'), xytext=(5, -15),
                         textcoords='offset points')
    fig.suptitle('Tracking and commands — ' + summary['termination_reason'])
    fig.tight_layout()
    fig.savefig(directory / 'tracking_and_commands.png', dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='Regenerate controller reports from saved CSV/JSON')
    parser.add_argument('directory')
    args = parser.parse_args()
    generate_plots(args.directory)


if __name__ == '__main__':
    main()
