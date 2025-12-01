#!/usr/bin/env python3
"""
Generate statistics tables grouped by trajectory type.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os

def detect_trajectory_type(csv_file):
    """Detect trajectory type from CSV data."""
    df = pd.read_csv(csv_file)

    max_x = df['ref_x'].max()
    max_y = df['ref_y'].max()
    min_y = df['ref_y'].min()

    y_variation = max_y - min_y

    # Classify trajectory
    if y_variation < 0.2:  # Straight line
        return 'recta'
    elif max_x > 1.5 and max_y > 1.5:
        # Check if it's square (sharp corners) or compound (smooth arc)
        dx = np.diff(df['ref_x'].values)
        dy = np.diff(df['ref_y'].values)

        # Count smooth transitions
        smooth_changes = np.sum((np.abs(dx) < 0.01) & (np.abs(dy) < 0.01))

        if smooth_changes > len(df) * 0.3:
            return 'compuesta'
        else:
            return 'cuadrada'
    else:
        return 'unknown'

def group_files_by_trajectory(directory='tb3_runs'):
    """Group CSV files by trajectory type."""
    csv_files = [f for f in glob.glob(os.path.join(directory, '*.csv'))
                 if 'summary' not in f]

    trajectory_groups = {'recta': [], 'cuadrada': [], 'compuesta': []}

    for csv_file in csv_files:
        try:
            traj_type = detect_trajectory_type(csv_file)
            if traj_type in trajectory_groups:
                trajectory_groups[traj_type].append(csv_file)
        except Exception as e:
            print(f"Error processing {csv_file}: {e}")

    return trajectory_groups

def calculate_controller_stats(csv_files):
    """Calculate statistics for controllers from CSV files."""
    controller_data = {}

    for csv_file in csv_files:
        filename = os.path.basename(csv_file)

        # Detect controller type
        if filename.startswith('pid'):
            controller = 'pid'
        elif filename.startswith('lyapunov'):
            controller = 'lyapunov'
        elif filename.startswith('mpc'):
            controller = 'mpc'
        elif filename.startswith('pure'):
            controller = 'pure'
        else:
            continue

        if controller not in controller_data:
            controller_data[controller] = []

        try:
            df = pd.read_csv(csv_file)

            t = df['t'].values
            err = df['err'].values
            v = df['v'].values
            w = df['w'].values

            # Calculate metrics
            metrics = {
                'IAE': np.trapezoid(np.abs(err), t),
                'ISE': np.trapezoid(err**2, t),
                'ICE': np.trapezoid(v**2 + w**2, t),
                'mean_error': np.mean(np.abs(err)),
                'max_error': np.max(np.abs(err)),
                'final_error': np.abs(err[-1]),
                'total_time': t[-1] - t[0],
                'distance_traveled': np.trapezoid(np.abs(v), t)
            }

            controller_data[controller].append(metrics)
        except Exception as e:
            print(f"Error processing {csv_file}: {e}")

    # Calculate aggregated statistics
    controller_stats = {}
    for controller, runs in controller_data.items():
        if runs:
            controller_stats[controller] = {
                metric: {
                    'mean': np.mean([run[metric] for run in runs]),
                    'std': np.std([run[metric] for run in runs])
                }
                for metric in runs[0].keys()
            }

    return controller_stats

def generate_statistics_table_by_trajectory(controller_stats, trajectory_type, output_dir='charts'):
    """Generate statistics table for a specific trajectory type."""

    if not controller_stats:
        print(f"No data available for trajectory type: {trajectory_type}")
        return None

    fig, ax = plt.subplots(figsize=(14, 10))
    ax.axis('tight')
    ax.axis('off')

    # Prepare table data
    controllers = list(controller_stats.keys())
    metrics = ['IAE', 'ISE', 'ICE', 'mean_error', 'max_error', 'final_error', 'total_time', 'distance_traveled']
    metric_names = ['IAE (m·s)', 'ISE (m²·s)', 'ICE', 'Mean Error (m)',
                    'Max Error (m)', 'Final Error (m)', 'Total Time (s)', 'Distance Traveled (m)']

    table_data = []
    table_data.append(['Metric'] + [f'{c.upper()}' for c in controllers])

    for metric, name in zip(metrics, metric_names):
        row = [name]
        for controller in controllers:
            mean_val = controller_stats[controller][metric]['mean']
            std_val = controller_stats[controller][metric]['std']
            row.append(f'{mean_val:.4f} ± {std_val:.4f}')
        table_data.append(row)

    # Create table
    table = ax.table(cellText=table_data, cellLoc='center', loc='center',
                     colWidths=[0.2] + [0.2] * len(controllers))

    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2)

    # Style header row
    for i in range(len(controllers) + 1):
        table[(0, i)].set_facecolor('#4CAF50')
        table[(0, i)].set_text_props(weight='bold', color='white')

    # Style metric names column
    for i in range(1, len(metrics) + 1):
        table[(i, 0)].set_facecolor('#E8E8E8')
        table[(i, 0)].set_text_props(weight='bold')

    # Highlight best values in each row
    for i, metric in enumerate(metrics, start=1):
        means = [controller_stats[c][metric]['mean'] for c in controllers]
        best_idx = means.index(min(means))
        table[(i, best_idx + 1)].set_facecolor('#90EE90')
        table[(i, best_idx + 1)].set_text_props(weight='bold')

    # Title with trajectory type
    traj_names = {
        'recta': 'Straight Line',
        'cuadrada': 'Square',
        'compuesta': 'Compound (Line + Arc)'
    }
    traj_display = traj_names.get(trajectory_type, trajectory_type.title())

    plt.title(f'Controller Performance Statistics - {traj_display} Trajectory\n(Best values highlighted in green)',
              fontsize=14, fontweight='bold', pad=20)

    output_file = os.path.join(output_dir, f'controller_statistics_{trajectory_type}.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"✓ Statistics table saved to: {output_file}")
    return output_file

def main():
    print("="*80)
    print("GENERATING STATISTICS TABLES BY TRAJECTORY TYPE")
    print("="*80 + "\n")

    # Create output directory
    os.makedirs('charts', exist_ok=True)

    # Group files by trajectory type
    print("Analyzing trajectory types...")
    trajectory_groups = group_files_by_trajectory()

    print("\nTrajectory distribution:")
    for traj_type, files in trajectory_groups.items():
        print(f"  {traj_type.upper():12s}: {len(files):3d} files")

    print("\nGenerating statistics tables...\n")

    # Generate table for each trajectory type
    generated_files = []
    for traj_type, files in trajectory_groups.items():
        if files:
            print(f"Processing {traj_type} trajectory ({len(files)} files)...")
            controller_stats = calculate_controller_stats(files)
            if controller_stats:
                output_file = generate_statistics_table_by_trajectory(
                    controller_stats, traj_type
                )
                if output_file:
                    generated_files.append(output_file)
        else:
            print(f"⚠ No data found for {traj_type} trajectory - skipping")

    print("\n" + "="*80)
    print("GENERATION COMPLETE")
    print("="*80)
    print(f"Total tables generated: {len(generated_files)}")
    for f in generated_files:
        print(f"  ✓ {f}")
    print("="*80 + "\n")

if __name__ == '__main__':
    main()
