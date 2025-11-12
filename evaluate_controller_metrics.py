#!/usr/bin/env python3
"""
Script to evaluate controller performance metrics from CSV data.

Metrics calculated:
- IAE: Integral of Absolute Error
- ITAE: Integral of Time-weighted Absolute Error
- ISE: Integral of Squared Error
- ICE: Integral of Control Effort

Author: Claude AI
Date: 2025-11-05
"""

import pandas as pd
import numpy as np
import os
import glob
from pathlib import Path


def calculate_metrics(csv_file):
    """
    Calculate performance metrics for a controller run.

    Args:
        csv_file (str): Path to the CSV file

    Returns:
        dict: Dictionary containing all calculated metrics
    """
    # Read CSV file
    df = pd.read_csv(csv_file)

    # Extract data
    t = df['t'].values
    err = df['err'].values
    v = df['v'].values
    w = df['w'].values

    # Calculate time differences for integration (trapezoidal rule)
    dt = np.diff(t)

    # IAE: Integral of Absolute Error
    # ∫|e(t)|dt
    iae = np.trapezoid(np.abs(err), t)

    # ITAE: Integral of Time-weighted Absolute Error
    # ∫t|e(t)|dt
    itae = np.trapezoid(t * np.abs(err), t)

    # ISE: Integral of Squared Error
    # ∫e²(t)dt
    ise = np.trapezoid(err**2, t)

    # ICE: Integral of Control Effort
    # ∫(v² + w²)dt - considering both linear and angular velocity
    control_effort = v**2 + w**2
    ice = np.trapezoid(control_effort, t)

    # Additional useful metrics
    max_error = np.max(np.abs(err))
    mean_error = np.mean(np.abs(err))
    final_error = np.abs(err[-1])

    # Settling time (time to reach and stay within 5% of final value)
    threshold = 0.05 * max_error if max_error > 0 else 0.05
    settling_indices = np.where(np.abs(err) > threshold)[0]
    settling_time = t[settling_indices[-1]] if len(settling_indices) > 0 else t[0]

    # Total simulation time
    total_time = t[-1] - t[0]

    return {
        'file': os.path.basename(csv_file),
        'IAE': iae,
        'ITAE': itae,
        'ISE': ise,
        'ICE': ice,
        'max_error': max_error,
        'mean_error': mean_error,
        'final_error': final_error,
        'settling_time': settling_time,
        'total_time': total_time,
        'num_samples': len(t)
    }


def extract_trajectory_and_controller(filename):
    """
    Extract trajectory type and controller type from filename.

    Args:
        filename (str): Name of the CSV file

    Returns:
        tuple: (trajectory_type, controller_type)
    """
    filename_lower = filename.lower()

    # Determine trajectory type
    if filename.startswith('recta_'):
        trajectory = 'recta'
        # Remove 'recta_' prefix to get controller
        remainder = filename[6:]
    elif filename.startswith('compuesta_'):
        trajectory = 'compuesta'
        # Remove 'compuesta_' prefix to get controller
        remainder = filename[10:]
    else:
        trajectory = 'circular'
        remainder = filename

    # Extract controller type
    # Handle different naming patterns
    if remainder.startswith('pid') or remainder.startswith('PID'):
        controller = 'pid'
    elif remainder.startswith('mpc') or remainder.startswith('MPC'):
        controller = 'mpc'
    elif remainder.startswith('lyapunov'):
        controller = 'lyapunov'
    elif remainder.startswith('pure'):
        controller = 'pure'
    else:
        controller = 'unknown'

    return trajectory, controller


def analyze_directory(directory_path, output_file='controller_metrics_summary.csv'):
    """
    Analyze all CSV files in a directory and generate summary report.

    Args:
        directory_path (str): Path to directory containing CSV files
        output_file (str): Name of output CSV file for summary
    """
    # Find all CSV files (exclude summary files)
    csv_files = glob.glob(os.path.join(directory_path, '*.csv'))
    csv_files = [f for f in csv_files if 'summary' not in os.path.basename(f).lower()]

    if not csv_files:
        print(f"No CSV files found in {directory_path}")
        return None

    print(f"Found {len(csv_files)} CSV files")
    print("Calculating metrics...\n")

    # Calculate metrics for each file
    results = []
    for csv_file in sorted(csv_files):
        try:
            metrics = calculate_metrics(csv_file)

            # Extract trajectory and controller type
            trajectory, controller = extract_trajectory_and_controller(metrics['file'])
            metrics['trajectory'] = trajectory
            metrics['controller'] = controller

            results.append(metrics)
            print(f"Processed: {metrics['file']} (trajectory: {trajectory}, controller: {controller})")
        except Exception as e:
            print(f"Error processing {csv_file}: {e}")

    # Create DataFrame with results
    df_results = pd.DataFrame(results)

    # Reorder columns
    cols = ['file', 'trajectory', 'controller', 'IAE', 'ITAE', 'ISE', 'ICE',
            'max_error', 'mean_error', 'final_error', 'settling_time',
            'total_time', 'num_samples']
    df_results = df_results[cols]

    # Save results
    output_path = os.path.join(directory_path, output_file)
    df_results.to_csv(output_path, index=False, float_format='%.6f')
    print(f"\n✓ Results saved to: {output_path}")

    return df_results


def print_summary_statistics(df_results):
    """
    Print summary statistics grouped by trajectory and controller type.

    Args:
        df_results (pd.DataFrame): DataFrame with metrics results
    """
    metrics_to_summarize = ['IAE', 'ITAE', 'ISE', 'ICE',
                            'max_error', 'mean_error', 'final_error']

    # Get unique trajectories
    trajectories = sorted(df_results['trajectory'].unique())

    for trajectory in trajectories:
        print("\n" + "="*80)
        print(f"TRAJECTORY: {trajectory.upper()}")
        print("="*80)

        # Filter data for this trajectory
        traj_data = df_results[df_results['trajectory'] == trajectory]

        # Group by controller type
        grouped = traj_data.groupby('controller')

        for controller, group in grouped:
            print(f"\n{controller.upper()} Controller ({len(group)} runs)")
            print("-" * 80)

            for metric in metrics_to_summarize:
                if metric in group.columns:
                    mean_val = group[metric].mean()
                    std_val = group[metric].std()
                    min_val = group[metric].min()
                    max_val = group[metric].max()

                    print(f"{metric:15s}: Mean={mean_val:10.6f} ± {std_val:10.6f} "
                          f"(Min={min_val:10.6f}, Max={max_val:10.6f})")

        print("\n" + "-"*80)
        print(f"COMPARISON FOR {trajectory.upper()} TRAJECTORY (Mean values)")
        print("-"*80)

        summary = grouped[metrics_to_summarize].mean(numeric_only=True)
        print(summary.to_string())
        print("\n")


def compare_controllers(df_results):
    """
    Generate a comparison table showing which controller performs best for each metric,
    separated by trajectory type.

    Args:
        df_results (pd.DataFrame): DataFrame with metrics results
    """
    metrics_to_compare = ['IAE', 'ITAE', 'ISE', 'ICE',
                          'max_error', 'mean_error', 'final_error']

    # Get unique trajectories
    trajectories = sorted(df_results['trajectory'].unique())

    for trajectory in trajectories:
        print("\n" + "="*80)
        print(f"BEST CONTROLLER FOR EACH METRIC - {trajectory.upper()} TRAJECTORY")
        print("(Lower is better)")
        print("="*80 + "\n")

        # Filter data for this trajectory
        traj_data = df_results[df_results['trajectory'] == trajectory]

        # Group by controller and calculate mean (only numeric columns)
        grouped = traj_data.groupby('controller').mean(numeric_only=True)

        for metric in metrics_to_compare:
            if metric in grouped.columns:
                best_controller = grouped[metric].idxmin()
                best_value = grouped[metric].min()
                worst_controller = grouped[metric].idxmax()
                worst_value = grouped[metric].max()
                print(f"{metric:15s}: Best={best_controller:10s} ({best_value:8.6f})  "
                      f"Worst={worst_controller:10s} ({worst_value:8.6f})")

        print("\n")


def compare_trajectories(df_results):
    """
    Compare performance across different trajectory types for each controller.

    Args:
        df_results (pd.DataFrame): DataFrame with metrics results
    """
    print("\n" + "="*80)
    print("TRAJECTORY COMPARISON BY CONTROLLER")
    print("(Mean values - Lower is better for all metrics)")
    print("="*80 + "\n")

    metrics_to_compare = ['IAE', 'ITAE', 'ISE', 'ICE',
                          'max_error', 'mean_error', 'final_error']

    # Get unique controllers and trajectories
    controllers = sorted(df_results['controller'].unique())
    trajectories = sorted(df_results['trajectory'].unique())

    for controller in controllers:
        print(f"\n{controller.upper()} Controller")
        print("-" * 80)

        # Filter data for this controller
        controller_data = df_results[df_results['controller'] == controller]

        # Group by trajectory
        grouped = controller_data.groupby('trajectory').mean(numeric_only=True)

        for metric in metrics_to_compare:
            if metric in grouped.columns:
                print(f"\n{metric:15s}:")
                for trajectory in trajectories:
                    if trajectory in grouped.index:
                        value = grouped.loc[trajectory, metric]
                        print(f"  {trajectory:12s}: {value:10.6f}")

                # Find best trajectory for this metric
                best_traj = grouped[metric].idxmin()
                best_val = grouped[metric].min()
                print(f"  → Best: {best_traj} ({best_val:.6f})")

    # Overall summary table
    print("\n" + "="*80)
    print("OVERALL SUMMARY TABLE (Mean values)")
    print("="*80 + "\n")

    # Create pivot table for easier viewing
    for metric in metrics_to_compare:
        print(f"\n{metric}:")
        pivot = df_results.pivot_table(
            values=metric,
            index='controller',
            columns='trajectory',
            aggfunc='mean'
        )
        print(pivot.to_string(float_format='%.6f'))
        print()

    print("\n")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description='Evaluate controller performance metrics from CSV data'
    )
    parser.add_argument(
        '--directory',
        type=str,
        default='tb3_runs',
        help='Directory containing CSV files (default: tb3_runs)'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='controller_metrics_summary.csv',
        help='Output CSV filename (default: controller_metrics_summary.csv)'
    )
    parser.add_argument(
        '--file',
        type=str,
        help='Analyze a single CSV file instead of a directory'
    )

    args = parser.parse_args()

    print("="*80)
    print("CONTROLLER PERFORMANCE METRICS EVALUATION")
    print("="*80 + "\n")

    if args.file:
        # Analyze single file
        print(f"Analyzing single file: {args.file}\n")
        metrics = calculate_metrics(args.file)

        print("Results:")
        print("-" * 80)
        for key, value in metrics.items():
            if isinstance(value, float):
                print(f"{key:20s}: {value:.6f}")
            else:
                print(f"{key:20s}: {value}")
    else:
        # Analyze directory
        print(f"Analyzing directory: {args.directory}\n")
        df_results = analyze_directory(args.directory, args.output)

        if df_results is not None and len(df_results) > 0:
            print_summary_statistics(df_results)
            compare_controllers(df_results)
            compare_trajectories(df_results)

            print("\nFor detailed results, see the CSV file:")
            print(f"→ {os.path.join(args.directory, args.output)}")
