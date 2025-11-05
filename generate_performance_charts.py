#!/usr/bin/env python3
"""
Script to generate performance visualization charts for controller analysis.

This script creates:
1. Individual performance charts for each controller run
2. Summary comparison charts for all controllers

Author: Claude AI
Date: 2025-11-05
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
from pathlib import Path
import seaborn as sns
from matplotlib.backends.backend_pdf import PdfPages

# Set style for better-looking plots
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")


def plot_individual_controller(csv_file, output_dir='charts'):
    """
    Generate detailed performance charts for a single controller run.

    Args:
        csv_file (str): Path to the CSV file
        output_dir (str): Directory to save output charts
    """
    # Read data
    df = pd.read_csv(csv_file)

    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Extract controller name and timestamp
    filename = os.path.basename(csv_file)
    controller_name = filename.split('_')[0]

    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(f'Controller Performance: {filename}', fontsize=16, fontweight='bold')

    # 1. Trajectory plot
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(df['ref_x'], df['ref_y'], 'k--', linewidth=2, label='Reference', alpha=0.7)
    ax1.plot(df['x'], df['y'], 'b-', linewidth=1.5, label='Actual')
    ax1.scatter(df['x'].iloc[0], df['y'].iloc[0], c='green', s=100, marker='o',
                label='Start', zorder=5)
    ax1.scatter(df['x'].iloc[-1], df['y'].iloc[-1], c='red', s=100, marker='s',
                label='End', zorder=5)
    ax1.set_xlabel('X position (m)', fontsize=10)
    ax1.set_ylabel('Y position (m)', fontsize=10)
    ax1.set_title('Trajectory Tracking', fontweight='bold')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # 2. Tracking error over time
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(df['t'], df['err'], 'r-', linewidth=1.5)
    ax2.fill_between(df['t'], 0, df['err'], alpha=0.3, color='red')
    ax2.set_xlabel('Time (s)', fontsize=10)
    ax2.set_ylabel('Tracking Error (m)', fontsize=10)
    ax2.set_title('Tracking Error vs Time', fontweight='bold')
    ax2.grid(True, alpha=0.3)

    # Add statistics box
    mean_err = np.mean(np.abs(df['err']))
    max_err = np.max(np.abs(df['err']))
    textstr = f'Mean: {mean_err:.4f} m\nMax: {max_err:.4f} m'
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax2.text(0.65, 0.95, textstr, transform=ax2.transAxes, fontsize=9,
             verticalalignment='top', bbox=props)

    # 3. Linear velocity (v)
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(df['t'], df['v'], 'g-', linewidth=1.5)
    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('Linear Velocity (m/s)', fontsize=10)
    ax3.set_title('Linear Velocity (v)', fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # 4. Angular velocity (w)
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(df['t'], df['w'], 'm-', linewidth=1.5)
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Angular Velocity (rad/s)', fontsize=10)
    ax4.set_title('Angular Velocity (w)', fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # 5. X and Y position tracking
    ax5 = plt.subplot(2, 3, 5)
    ax5.plot(df['t'], df['ref_x'], 'k--', linewidth=2, label='Ref X', alpha=0.7)
    ax5.plot(df['t'], df['x'], 'b-', linewidth=1.5, label='Actual X')
    ax5.plot(df['t'], df['ref_y'], 'gray', linestyle='--', linewidth=2, label='Ref Y', alpha=0.7)
    ax5.plot(df['t'], df['y'], 'orange', linewidth=1.5, label='Actual Y')
    ax5.set_xlabel('Time (s)', fontsize=10)
    ax5.set_ylabel('Position (m)', fontsize=10)
    ax5.set_title('Position Components vs Time', fontweight='bold')
    ax5.legend(loc='best', fontsize=8)
    ax5.grid(True, alpha=0.3)

    # 6. Control effort (combined v and w)
    ax6 = plt.subplot(2, 3, 6)
    control_effort = np.sqrt(df['v']**2 + df['w']**2)
    ax6.plot(df['t'], control_effort, 'c-', linewidth=1.5)
    ax6.fill_between(df['t'], 0, control_effort, alpha=0.3, color='cyan')
    ax6.set_xlabel('Time (s)', fontsize=10)
    ax6.set_ylabel('Control Effort', fontsize=10)
    ax6.set_title('Total Control Effort', fontweight='bold')
    ax6.grid(True, alpha=0.3)

    # Add statistics box
    mean_effort = np.mean(control_effort)
    max_effort = np.max(control_effort)
    textstr = f'Mean: {mean_effort:.4f}\nMax: {max_effort:.4f}'
    props = dict(boxstyle='round', facecolor='lightblue', alpha=0.5)
    ax6.text(0.65, 0.95, textstr, transform=ax6.transAxes, fontsize=9,
             verticalalignment='top', bbox=props)

    plt.tight_layout()

    # Save figure
    output_file = os.path.join(output_dir, f'{filename[:-4]}_performance.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    return output_file


def generate_summary_comparison(directory_path, output_dir='charts'):
    """
    Generate summary comparison charts for all controllers.

    Args:
        directory_path (str): Path to directory containing CSV files
        output_dir (str): Directory to save output charts
    """
    # Find all CSV files
    csv_files = glob.glob(os.path.join(directory_path, '*.csv'))

    if not csv_files:
        print(f"No CSV files found in {directory_path}")
        return None

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Group files by controller type
    controller_data = {}

    for csv_file in csv_files:
        filename = os.path.basename(csv_file)
        controller_type = filename.split('_')[0]

        if controller_type not in controller_data:
            controller_data[controller_type] = []

        try:
            df = pd.read_csv(csv_file)

            # Calculate metrics
            t = df['t'].values
            err = df['err'].values
            v = df['v'].values
            w = df['w'].values

            metrics = {
                'IAE': np.trapezoid(np.abs(err), t),
                'ISE': np.trapezoid(err**2, t),
                'ICE': np.trapezoid(v**2 + w**2, t),
                'max_error': np.max(np.abs(err)),
                'mean_error': np.mean(np.abs(err)),
                'final_error': np.abs(err[-1]),
                'total_time': t[-1] - t[0]
            }

            controller_data[controller_type].append(metrics)
        except Exception as e:
            print(f"Error processing {csv_file}: {e}")

    # Calculate statistics for each controller
    controller_stats = {}
    for controller, runs in controller_data.items():
        if runs:
            controller_stats[controller] = {
                metric: {
                    'mean': np.mean([run[metric] for run in runs]),
                    'std': np.std([run[metric] for run in runs]),
                    'min': np.min([run[metric] for run in runs]),
                    'max': np.max([run[metric] for run in runs])
                }
                for metric in runs[0].keys()
            }

    # Create comprehensive comparison figure
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle('Controller Performance Comparison - All Controllers',
                 fontsize=18, fontweight='bold')

    controllers = list(controller_stats.keys())
    colors = plt.cm.Set3(np.linspace(0, 1, len(controllers)))

    # 1. IAE Comparison
    ax1 = plt.subplot(2, 3, 1)
    means = [controller_stats[c]['IAE']['mean'] for c in controllers]
    stds = [controller_stats[c]['IAE']['std'] for c in controllers]
    bars = ax1.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax1.set_ylabel('IAE (m·s)', fontsize=11, fontweight='bold')
    ax1.set_title('Integral of Absolute Error (IAE)', fontweight='bold', fontsize=12)
    ax1.grid(True, alpha=0.3, axis='y')
    # Add value labels on bars
    for bar in bars:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.3f}', ha='center', va='bottom', fontsize=9)

    # 2. ISE Comparison
    ax2 = plt.subplot(2, 3, 2)
    means = [controller_stats[c]['ISE']['mean'] for c in controllers]
    stds = [controller_stats[c]['ISE']['std'] for c in controllers]
    bars = ax2.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax2.set_ylabel('ISE (m²·s)', fontsize=11, fontweight='bold')
    ax2.set_title('Integral of Squared Error (ISE)', fontweight='bold', fontsize=12)
    ax2.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.3f}', ha='center', va='bottom', fontsize=9)

    # 3. ICE Comparison
    ax3 = plt.subplot(2, 3, 3)
    means = [controller_stats[c]['ICE']['mean'] for c in controllers]
    stds = [controller_stats[c]['ICE']['std'] for c in controllers]
    bars = ax3.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax3.set_ylabel('ICE', fontsize=11, fontweight='bold')
    ax3.set_title('Integral of Control Effort (ICE)', fontweight='bold', fontsize=12)
    ax3.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.3f}', ha='center', va='bottom', fontsize=9)

    # 4. Mean Error Comparison
    ax4 = plt.subplot(2, 3, 4)
    means = [controller_stats[c]['mean_error']['mean'] for c in controllers]
    stds = [controller_stats[c]['mean_error']['std'] for c in controllers]
    bars = ax4.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax4.set_ylabel('Mean Error (m)', fontsize=11, fontweight='bold')
    ax4.set_title('Average Tracking Error', fontweight='bold', fontsize=12)
    ax4.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.4f}', ha='center', va='bottom', fontsize=9)

    # 5. Max Error Comparison
    ax5 = plt.subplot(2, 3, 5)
    means = [controller_stats[c]['max_error']['mean'] for c in controllers]
    stds = [controller_stats[c]['max_error']['std'] for c in controllers]
    bars = ax5.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax5.set_ylabel('Max Error (m)', fontsize=11, fontweight='bold')
    ax5.set_title('Maximum Tracking Error', fontweight='bold', fontsize=12)
    ax5.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax5.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.4f}', ha='center', va='bottom', fontsize=9)

    # 6. Overall Performance Score (normalized)
    ax6 = plt.subplot(2, 3, 6)
    # Normalize metrics (lower is better, so invert)
    metrics_to_score = ['IAE', 'ISE', 'ICE', 'mean_error', 'max_error']
    scores = []

    for controller in controllers:
        score = 0
        for metric in metrics_to_score:
            values = [controller_stats[c][metric]['mean'] for c in controllers]
            normalized = (max(values) - controller_stats[controller][metric]['mean']) / (max(values) - min(values) + 1e-10)
            score += normalized
        scores.append(score / len(metrics_to_score) * 100)  # Convert to percentage

    bars = ax6.bar(controllers, scores, color=colors, edgecolor='black',
                   linewidth=1.5, alpha=0.8)
    ax6.set_ylabel('Performance Score (%)', fontsize=11, fontweight='bold')
    ax6.set_title('Overall Performance Score\n(Higher is Better)',
                  fontweight='bold', fontsize=12)
    ax6.set_ylim([0, 100])
    ax6.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax6.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.1f}%', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()

    # Save comparison figure
    output_file = os.path.join(output_dir, 'controller_comparison_summary.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"✓ Summary comparison chart saved to: {output_file}")

    # Generate detailed statistics table
    generate_statistics_table(controller_stats, output_dir)

    return output_file, controller_stats


def generate_statistics_table(controller_stats, output_dir):
    """
    Generate a detailed statistics table as a figure.

    Args:
        controller_stats (dict): Controller statistics dictionary
        output_dir (str): Directory to save output
    """
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.axis('tight')
    ax.axis('off')

    # Prepare table data
    controllers = list(controller_stats.keys())
    metrics = ['IAE', 'ISE', 'ICE', 'mean_error', 'max_error', 'final_error']
    metric_names = ['IAE (m·s)', 'ISE (m²·s)', 'ICE', 'Mean Error (m)',
                    'Max Error (m)', 'Final Error (m)']

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
        best_idx = means.index(min(means))  # Lower is better
        table[(i, best_idx + 1)].set_facecolor('#90EE90')
        table[(i, best_idx + 1)].set_text_props(weight='bold')

    plt.title('Controller Performance Statistics Summary\n(Best values highlighted in green)',
              fontsize=14, fontweight='bold', pad=20)

    output_file = os.path.join(output_dir, 'controller_statistics_table.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"✓ Statistics table saved to: {output_file}")


def generate_all_charts(directory_path, output_dir='charts', max_individual=10):
    """
    Generate all performance charts.

    Args:
        directory_path (str): Path to directory containing CSV files
        output_dir (str): Directory to save output charts
        max_individual (int): Maximum number of individual charts to generate per controller
    """
    print("="*80)
    print("GENERATING PERFORMANCE CHARTS")
    print("="*80 + "\n")

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Get all CSV files
    csv_files = sorted(glob.glob(os.path.join(directory_path, '*.csv')))

    if not csv_files:
        print(f"No CSV files found in {directory_path}")
        return

    print(f"Found {len(csv_files)} CSV files\n")

    # Generate summary comparison first
    print("Generating summary comparison charts...")
    generate_summary_comparison(directory_path, output_dir)
    print()

    # Generate individual charts (limited per controller)
    print(f"Generating individual controller charts (max {max_individual} per controller)...")

    controller_counts = {}
    individual_count = 0

    for csv_file in csv_files:
        filename = os.path.basename(csv_file)
        controller_type = filename.split('_')[0]

        if controller_type not in controller_counts:
            controller_counts[controller_type] = 0

        if controller_counts[controller_type] < max_individual:
            try:
                output_file = plot_individual_controller(csv_file, output_dir)
                print(f"  ✓ {filename}")
                controller_counts[controller_type] += 1
                individual_count += 1
            except Exception as e:
                print(f"  ✗ Error processing {filename}: {e}")

    print(f"\n{'='*80}")
    print(f"CHART GENERATION COMPLETE")
    print(f"{'='*80}")
    print(f"Total individual charts generated: {individual_count}")
    print(f"Summary charts generated: 2")
    print(f"Output directory: {output_dir}/")
    print(f"{'='*80}\n")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description='Generate performance visualization charts for controller analysis'
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
        default='charts',
        help='Output directory for charts (default: charts)'
    )
    parser.add_argument(
        '--max-individual',
        type=int,
        default=10,
        help='Maximum individual charts per controller (default: 10)'
    )
    parser.add_argument(
        '--single',
        type=str,
        help='Generate chart for a single CSV file'
    )
    parser.add_argument(
        '--summary-only',
        action='store_true',
        help='Generate only summary comparison charts'
    )

    args = parser.parse_args()

    if args.single:
        # Generate chart for single file
        print(f"Generating chart for: {args.single}\n")
        output_file = plot_individual_controller(args.single, args.output)
        print(f"\n✓ Chart saved to: {output_file}")
    elif args.summary_only:
        # Generate only summary charts
        print("Generating summary comparison charts only...\n")
        generate_summary_comparison(args.directory, args.output)
    else:
        # Generate all charts
        generate_all_charts(args.directory, args.output, args.max_individual)
