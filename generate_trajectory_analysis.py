#!/usr/bin/env python3
"""
Script to generate comprehensive performance analysis by trajectory type.

This script creates:
1. Performance comparison charts for each trajectory type (recta, compuesta, cuadrada)
2. Controller performance across different trajectories
3. Detailed statistics tables for each trajectory type

Author: Claude AI
Date: 2025-11-12
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
from pathlib import Path
import seaborn as sns

# Set style
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")


def identify_trajectory_type(filename):
    """
    Identify trajectory type from filename.

    Args:
        filename (str): CSV filename

    Returns:
        str: Trajectory type ('recta', 'compuesta', 'cuadrada')
    """
    filename_lower = filename.lower()

    if 'recta' in filename_lower:
        return 'recta'
    elif 'compuesta' in filename_lower:
        return 'compuesta'
    else:
        # Files without prefix are from the original square trajectory
        return 'cuadrada'


def identify_controller_type(filename):
    """
    Identify controller type from filename.

    Args:
        filename (str): CSV filename

    Returns:
        str: Controller type ('pid', 'lyapunov', 'mpc', 'pure')
    """
    filename_lower = filename.lower()

    if 'pid' in filename_lower:
        return 'pid'
    elif 'lyapunov' in filename_lower:
        return 'lyapunov'
    elif 'mpc' in filename_lower:
        return 'mpc'
    elif 'pure' in filename_lower:
        return 'pure'
    else:
        return 'unknown'


def calculate_metrics_from_file(csv_file):
    """
    Calculate performance metrics from a CSV file.

    Args:
        csv_file (str): Path to CSV file

    Returns:
        dict: Dictionary with all calculated metrics
    """
    try:
        df = pd.read_csv(csv_file)

        # Extract data
        t = df['t'].values
        err = df['err'].values
        v = df['v'].values
        w = df['w'].values

        # Calculate metrics
        metrics = {
            'IAE': np.trapezoid(np.abs(err), t),
            'ITAE': np.trapezoid(t * np.abs(err), t),
            'ISE': np.trapezoid(err**2, t),
            'ICE': np.trapezoid(v**2 + w**2, t),
            'max_error': np.max(np.abs(err)),
            'mean_error': np.mean(np.abs(err)),
            'final_error': np.abs(err[-1]),
            'total_time': t[-1] - t[0],
            'num_samples': len(t)
        }

        return metrics

    except Exception as e:
        print(f"Error processing {csv_file}: {e}")
        return None


def analyze_by_trajectory(directory_path):
    """
    Analyze all CSV files grouped by trajectory type.

    Args:
        directory_path (str): Path to directory containing CSV files

    Returns:
        dict: Nested dictionary with trajectory -> controller -> metrics
    """
    csv_files = glob.glob(os.path.join(directory_path, '*.csv'))

    # Filter out summary files
    csv_files = [f for f in csv_files if 'summary' not in f.lower()]

    print(f"Found {len(csv_files)} CSV files")

    # Organize data by trajectory and controller
    data = {
        'recta': {'pid': [], 'lyapunov': [], 'mpc': [], 'pure': []},
        'compuesta': {'pid': [], 'lyapunov': [], 'mpc': [], 'pure': []},
        'cuadrada': {'pid': [], 'lyapunov': [], 'mpc': [], 'pure': []}
    }

    for csv_file in csv_files:
        filename = os.path.basename(csv_file)
        trajectory_type = identify_trajectory_type(filename)
        controller_type = identify_controller_type(filename)

        if controller_type == 'unknown':
            continue

        metrics = calculate_metrics_from_file(csv_file)
        if metrics:
            data[trajectory_type][controller_type].append(metrics)

    # Calculate statistics
    stats = {}
    for traj_type in data.keys():
        stats[traj_type] = {}
        for controller in data[traj_type].keys():
            runs = data[traj_type][controller]
            if len(runs) > 0:
                stats[traj_type][controller] = {
                    metric: {
                        'mean': np.mean([run[metric] for run in runs]),
                        'std': np.std([run[metric] for run in runs]),
                        'min': np.min([run[metric] for run in runs]),
                        'max': np.max([run[metric] for run in runs]),
                        'count': len(runs)
                    }
                    for metric in runs[0].keys()
                }

    return stats


def plot_trajectory_comparison(stats, trajectory_type, output_dir='charts'):
    """
    Generate comparison chart for a specific trajectory type.

    Args:
        stats (dict): Statistics dictionary
        trajectory_type (str): Type of trajectory ('recta', 'compuesta', 'cuadrada')
        output_dir (str): Output directory
    """
    trajectory_stats = stats[trajectory_type]

    # Filter out controllers with no data
    controllers = [c for c in trajectory_stats.keys() if trajectory_stats[c]]

    if not controllers:
        print(f"No data for trajectory: {trajectory_type}")
        return

    # Create figure
    fig = plt.figure(figsize=(20, 14))

    trajectory_names = {
        'recta': 'Trayectoria Recta',
        'compuesta': 'Trayectoria Compuesta',
        'cuadrada': 'Trayectoria Cuadrada'
    }

    fig.suptitle(f'Comparación de Controladores - {trajectory_names[trajectory_type]}',
                 fontsize=18, fontweight='bold')

    colors = plt.cm.Set3(np.linspace(0, 1, len(controllers)))

    # 1. IAE
    ax1 = plt.subplot(3, 3, 1)
    means = [trajectory_stats[c]['IAE']['mean'] for c in controllers]
    stds = [trajectory_stats[c]['IAE']['std'] for c in controllers]
    bars = ax1.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax1.set_ylabel('IAE (m·s)', fontsize=11, fontweight='bold')
    ax1.set_title('Integral of Absolute Error (IAE)', fontweight='bold')
    ax1.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.3f}', ha='center', va='bottom', fontsize=9)

    # 2. ITAE
    ax2 = plt.subplot(3, 3, 2)
    means = [trajectory_stats[c]['ITAE']['mean'] for c in controllers]
    stds = [trajectory_stats[c]['ITAE']['std'] for c in controllers]
    bars = ax2.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax2.set_ylabel('ITAE (m·s²)', fontsize=11, fontweight='bold')
    ax2.set_title('Integral of Time-weighted Absolute Error (ITAE)', fontweight='bold')
    ax2.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.3f}', ha='center', va='bottom', fontsize=9)

    # 3. ISE
    ax3 = plt.subplot(3, 3, 3)
    means = [trajectory_stats[c]['ISE']['mean'] for c in controllers]
    stds = [trajectory_stats[c]['ISE']['std'] for c in controllers]
    bars = ax3.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax3.set_ylabel('ISE (m²·s)', fontsize=11, fontweight='bold')
    ax3.set_title('Integral of Squared Error (ISE)', fontweight='bold')
    ax3.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.3f}', ha='center', va='bottom', fontsize=9)

    # 4. ICE
    ax4 = plt.subplot(3, 3, 4)
    means = [trajectory_stats[c]['ICE']['mean'] for c in controllers]
    stds = [trajectory_stats[c]['ICE']['std'] for c in controllers]
    bars = ax4.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax4.set_ylabel('ICE', fontsize=11, fontweight='bold')
    ax4.set_title('Integral of Control Effort (ICE)', fontweight='bold')
    ax4.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.3f}', ha='center', va='bottom', fontsize=9)

    # 5. Mean Error
    ax5 = plt.subplot(3, 3, 5)
    means = [trajectory_stats[c]['mean_error']['mean'] for c in controllers]
    stds = [trajectory_stats[c]['mean_error']['std'] for c in controllers]
    bars = ax5.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax5.set_ylabel('Error Promedio (m)', fontsize=11, fontweight='bold')
    ax5.set_title('Error de Seguimiento Promedio', fontweight='bold')
    ax5.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax5.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.4f}', ha='center', va='bottom', fontsize=9)

    # 6. Max Error
    ax6 = plt.subplot(3, 3, 6)
    means = [trajectory_stats[c]['max_error']['mean'] for c in controllers]
    stds = [trajectory_stats[c]['max_error']['std'] for c in controllers]
    bars = ax6.bar(controllers, means, yerr=stds, capsize=5, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax6.set_ylabel('Error Máximo (m)', fontsize=11, fontweight='bold')
    ax6.set_title('Error de Seguimiento Máximo', fontweight='bold')
    ax6.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax6.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.4f}', ha='center', va='bottom', fontsize=9)

    # 7. Sample count
    ax7 = plt.subplot(3, 3, 7)
    counts = [trajectory_stats[c]['num_samples']['count'] for c in controllers]
    bars = ax7.bar(controllers, counts, color=colors,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax7.set_ylabel('Número de Corridas', fontsize=11, fontweight='bold')
    ax7.set_title('Cantidad de Datos Analizados', fontweight='bold')
    ax7.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax7.text(bar.get_x() + bar.get_width()/2., height,
                f'{int(height)}', ha='center', va='bottom', fontsize=9)

    # 8. Performance Score
    ax8 = plt.subplot(3, 3, 8)
    metrics_to_score = ['IAE', 'ITAE', 'ISE', 'ICE', 'mean_error', 'max_error']
    scores = []

    for controller in controllers:
        score = 0
        for metric in metrics_to_score:
            values = [trajectory_stats[c][metric]['mean'] for c in controllers]
            normalized = (max(values) - trajectory_stats[controller][metric]['mean']) / (max(values) - min(values) + 1e-10)
            score += normalized
        scores.append(score / len(metrics_to_score) * 100)

    bars = ax8.bar(controllers, scores, color=colors, edgecolor='black',
                   linewidth=1.5, alpha=0.8)
    ax8.set_ylabel('Puntuación de Desempeño (%)', fontsize=11, fontweight='bold')
    ax8.set_title('Puntuación General de Desempeño\n(Mayor es Mejor)', fontweight='bold')
    ax8.set_ylim([0, 100])
    ax8.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        height = bar.get_height()
        ax8.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.1f}%', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()

    # Save figure
    output_file = os.path.join(output_dir, f'comparison_{trajectory_type}.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"✓ Saved: {output_file}")

    return output_file


def plot_cross_trajectory_comparison(stats, output_dir='charts'):
    """
    Generate comparison chart showing each controller's performance across trajectories.

    Args:
        stats (dict): Statistics dictionary
        output_dir (str): Output directory
    """
    # Get all controllers
    all_controllers = set()
    for traj in stats.values():
        all_controllers.update([c for c in traj.keys() if traj[c]])

    controllers = sorted(list(all_controllers))
    trajectories = ['recta', 'compuesta', 'cuadrada']
    trajectory_labels = ['Recta', 'Compuesta', 'Cuadrada']

    # Create figure
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle('Comparación de Controladores Across Trayectorias',
                 fontsize=18, fontweight='bold')

    metrics_to_plot = [
        ('IAE', 'IAE (m·s)', '%.3f'),
        ('ITAE', 'ITAE (m·s²)', '%.3f'),
        ('ISE', 'ISE (m²·s)', '%.3f'),
        ('ICE', 'ICE', '%.3f'),
        ('mean_error', 'Error Promedio (m)', '%.4f'),
        ('max_error', 'Error Máximo (m)', '%.4f')
    ]

    for idx, (metric, ylabel, fmt) in enumerate(metrics_to_plot, 1):
        ax = plt.subplot(2, 3, idx)

        x = np.arange(len(trajectories))
        width = 0.2

        for i, controller in enumerate(controllers):
            values = []
            for traj in trajectories:
                if controller in stats[traj] and stats[traj][controller]:
                    values.append(stats[traj][controller][metric]['mean'])
                else:
                    values.append(0)

            offset = (i - len(controllers)/2 + 0.5) * width
            bars = ax.bar(x + offset, values, width, label=controller.upper())

            # Add value labels
            for bar in bars:
                height = bar.get_height()
                if height > 0:
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           fmt % height, ha='center', va='bottom', fontsize=8)

        ax.set_ylabel(ylabel, fontsize=10, fontweight='bold')
        ax.set_title(ylabel.split('(')[0].strip(), fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(trajectory_labels)
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()

    # Save figure
    output_file = os.path.join(output_dir, 'comparison_cross_trajectory.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"✓ Saved: {output_file}")

    return output_file


def generate_statistics_table(stats, trajectory_type, output_dir='charts'):
    """
    Generate statistics table for a specific trajectory.

    Args:
        stats (dict): Statistics dictionary
        trajectory_type (str): Trajectory type
        output_dir (str): Output directory
    """
    trajectory_stats = stats[trajectory_type]
    controllers = [c for c in trajectory_stats.keys() if trajectory_stats[c]]

    if not controllers:
        return

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.axis('tight')
    ax.axis('off')

    trajectory_names = {
        'recta': 'Trayectoria Recta',
        'compuesta': 'Trayectoria Compuesta',
        'cuadrada': 'Trayectoria Cuadrada'
    }

    metrics = ['IAE', 'ITAE', 'ISE', 'ICE', 'mean_error', 'max_error', 'final_error']
    metric_names = ['IAE (m·s)', 'ITAE (m·s²)', 'ISE (m²·s)', 'ICE',
                    'Error Promedio (m)', 'Error Máximo (m)', 'Error Final (m)']

    table_data = []
    table_data.append(['Métrica'] + [c.upper() for c in controllers])

    for metric, name in zip(metrics, metric_names):
        row = [name]
        for controller in controllers:
            mean_val = trajectory_stats[controller][metric]['mean']
            std_val = trajectory_stats[controller][metric]['std']
            row.append(f'{mean_val:.4f} ± {std_val:.4f}')
        table_data.append(row)

    # Create table
    table = ax.table(cellText=table_data, cellLoc='center', loc='center',
                     colWidths=[0.25] + [0.25] * len(controllers))

    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2)

    # Style header
    for i in range(len(controllers) + 1):
        table[(0, i)].set_facecolor('#4CAF50')
        table[(0, i)].set_text_props(weight='bold', color='white')

    # Style metric names
    for i in range(1, len(metrics) + 1):
        table[(i, 0)].set_facecolor('#E8E8E8')
        table[(i, 0)].set_text_props(weight='bold')

    # Highlight best values
    for i, metric in enumerate(metrics, start=1):
        means = [trajectory_stats[c][metric]['mean'] for c in controllers]
        best_idx = means.index(min(means))
        table[(i, best_idx + 1)].set_facecolor('#90EE90')
        table[(i, best_idx + 1)].set_text_props(weight='bold')

    plt.title(f'Estadísticas de Desempeño - {trajectory_names[trajectory_type]}\n(Mejores valores resaltados en verde)',
              fontsize=14, fontweight='bold', pad=20)

    output_file = os.path.join(output_dir, f'statistics_{trajectory_type}.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"✓ Saved: {output_file}")


def generate_all_trajectory_analysis(directory_path='tb3_runs', output_dir='charts'):
    """
    Generate complete trajectory analysis.

    Args:
        directory_path (str): Directory with CSV files
        output_dir (str): Output directory
    """
    print("="*80)
    print("ANÁLISIS DE DESEMPEÑO POR TIPO DE TRAYECTORIA")
    print("="*80 + "\n")

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Analyze data
    print("Analizando archivos CSV...")
    stats = analyze_by_trajectory(directory_path)

    # Print summary
    print("\nResumen de datos:")
    for traj_type in ['recta', 'compuesta', 'cuadrada']:
        print(f"\n{traj_type.upper()}:")
        for controller in ['pid', 'lyapunov', 'mpc', 'pure']:
            if controller in stats[traj_type] and stats[traj_type][controller]:
                count = stats[traj_type][controller]['IAE']['count']
                print(f"  {controller.upper()}: {count} corridas")

    print("\n" + "="*80)
    print("Generando gráficas...")
    print("="*80 + "\n")

    # Generate trajectory-specific comparisons
    for traj_type in ['recta', 'compuesta', 'cuadrada']:
        print(f"Generando gráficas para trayectoria: {traj_type}")
        plot_trajectory_comparison(stats, traj_type, output_dir)
        generate_statistics_table(stats, traj_type, output_dir)

    # Generate cross-trajectory comparison
    print("\nGenerando comparación cross-trayectorias...")
    plot_cross_trajectory_comparison(stats, output_dir)

    print("\n" + "="*80)
    print("ANÁLISIS COMPLETO")
    print("="*80)
    print(f"Directorio de salida: {output_dir}/")
    print("Gráficas generadas:")
    print("  - comparison_recta.png")
    print("  - comparison_compuesta.png")
    print("  - comparison_cuadrada.png")
    print("  - comparison_cross_trajectory.png")
    print("  - statistics_recta.png")
    print("  - statistics_compuesta.png")
    print("  - statistics_cuadrada.png")
    print("="*80 + "\n")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description='Generate trajectory-based performance analysis'
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

    args = parser.parse_args()

    generate_all_trajectory_analysis(args.directory, args.output)
