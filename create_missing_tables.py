#!/usr/bin/env python3
"""
Create placeholder tables for trajectory types with no data.
"""

import matplotlib.pyplot as plt
import os

def create_no_data_table(trajectory_type, output_dir='charts'):
    """Create a placeholder image for trajectories with no data."""

    fig, ax = plt.subplots(figsize=(14, 10))
    ax.axis('off')

    traj_names = {
        'recta': 'Straight Line (Recta)',
        'cuadrada': 'Square (Cuadrada)'
    }

    traj_display = traj_names.get(trajectory_type, trajectory_type.title())

    # Add title
    title_text = f'Controller Performance Statistics - {traj_display} Trajectory'
    ax.text(0.5, 0.7, title_text,
            fontsize=16, fontweight='bold', ha='center', va='center',
            transform=ax.transAxes)

    # Add main message
    message = 'No Data Available'
    ax.text(0.5, 0.5, message,
            fontsize=24, fontweight='bold', ha='center', va='center',
            color='#666666', transform=ax.transAxes)

    # Add explanation
    explanation = f'No experimental data found for {traj_display} trajectory.\nPlease run experiments with this trajectory type and regenerate statistics.'
    ax.text(0.5, 0.3, explanation,
            fontsize=12, ha='center', va='center',
            color='#888888', transform=ax.transAxes,
            style='italic')

    # Add a border
    rect = plt.Rectangle((0.1, 0.2), 0.8, 0.6, fill=False,
                         edgecolor='#CCCCCC', linewidth=2,
                         transform=ax.transAxes)
    ax.add_patch(rect)

    output_file = os.path.join(output_dir, f'controller_statistics_{trajectory_type}.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()

    print(f"✓ Placeholder created: {output_file}")
    return output_file

def main():
    print("Creating placeholder tables for missing trajectory types...\n")

    os.makedirs('charts', exist_ok=True)

    # Create placeholders for missing trajectories
    create_no_data_table('recta')
    create_no_data_table('cuadrada')

    print("\nPlaceholders created successfully!")

if __name__ == '__main__':
    main()
