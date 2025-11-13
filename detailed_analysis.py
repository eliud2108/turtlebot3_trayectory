#!/usr/bin/env python3
"""
Análisis detallado y exhaustivo de controladores de trayectoria
Incluye análisis por fases, estadísticas avanzadas y comparaciones
"""

import pandas as pd
import numpy as np
import glob
import os
from pathlib import Path

def detect_trajectory_phase(ref_x, ref_y, threshold=1.7):
    """Detecta si el punto está en fase recta o arco"""
    if ref_x < threshold:
        return 'recta'
    else:
        return 'arco'

def analyze_single_run(csv_file):
    """Analiza una ejecución individual con detalle por fases"""
    df = pd.read_csv(csv_file)

    # Detectar fases
    df['phase'] = df.apply(lambda row: detect_trajectory_phase(row['ref_x'], row['ref_y']), axis=1)

    # Análisis general
    general_stats = {
        'file': os.path.basename(csv_file),
        'total_time': df['t'].iloc[-1] - df['t'].iloc[0],
        'total_points': len(df),
        'sampling_rate': len(df) / (df['t'].iloc[-1] - df['t'].iloc[0]),
    }

    # Análisis por fase
    phase_stats = {}
    for phase in ['recta', 'arco']:
        phase_df = df[df['phase'] == phase]
        if len(phase_df) > 0:
            phase_stats[phase] = {
                'duration': phase_df['t'].iloc[-1] - phase_df['t'].iloc[0],
                'points': len(phase_df),
                'mean_error': phase_df['err'].abs().mean(),
                'max_error': phase_df['err'].abs().max(),
                'std_error': phase_df['err'].abs().std(),
                'mean_v': phase_df['v'].abs().mean(),
                'mean_w': phase_df['w'].abs().mean(),
                'max_v': phase_df['v'].abs().max(),
                'max_w': phase_df['w'].abs().max(),
            }

    # Análisis de velocidades
    velocity_stats = {
        'v_mean': df['v'].abs().mean(),
        'v_std': df['v'].abs().std(),
        'v_max': df['v'].abs().max(),
        'w_mean': df['w'].abs().mean(),
        'w_std': df['w'].abs().std(),
        'w_max': df['w'].abs().max(),
    }

    # Análisis de error
    error_stats = {
        'err_mean': df['err'].abs().mean(),
        'err_std': df['err'].abs().std(),
        'err_max': df['err'].abs().max(),
        'err_min': df['err'].abs().min(),
        'err_median': df['err'].abs().median(),
        'err_q25': df['err'].abs().quantile(0.25),
        'err_q75': df['err'].abs().quantile(0.75),
        'err_q95': df['err'].abs().quantile(0.95),
        'err_final': abs(df['err'].iloc[-1]),
    }

    # Análisis de eficiencia
    efficiency_stats = {
        'distance_traveled': np.trapz(df['v'].abs().values, df['t'].values),
        'energy_consumed': np.trapz(df['v'].values**2 + df['w'].values**2, df['t'].values),
        'avg_power': (df['v'].values**2 + df['w'].values**2).mean(),
    }

    # Contar cuántos puntos tienen error > umbral
    error_distribution = {
        'err_gt_0.01': (df['err'].abs() > 0.01).sum(),
        'err_gt_0.02': (df['err'].abs() > 0.02).sum(),
        'err_gt_0.03': (df['err'].abs() > 0.03).sum(),
        'err_gt_0.05': (df['err'].abs() > 0.05).sum(),
    }

    return {
        'general': general_stats,
        'phases': phase_stats,
        'velocities': velocity_stats,
        'errors': error_stats,
        'efficiency': efficiency_stats,
        'error_distribution': error_distribution
    }

def analyze_all_controllers(directory='tb3_runs'):
    """Analiza todos los controladores y agrupa resultados"""

    # Buscar todos los CSV
    csv_files = glob.glob(os.path.join(directory, '*.csv'))
    csv_files = [f for f in csv_files if 'summary' not in f]

    results = {}
    for csv_file in sorted(csv_files):
        basename = os.path.basename(csv_file)
        # Detectar tipo de controlador
        if basename.startswith('pid'):
            controller = 'pid'
        elif basename.startswith('lyapunov'):
            controller = 'lyapunov'
        elif basename.startswith('mpc'):
            controller = 'mpc'
        elif basename.startswith('pure'):
            controller = 'pure'
        else:
            continue

        try:
            stats = analyze_single_run(csv_file)

            if controller not in results:
                results[controller] = {
                    'runs': [],
                    'files': []
                }

            results[controller]['runs'].append(stats)
            results[controller]['files'].append(basename)
        except Exception as e:
            print(f"Error processing {csv_file}: {e}")

    return results

def compute_aggregated_stats(results):
    """Calcula estadísticas agregadas por controlador"""
    aggregated = {}

    for controller, data in results.items():
        runs = data['runs']
        n_runs = len(runs)

        # Agregar estadísticas generales
        total_times = [r['general']['total_time'] for r in runs]

        # Agregar estadísticas de error
        mean_errors = [r['errors']['err_mean'] for r in runs]
        max_errors = [r['errors']['err_max'] for r in runs]
        std_errors = [r['errors']['err_std'] for r in runs]
        final_errors = [r['errors']['err_final'] for r in runs]
        median_errors = [r['errors']['err_median'] for r in runs]
        q95_errors = [r['errors']['err_q95'] for r in runs]

        # Agregar estadísticas de velocidad
        v_means = [r['velocities']['v_mean'] for r in runs]
        w_means = [r['velocities']['w_mean'] for r in runs]
        v_maxs = [r['velocities']['v_max'] for r in runs]
        w_maxs = [r['velocities']['w_max'] for r in runs]

        # Agregar estadísticas de eficiencia
        distances = [r['efficiency']['distance_traveled'] for r in runs]
        energies = [r['efficiency']['energy_consumed'] for r in runs]
        powers = [r['efficiency']['avg_power'] for r in runs]

        # Estadísticas por fase
        phase_stats = {}
        for phase in ['recta', 'arco']:
            phase_data = []
            for run in runs:
                if phase in run['phases']:
                    phase_data.append(run['phases'][phase])

            if phase_data:
                phase_stats[phase] = {
                    'n_runs': len(phase_data),
                    'duration_mean': np.mean([p['duration'] for p in phase_data]),
                    'duration_std': np.std([p['duration'] for p in phase_data]),
                    'error_mean': np.mean([p['mean_error'] for p in phase_data]),
                    'error_max': np.mean([p['max_error'] for p in phase_data]),
                    'error_std': np.mean([p['std_error'] for p in phase_data]),
                    'v_mean': np.mean([p['mean_v'] for p in phase_data]),
                    'w_mean': np.mean([p['mean_w'] for p in phase_data]),
                }

        aggregated[controller] = {
            'n_runs': n_runs,
            'time': {
                'mean': np.mean(total_times),
                'std': np.std(total_times),
                'min': np.min(total_times),
                'max': np.max(total_times),
                'median': np.median(total_times),
            },
            'error': {
                'mean_avg': np.mean(mean_errors),
                'mean_std': np.std(mean_errors),
                'max_avg': np.mean(max_errors),
                'max_std': np.std(max_errors),
                'final_avg': np.mean(final_errors),
                'final_std': np.std(final_errors),
                'median_avg': np.mean(median_errors),
                'q95_avg': np.mean(q95_errors),
                'std_avg': np.mean(std_errors),
            },
            'velocity': {
                'v_mean': np.mean(v_means),
                'v_mean_std': np.std(v_means),
                'v_max': np.mean(v_maxs),
                'w_mean': np.mean(w_means),
                'w_mean_std': np.std(w_means),
                'w_max': np.mean(w_maxs),
            },
            'efficiency': {
                'distance_mean': np.mean(distances),
                'distance_std': np.std(distances),
                'energy_mean': np.mean(energies),
                'energy_std': np.std(energies),
                'power_mean': np.mean(powers),
                'power_std': np.std(powers),
            },
            'phases': phase_stats
        }

    return aggregated

def print_detailed_report(aggregated):
    """Imprime reporte detallado"""
    print("="*80)
    print("ANÁLISIS DETALLADO DE CONTROLADORES")
    print("="*80)

    for controller, stats in sorted(aggregated.items()):
        print(f"\n{'='*80}")
        print(f"{controller.upper()} - {stats['n_runs']} ejecuciones")
        print(f"{'='*80}")

        print(f"\nTIEMPO DE EJECUCIÓN:")
        print(f"  Media: {stats['time']['mean']:.3f} ± {stats['time']['std']:.3f} s")
        print(f"  Rango: [{stats['time']['min']:.3f}, {stats['time']['max']:.3f}] s")
        print(f"  Mediana: {stats['time']['median']:.3f} s")

        print(f"\nERRORES:")
        print(f"  Error promedio: {stats['error']['mean_avg']*1000:.3f} ± {stats['error']['mean_std']*1000:.3f} mm")
        print(f"  Error máximo: {stats['error']['max_avg']*1000:.3f} ± {stats['error']['max_std']*1000:.3f} mm")
        print(f"  Error final: {stats['error']['final_avg']*1000:.3f} ± {stats['error']['final_std']*1000:.3f} mm")
        print(f"  Mediana de error: {stats['error']['median_avg']*1000:.3f} mm")
        print(f"  Percentil 95: {stats['error']['q95_avg']*1000:.3f} mm")

        print(f"\nVELOCIDADES:")
        print(f"  Vel. lineal media: {stats['velocity']['v_mean']:.4f} ± {stats['velocity']['v_mean_std']:.4f} m/s")
        print(f"  Vel. lineal máx: {stats['velocity']['v_max']:.4f} m/s")
        print(f"  Vel. angular media: {stats['velocity']['w_mean']:.4f} ± {stats['velocity']['w_mean_std']:.4f} rad/s")
        print(f"  Vel. angular máx: {stats['velocity']['w_max']:.4f} rad/s")

        print(f"\nEFICIENCIA:")
        print(f"  Distancia recorrida: {stats['efficiency']['distance_mean']:.3f} ± {stats['efficiency']['distance_std']:.3f} m")
        print(f"  Energía consumida: {stats['efficiency']['energy_mean']:.3f} ± {stats['efficiency']['energy_std']:.3f}")
        print(f"  Potencia promedio: {stats['efficiency']['power_mean']:.4f} ± {stats['efficiency']['power_std']:.4f}")

        if 'phases' in stats and stats['phases']:
            print(f"\nANÁLISIS POR FASES:")
            for phase, pstats in stats['phases'].items():
                print(f"  {phase.upper()}:")
                print(f"    Duración: {pstats['duration_mean']:.3f} ± {pstats['duration_std']:.3f} s")
                print(f"    Error promedio: {pstats['error_mean']*1000:.3f} mm")
                print(f"    Error máximo: {pstats['error_max']*1000:.3f} mm")
                print(f"    Vel. lineal: {pstats['v_mean']:.4f} m/s")
                print(f"    Vel. angular: {pstats['w_mean']:.4f} rad/s")

def generate_comparison_table(aggregated):
    """Genera tabla de comparación"""
    print("\n\n" + "="*80)
    print("TABLA COMPARATIVA GENERAL")
    print("="*80)

    print(f"\n{'Métrica':<30} {'PID':<15} {'LYAPUNOV':<15} {'MPC':<15} {'PURE':<15}")
    print("-"*90)

    # Tiempo
    print(f"{'Tiempo (s)':<30} ", end="")
    for ctrl in ['pid', 'lyapunov', 'mpc', 'pure']:
        if ctrl in aggregated:
            print(f"{aggregated[ctrl]['time']['mean']:>7.2f}±{aggregated[ctrl]['time']['std']:>5.2f}  ", end="")
    print()

    # Error promedio
    print(f"{'Error promedio (mm)':<30} ", end="")
    for ctrl in ['pid', 'lyapunov', 'mpc', 'pure']:
        if ctrl in aggregated:
            print(f"{aggregated[ctrl]['error']['mean_avg']*1000:>7.2f}±{aggregated[ctrl]['error']['mean_std']*1000:>5.2f}  ", end="")
    print()

    # Error máximo
    print(f"{'Error máximo (mm)':<30} ", end="")
    for ctrl in ['pid', 'lyapunov', 'mpc', 'pure']:
        if ctrl in aggregated:
            print(f"{aggregated[ctrl]['error']['max_avg']*1000:>7.2f}±{aggregated[ctrl]['error']['max_std']*1000:>5.2f}  ", end="")
    print()

    # Error final
    print(f"{'Error final (mm)':<30} ", end="")
    for ctrl in ['pid', 'lyapunov', 'mpc', 'pure']:
        if ctrl in aggregated:
            print(f"{aggregated[ctrl]['error']['final_avg']*1000:>7.2f}±{aggregated[ctrl]['error']['final_std']*1000:>5.2f}  ", end="")
    print()

    # Energía
    print(f"{'Energía consumida':<30} ", end="")
    for ctrl in ['pid', 'lyapunov', 'mpc', 'pure']:
        if ctrl in aggregated:
            print(f"{aggregated[ctrl]['efficiency']['energy_mean']:>7.2f}±{aggregated[ctrl]['efficiency']['energy_std']:>5.2f}  ", end="")
    print()

    # Distancia
    print(f"{'Distancia (m)':<30} ", end="")
    for ctrl in ['pid', 'lyapunov', 'mpc', 'pure']:
        if ctrl in aggregated:
            print(f"{aggregated[ctrl]['efficiency']['distance_mean']:>7.2f}±{aggregated[ctrl]['efficiency']['distance_std']:>5.2f}  ", end="")
    print()

if __name__ == '__main__':
    print("Analizando todos los controladores...")
    results = analyze_all_controllers()

    print(f"\nControladores encontrados: {list(results.keys())}")
    print(f"Total de ejecuciones: {sum(len(r['runs']) for r in results.values())}")

    print("\nCalculando estadísticas agregadas...")
    aggregated = compute_aggregated_stats(results)

    print_detailed_report(aggregated)
    generate_comparison_table(aggregated)

    print("\n\n" + "="*80)
    print("ANÁLISIS COMPLETADO")
    print("="*80)
