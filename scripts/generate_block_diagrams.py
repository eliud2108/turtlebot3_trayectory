#!/usr/bin/env python3
"""
Script para generar diagramas de bloques de los controladores del TurtleBot3.
Genera diagramas visuales para PID, Lyapunov, Pure Pursuit y MPC.

Autor: TurtleBot3 Control Team
Fecha: 2025-12-01
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
import numpy as np
import os


class BlockDiagramGenerator:
    """Clase para generar diagramas de bloques de controladores."""

    def __init__(self, output_dir='../docs/diagrams'):
        """
        Inicializa el generador de diagramas.

        Args:
            output_dir: Directorio donde se guardarán los diagramas
        """
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        # Colores para los bloques
        self.colors = {
            'input': '#e1f5ff',
            'error': '#fff4e1',
            'control': '#ffe1f5',
            'calc': '#e1ffe1',
            'sat': '#ffe1e1',
            'output': '#f0e1ff'
        }

    def create_block(self, ax, x, y, width, height, text, color, style='round'):
        """
        Crea un bloque en el diagrama.

        Args:
            ax: Axes de matplotlib
            x, y: Posición del centro del bloque
            width, height: Dimensiones del bloque
            text: Texto del bloque
            color: Color de fondo
            style: Estilo del bloque ('round', 'rect', 'circle')
        """
        if style == 'round':
            box = FancyBboxPatch(
                (x - width/2, y - height/2), width, height,
                boxstyle="round,pad=0.05",
                facecolor=color, edgecolor='black', linewidth=2
            )
        elif style == 'circle':
            box = Circle((x, y), width/2, facecolor=color,
                        edgecolor='black', linewidth=2)
        else:
            box = patches.Rectangle(
                (x - width/2, y - height/2), width, height,
                facecolor=color, edgecolor='black', linewidth=2
            )

        ax.add_patch(box)
        ax.text(x, y, text, ha='center', va='center',
               fontsize=9, weight='bold', wrap=True)

    def create_arrow(self, ax, x1, y1, x2, y2, label='', style='->'):
        """
        Crea una flecha entre bloques.

        Args:
            ax: Axes de matplotlib
            x1, y1: Coordenadas de inicio
            x2, y2: Coordenadas de fin
            label: Etiqueta de la flecha
            style: Estilo de la flecha
        """
        arrow = FancyArrowPatch(
            (x1, y1), (x2, y2),
            arrowstyle=style, color='black', linewidth=2,
            mutation_scale=20, zorder=1
        )
        ax.add_patch(arrow)

        if label:
            mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
            ax.text(mid_x, mid_y + 0.1, label, ha='center',
                   fontsize=7, bbox=dict(boxstyle='round',
                   facecolor='white', alpha=0.8))

    def generate_pid_diagram(self):
        """Genera el diagrama de bloques del controlador PID."""
        fig, ax = plt.subplots(figsize=(14, 10))
        ax.set_xlim(0, 14)
        ax.set_ylim(0, 10)
        ax.axis('off')

        # Título
        ax.text(7, 9.5, 'Diagrama de Bloques - Controlador PID',
               ha='center', fontsize=16, weight='bold')

        # Entradas
        self.create_block(ax, 1, 8, 1.2, 0.6, '/odom\n(x,y,θ)',
                         self.colors['input'])
        self.create_block(ax, 1, 7, 1.2, 0.6, 'Waypoints',
                         self.colors['input'])
        self.create_block(ax, 1, 6, 1.2, 0.6, '/battery',
                         self.colors['input'])

        # Cálculo de error
        self.create_block(ax, 3.5, 7.5, 1.5, 0.8, 'Calcular\nWaypoint\nmás cercano',
                         self.colors['error'])
        self.create_block(ax, 3.5, 6.2, 1.5, 0.6, 'Error Angular\nα',
                         self.colors['error'])
        self.create_block(ax, 3.5, 5.3, 1.5, 0.6, 'Error Dist.\nρ',
                         self.colors['error'])

        # Bloques PID
        self.create_block(ax, 6, 7.5, 1.2, 0.5, 'P = kp·α',
                         self.colors['control'])
        self.create_block(ax, 6, 6.7, 1.2, 0.5, 'I = ki·∫α',
                         self.colors['control'])
        self.create_block(ax, 6, 5.9, 1.2, 0.5, 'D = kd·dα/dt',
                         self.colors['control'])

        # Suma PID
        self.create_block(ax, 7.5, 6.7, 0.6, 0.6, 'Σ',
                         self.colors['control'], style='circle')

        # Cálculo de velocidades
        self.create_block(ax, 9.5, 7.5, 1.8, 0.7, 'ω = ang_gain×\n(P+I+D)',
                         self.colors['calc'])
        self.create_block(ax, 9.5, 5.5, 1.8, 0.9, 'v = vmax×\n(1-|α|/(π/2))×\nmin(1,ρ/0.5)',
                         self.colors['calc'])

        # Saturación
        self.create_block(ax, 11.5, 7.5, 1.2, 0.6, 'Saturar ω\n[-1.3, 1.3]',
                         self.colors['sat'])
        self.create_block(ax, 11.5, 5.5, 1.2, 0.6, 'Saturar v\n[-0.18, 0.18]',
                         self.colors['sat'])

        # Salidas
        self.create_block(ax, 13, 6.5, 1.2, 0.8, '/cmd_vel\n(v, ω)',
                         self.colors['output'])
        self.create_block(ax, 13, 4.5, 1.2, 0.6, 'Logs\nCSV',
                         self.colors['output'])

        # Flechas - Entradas a cálculo de error
        self.create_arrow(ax, 1.6, 8, 2.75, 7.5)
        self.create_arrow(ax, 1.6, 7, 2.75, 7.5)

        # Flechas - Error a PID
        self.create_arrow(ax, 4.25, 7.5, 5.4, 7.5)
        self.create_arrow(ax, 4.25, 6.2, 5.4, 6.9)
        self.create_arrow(ax, 4.25, 6.2, 5.4, 6.5)
        self.create_arrow(ax, 4.25, 5.3, 9.5, 5.9, label='ρ')

        # Flechas - PID a suma
        self.create_arrow(ax, 6.6, 7.5, 7.2, 6.9)
        self.create_arrow(ax, 6.6, 6.7, 7.2, 6.7)
        self.create_arrow(ax, 6.6, 5.9, 7.2, 6.5)

        # Flechas - Suma a velocidad angular
        self.create_arrow(ax, 7.8, 6.7, 8.6, 7.3)

        # Flechas - Error a velocidad lineal
        self.create_arrow(ax, 4.25, 6.2, 8.6, 5.7, label='α')

        # Flechas - Velocidades a saturación
        self.create_arrow(ax, 10.4, 7.5, 10.9, 7.5)
        self.create_arrow(ax, 10.4, 5.5, 10.9, 5.5)

        # Flechas - Saturación a salida
        self.create_arrow(ax, 12.1, 7.5, 12.4, 6.9)
        self.create_arrow(ax, 12.1, 5.5, 12.4, 6.1)

        # Flechas - A logs
        self.create_arrow(ax, 1, 6, 12.4, 4.7)
        self.create_arrow(ax, 13, 5.7, 13, 5.1)

        # Parámetros
        param_text = (
            'Parámetros:\n'
            'kp = 2.3-2.5\n'
            'ki = 0.25-0.3\n'
            'kd = 0.18-0.2\n'
            'ang_gain = 1.5\n'
            'vmax = 0.18 m/s\n'
            'ωmax = 1.3 rad/s\n'
            'Frecuencia: 100 Hz'
        )
        ax.text(0.5, 3.5, param_text, fontsize=8,
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
               verticalalignment='top')

        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/pid_block_diagram.png',
                   dpi=300, bbox_inches='tight')
        print(f"Diagrama PID guardado en: {self.output_dir}/pid_block_diagram.png")
        plt.close()

    def generate_lyapunov_diagram(self):
        """Genera el diagrama de bloques del controlador Lyapunov."""
        fig, ax = plt.subplots(figsize=(14, 10))
        ax.set_xlim(0, 14)
        ax.set_ylim(0, 10)
        ax.axis('off')

        # Título
        ax.text(7, 9.5, 'Diagrama de Bloques - Controlador Lyapunov',
               ha='center', fontsize=16, weight='bold')

        # Entradas
        self.create_block(ax, 1, 8, 1.2, 0.6, '/odom\n(x,y,θ)',
                         self.colors['input'])
        self.create_block(ax, 1, 7, 1.2, 0.6, 'Waypoints',
                         self.colors['input'])

        # Transformación
        self.create_block(ax, 3, 7.5, 1.4, 0.8, 'Calcular\nΔx, Δy',
                         self.colors['error'])

        # Coordenadas polares
        self.create_block(ax, 5, 8.3, 1.3, 0.6, 'ρ = √(Δx²+Δy²)',
                         self.colors['error'])
        self.create_block(ax, 5, 7.3, 1.3, 0.7, 'α = atan2(Δy,Δx)\n- θ',
                         self.colors['error'])
        self.create_block(ax, 5, 6.2, 1.3, 0.6, 'β = θd - θ',
                         self.colors['error'])

        # Función de Lyapunov
        self.create_block(ax, 7.5, 7.3, 1.6, 0.8,
                         'V = ½(ρ²+α²+β²)\nEstabilidad',
                         self.colors['control'])

        # Leyes de control
        self.create_block(ax, 10, 8, 2, 0.9,
                         'v = k_rho·ρ·cos(α)\n×e^(-|α|)',
                         self.colors['calc'])
        self.create_block(ax, 10, 6.3, 2.2, 1,
                         'ω = k_alpha·α +\nk_rho·sin(α)cos(α)/ρ +\nk_beta·β',
                         self.colors['calc'])

        # Adaptación (compuesta)
        self.create_block(ax, 10, 4.5, 2, 0.8,
                         'Lookahead\nAdaptativo\n[0.18-0.35m]',
                         self.colors['calc'])

        # Saturación
        self.create_block(ax, 12.3, 7.5, 1.2, 0.6,
                         'Saturar\nv, ω',
                         self.colors['sat'])

        # Salida
        self.create_block(ax, 13, 6.5, 1.2, 0.8,
                         '/cmd_vel\n(v, ω)',
                         self.colors['output'])

        # Flechas
        self.create_arrow(ax, 1.6, 8, 2.3, 7.7)
        self.create_arrow(ax, 1.6, 7, 2.3, 7.3)
        self.create_arrow(ax, 3.7, 7.5, 4.35, 8.3)
        self.create_arrow(ax, 3.7, 7.5, 4.35, 7.3)
        self.create_arrow(ax, 3.7, 7.5, 4.35, 6.2)

        self.create_arrow(ax, 5.65, 8.3, 6.7, 7.6)
        self.create_arrow(ax, 5.65, 7.3, 6.7, 7.4)
        self.create_arrow(ax, 5.65, 6.2, 6.7, 7.0)

        self.create_arrow(ax, 9.1, 7.3, 9, 8)
        self.create_arrow(ax, 9.1, 7.3, 9, 6.6)

        self.create_arrow(ax, 11, 8, 11.7, 7.7)
        self.create_arrow(ax, 11.1, 6.3, 11.7, 7.3)

        self.create_arrow(ax, 11, 4.5, 12.3, 6.9)

        self.create_arrow(ax, 12.9, 7.5, 12.9, 7.1)

        # Parámetros
        param_text = (
            'Parámetros:\n'
            'k_rho = 0.3-0.45\n'
            'k_alpha = 1.5-1.9\n'
            'k_beta = -0.3\n'
            'vmax = 0.18 m/s\n'
            'ωmax = 1.2 rad/s\n'
            'Frecuencia: 50-80 Hz'
        )
        ax.text(0.5, 4, param_text, fontsize=8,
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
               verticalalignment='top')

        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/lyapunov_block_diagram.png',
                   dpi=300, bbox_inches='tight')
        print(f"Diagrama Lyapunov guardado en: {self.output_dir}/lyapunov_block_diagram.png")
        plt.close()

    def generate_pure_pursuit_diagram(self):
        """Genera el diagrama de bloques del controlador Pure Pursuit."""
        fig, ax = plt.subplots(figsize=(14, 10))
        ax.set_xlim(0, 14)
        ax.set_ylim(0, 10)
        ax.axis('off')

        # Título
        ax.text(7, 9.5, 'Diagrama de Bloques - Controlador Pure Pursuit',
               ha='center', fontsize=16, weight='bold')

        # Entradas
        self.create_block(ax, 1, 8, 1.2, 0.6, '/odom\n(x,y,θ)',
                         self.colors['input'])
        self.create_block(ax, 1, 7, 1.2, 0.6, 'Path\nReferencia',
                         self.colors['input'])

        # Interpolación
        self.create_block(ax, 3, 7.5, 1.5, 0.8,
                         'Interpolación\nlineal\nwaypoints',
                         self.colors['error'])

        # Lookahead
        self.create_block(ax, 5.2, 8, 1.6, 0.7,
                         'Buscar punto\nlookahead',
                         self.colors['error'])
        self.create_block(ax, 5.2, 6.8, 1.6, 0.7,
                         'L = 0.25-0.40m',
                         self.colors['error'])

        # Transformación
        self.create_block(ax, 7.5, 7.5, 1.5, 0.8,
                         'Transformar\na marco\nrobot',
                         self.colors['error'])

        # Geometría
        self.create_block(ax, 9.8, 8, 1.6, 0.7,
                         'α = atan2(Δy,Δx)\n- θ',
                         self.colors['calc'])
        self.create_block(ax, 9.8, 6.8, 1.6, 0.7,
                         'κ = 2sin(α)/L',
                         self.colors['calc'])

        # Control
        self.create_block(ax, 9.8, 5.5, 1.6, 0.7,
                         'v = vbase',
                         self.colors['calc'])
        self.create_block(ax, 9.8, 4.5, 1.6, 0.7,
                         'ω = v × κ',
                         self.colors['calc'])

        # Reducción en curvas
        self.create_block(ax, 11.8, 5, 1.5, 0.9,
                         'Si |ω|>0.5:\nv×(0.5/|ω|)',
                         self.colors['calc'])

        # Salida
        self.create_block(ax, 13, 6.5, 1.2, 0.8,
                         '/cmd_vel\n(v, ω)',
                         self.colors['output'])
        self.create_block(ax, 13, 5, 1.2, 0.6,
                         '/lookahead\n_point',
                         self.colors['output'])

        # Flechas
        self.create_arrow(ax, 1.6, 7, 2.25, 7.4)
        self.create_arrow(ax, 3.75, 7.5, 4.4, 7.8)
        self.create_arrow(ax, 1.6, 8, 4.4, 8.2)

        self.create_arrow(ax, 5.2, 6.8, 5.2, 7.65)
        self.create_arrow(ax, 6.0, 8, 6.75, 7.7)

        self.create_arrow(ax, 9, 7.5, 9, 8)
        self.create_arrow(ax, 9, 7.5, 9, 6.8)

        self.create_arrow(ax, 10.6, 8, 11, 6.8)
        self.create_arrow(ax, 10.6, 6.8, 11, 6.8)

        self.create_arrow(ax, 10.6, 5.5, 10.6, 4.8)
        self.create_arrow(ax, 10.6, 4.5, 11.05, 4.8)

        self.create_arrow(ax, 12.55, 5, 12.4, 6.2)

        self.create_arrow(ax, 6.0, 8, 12.4, 5.3)

        # Parámetros
        param_text = (
            'Parámetros:\n'
            'L = 0.25-0.40 m\n'
            'vbase = 0.18 m/s\n'
            'ωmax = 1.2 rad/s\n'
            'wheelbase = 0.287 m\n'
            'Frecuencia: 20 Hz'
        )
        ax.text(0.5, 4, param_text, fontsize=8,
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
               verticalalignment='top')

        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/pure_pursuit_block_diagram.png',
                   dpi=300, bbox_inches='tight')
        print(f"Diagrama Pure Pursuit guardado en: {self.output_dir}/pure_pursuit_block_diagram.png")
        plt.close()

    def generate_mpc_diagram(self):
        """Genera el diagrama de bloques del controlador MPC."""
        fig, ax = plt.subplots(figsize=(14, 11))
        ax.set_xlim(0, 14)
        ax.set_ylim(0, 11)
        ax.axis('off')

        # Título
        ax.text(7, 10.3, 'Diagrama de Bloques - Controlador MPC',
               ha='center', fontsize=16, weight='bold')

        # Entradas
        self.create_block(ax, 1, 9, 1.2, 0.6, '/odom\n(x,y,θ)',
                         self.colors['input'])
        self.create_block(ax, 1, 8, 1.2, 0.6, 'Waypoints',
                         self.colors['input'])

        # Estado y selección
        self.create_block(ax, 3, 8.5, 1.4, 0.8,
                         'Estado\n[x,y,θ]',
                         self.colors['error'])
        self.create_block(ax, 3, 7.2, 1.4, 0.8,
                         'Lookahead\n0.2-0.35m',
                         self.colors['error'])

        # Modo
        self.create_block(ax, 5.2, 7.8, 1.6, 1.2,
                         '¿|error_θ|>0.35?\nRotación/Avance',
                         self.colors['control'])

        # Horizonte
        self.create_block(ax, 7.5, 9, 1.5, 0.7,
                         'Horizonte\nN = 8-12',
                         self.colors['control'])
        self.create_block(ax, 7.5, 8, 1.5, 0.8,
                         'Muestreo\nv: 5, ω: 7',
                         self.colors['control'])

        # Modelo dinámico
        self.create_block(ax, 7.5, 6.5, 1.8, 1,
                         'x[k+1] = x[k]+\nv·cos(θ)·Δt\ny,θ similar',
                         self.colors['calc'])

        # Función de costo
        self.create_block(ax, 10, 9, 2, 1.3,
                         'J = Σ[\nWp·||p-pref||² +\nWa·(θ-θref)² +\nWv·v² + Wω·ω² +\nWκ·κ²]',
                         self.colors['control'])

        # Optimización
        self.create_block(ax, 10, 6.8, 2, 0.8,
                         'Búsqueda grilla\nmin J(v,ω)',
                         self.colors['control'])

        # Suavizado
        self.create_block(ax, 10, 5.3, 2, 0.8,
                         'Suavizado EMA\nα = 0.8-0.9',
                         self.colors['calc'])

        # Saturación
        self.create_block(ax, 10, 4, 2, 0.7,
                         'Saturar v, ω',
                         self.colors['sat'])

        # Salidas
        self.create_block(ax, 12.8, 6, 1.2, 0.7,
                         '/cmd_vel',
                         self.colors['output'])
        self.create_block(ax, 12.8, 4.8, 1.2, 0.8,
                         '/mpc_\nprediction',
                         self.colors['output'])

        # Flechas principales
        self.create_arrow(ax, 1.6, 9, 2.3, 8.7)
        self.create_arrow(ax, 1.6, 8, 2.3, 7.4)

        self.create_arrow(ax, 3.7, 8.5, 4.4, 8.2)
        self.create_arrow(ax, 3.7, 7.2, 4.4, 7.6)

        self.create_arrow(ax, 6, 7.8, 6.6, 8.8)
        self.create_arrow(ax, 6, 7.8, 6.6, 7.8)

        self.create_arrow(ax, 9.0, 9, 9, 9.3)
        self.create_arrow(ax, 9.0, 8, 9, 9.3)
        self.create_arrow(ax, 9.0, 6.5, 9, 9)

        self.create_arrow(ax, 11, 9, 11, 7.6)
        self.create_arrow(ax, 11, 6.8, 11, 6.1)
        self.create_arrow(ax, 10, 5.3, 10, 4.7)

        self.create_arrow(ax, 11, 4, 12.2, 5.8)
        self.create_arrow(ax, 11, 4, 12.2, 5.0)

        self.create_arrow(ax, 9.0, 6.5, 12.2, 5.2)

        # Parámetros
        param_text = (
            'Parámetros:\n'
            'N = 8-12 pasos\n'
            'Δt = 0.07-0.1 s\n'
            'Wp = 20-28\n'
            'Wa = 1.0-1.3\n'
            'Wv = 0.008-0.02\n'
            'Wω = 0.2-0.3\n'
            'vmax = 0.18 m/s\n'
            'ωmax = 1.0 rad/s\n'
            'Frecuencia: 10-14 Hz'
        )
        ax.text(0.5, 3.5, param_text, fontsize=8,
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
               verticalalignment='top')

        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/mpc_block_diagram.png',
                   dpi=300, bbox_inches='tight')
        print(f"Diagrama MPC guardado en: {self.output_dir}/mpc_block_diagram.png")
        plt.close()

    def generate_comparison_diagram(self):
        """Genera un diagrama comparativo de los cuatro controladores."""
        fig, ax = plt.subplots(figsize=(14, 8))
        ax.set_xlim(0, 14)
        ax.set_ylim(0, 8)
        ax.axis('off')

        # Título
        ax.text(7, 7.5, 'Comparación de Controladores - Flujo General',
               ha='center', fontsize=16, weight='bold')

        # Bloques comunes
        y_positions = [6, 4.5, 3, 1.5]
        controllers = ['PID', 'Lyapunov', 'Pure Pursuit', 'MPC']

        for i, (y, ctrl) in enumerate(zip(y_positions, controllers)):
            # Nombre
            ax.text(0.5, y, ctrl, fontsize=11, weight='bold',
                   ha='right', va='center')

            # Sensores
            self.create_block(ax, 2, y, 1.2, 0.5, 'Odom',
                             self.colors['input'])

            # Algoritmo
            if ctrl == 'PID':
                algo_text = 'P+I+D'
            elif ctrl == 'Lyapunov':
                algo_text = 'V(ρ,α,β)'
            elif ctrl == 'Pure Pursuit':
                algo_text = 'κ=2sin(α)/L'
            else:
                algo_text = 'min J(N)'

            self.create_block(ax, 4, y, 1.5, 0.5, algo_text,
                             self.colors['control'])

            # Velocidades
            self.create_block(ax, 6.5, y, 1.2, 0.5, 'v, ω',
                             self.colors['calc'])

            # Saturación
            self.create_block(ax, 8.5, y, 1.2, 0.5, 'Sat',
                             self.colors['sat'])

            # Salida
            self.create_block(ax, 10.5, y, 1.2, 0.5, 'cmd_vel',
                             self.colors['output'])

            # Robot
            self.create_block(ax, 12.5, y, 1.2, 0.5, 'Robot',
                             self.colors['input'])

            # Flechas
            self.create_arrow(ax, 2.6, y, 3.25, y)
            self.create_arrow(ax, 4.75, y, 5.9, y)
            self.create_arrow(ax, 7.1, y, 7.9, y)
            self.create_arrow(ax, 9.1, y, 9.9, y)
            self.create_arrow(ax, 11.1, y, 11.9, y)

            # Retroalimentación
            if i < len(y_positions) - 1:
                next_y = y_positions[i + 1]
                self.create_arrow(ax, 12.5, y - 0.3, 12.5, next_y + 0.3,
                                style='-', label='')

            # Línea de retroalimentación superior
            if i == 0:
                ax.plot([13.2, 13.2, 1.3, 1.3],
                       [y, 7, 7, y_positions[0]],
                       'k--', linewidth=1.5)
                ax.text(13.5, 5, 'Retroalimentación',
                       rotation=90, va='center', fontsize=9)

        # Características
        char_text = (
            'Características:\n'
            'PID: Simple, empírico, 100 Hz\n'
            'Lyapunov: Estabilidad probada, 50-80 Hz\n'
            'Pure Pursuit: Geométrico, suave, 20 Hz\n'
            'MPC: Predictivo, óptimo, 10-14 Hz'
        )
        ax.text(7, 0.3, char_text, fontsize=9, ha='center',
               bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7))

        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/comparison_block_diagram.png',
                   dpi=300, bbox_inches='tight')
        print(f"Diagrama de comparación guardado en: {self.output_dir}/comparison_block_diagram.png")
        plt.close()

    def generate_all_diagrams(self):
        """Genera todos los diagramas de bloques."""
        print("\n=== Generando Diagramas de Bloques ===\n")

        self.generate_pid_diagram()
        self.generate_lyapunov_diagram()
        self.generate_pure_pursuit_diagram()
        self.generate_mpc_diagram()
        self.generate_comparison_diagram()

        print(f"\n✓ Todos los diagramas han sido generados en: {self.output_dir}/")
        print("\nDiagramas creados:")
        print("  1. pid_block_diagram.png")
        print("  2. lyapunov_block_diagram.png")
        print("  3. pure_pursuit_block_diagram.png")
        print("  4. mpc_block_diagram.png")
        print("  5. comparison_block_diagram.png")


def main():
    """Función principal."""
    import sys

    # Determinar directorio de salida
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        # Directorio por defecto
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(os.path.dirname(script_dir), 'docs', 'diagrams')

    # Generar diagramas
    generator = BlockDiagramGenerator(output_dir)
    generator.generate_all_diagrams()


if __name__ == '__main__':
    main()
