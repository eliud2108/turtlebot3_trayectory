This repository contains the implementation of four controllers for trajectory performance analysis on the turtlebot3 waffle robot. The controllers used are: PID, Lyapunov, Pure Pursuit, and MPC. Metrics are stored in the tb3_runs folder for later analysis.

## Análisis de Desempeño

Este repositorio incluye herramientas para evaluar y visualizar el desempeño de los controladores.

### Instalación de Dependencias

Para ejecutar los scripts de análisis, instala las dependencias de Python:

```bash
pip3 install -r requirements.txt
```

### Métricas de Desempeño

Para calcular métricas cuantitativas (IAE, ITAE, ISE, ICE):

```bash
python3 evaluate_controller_metrics.py --directory tb3_runs
```

Esto genera un archivo `controller_metrics_summary.csv` con todas las métricas calculadas.

### Gráficas de Desempeño

#### Gráficas Generales (todos los datos combinados)

Para generar visualizaciones del desempeño de los controladores:

```bash
# Generar todas las gráficas (resumen + individuales)
python3 generate_performance_charts.py

# Generar solo gráficas de resumen comparativo
python3 generate_performance_charts.py --summary-only

# Generar gráficas individuales (máximo 5 por controlador)
python3 generate_performance_charts.py --max-individual 5

# Generar gráfica para un archivo específico
python3 generate_performance_charts.py --single tb3_runs/pid_run_20251101_145049.csv
```

Las gráficas generadas incluyen:

1. **Gráficas de Resumen** (en carpeta `charts/`):
   - `controller_comparison_summary.png`: Comparación de todos los controladores con métricas IAE, ITAE, ISE, ICE, errores promedio y máximo, y puntuación general de desempeño
   - `controller_statistics_table.png`: Tabla estadística detallada con medias y desviaciones estándar para cada controlador

2. **Gráficas Individuales** (para cada corrida de controlador):
   - Seguimiento de trayectoria (referencia vs real)
   - Error de seguimiento en el tiempo
   - Velocidad lineal (v) y angular (w)
   - Componentes de posición X e Y
   - Esfuerzo de control total

#### Análisis por Tipo de Trayectoria

El repositorio incluye datos de tres tipos de trayectorias diferentes:

- **Recta**: Trayectoria lineal de 1.75m
- **Compuesta**: Trayectoria con múltiples segmentos y cambios de dirección
- **Cuadrada**: Trayectoria cuadrada (datos originales)

Para generar análisis específicos por tipo de trayectoria:

```bash
# Generar análisis completo por tipo de trayectoria
python3 generate_trajectory_analysis.py

# Especificar directorios personalizados
python3 generate_trajectory_analysis.py --directory tb3_runs --output charts
```

**Gráficas generadas por trayectoria:**

1. **Comparaciones por Trayectoria** (`charts/comparison_[tipo].png`):
   - `comparison_recta.png`: Comparación de los 4 controladores en trayectoria recta
   - `comparison_compuesta.png`: Comparación de los 4 controladores en trayectoria compuesta
   - `comparison_cuadrada.png`: Comparación de los 4 controladores en trayectoria cuadrada

   Cada gráfica incluye: IAE, ITAE, ISE, ICE, Error Promedio, Error Máximo, Número de Corridas, y Puntuación de Desempeño

2. **Tablas Estadísticas por Trayectoria** (`charts/statistics_[tipo].png`):
   - `statistics_recta.png`: Estadísticas detalladas para trayectoria recta
   - `statistics_compuesta.png`: Estadísticas detalladas para trayectoria compuesta
   - `statistics_cuadrada.png`: Estadísticas detalladas para trayectoria cuadrada

   Muestran: Media ± Desviación Estándar para cada métrica, con los mejores valores resaltados

3. **Comparación Cross-Trayectorias** (`charts/comparison_cross_trajectory.png`):
   - Compara el desempeño de cada controlador a través de las tres trayectorias
   - Permite identificar qué controlador es más robusto ante diferentes tipos de trayectorias
   - Muestra todas las métricas (IAE, ITAE, ISE, ICE, Error Promedio, Error Máximo)

**Resumen de Datos Analizados:**

- **Recta**: 30 corridas × 4 controladores = 120 archivos
- **Compuesta**: 30 corridas × 4 controladores = 120 archivos
- **Cuadrada**: 30 corridas × 4 controladores = 120 archivos
- **Total**: 360 archivos de datos experimentales

## Modo de ejecución:

#  Compilar paquete
cd ~/turtlebot3_pruebas
colcon build --packages-select turtlebot3_control
source install/setup.bash

export TURTLEBOT3_MODEL=waffle

# Para simulación
1. se corre ros2 launch turtlebot3_gazebo empty_world.launch.py

2. luego en otra pestaña se corre el controlador correspondiente.


# Para robot fisico
# En el robot
ros2 launch turtlebot3_bringup robot.launch.py

## CONTROLADORES

# En otra pestaña - PID
ros2 run turtlebot3_control simple_pid_controller

# En otra pestaña - Lyapunov
ros2 run turtlebot3_control simple_lyapunov_controller

# MPC
ros2 run turtlebot3_control mpc_controller_real

# PURE PERSUIT
ros2 run turtlebot3_control pure_pursuit_controller_real
