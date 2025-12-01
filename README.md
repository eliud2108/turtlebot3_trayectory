This repository contains the implementation of four controllers for trajectory performance analysis on the turtlebot3 waffle robot. The controllers used are: PID, Lyapunov, Pure Pursuit, and MPC. Metrics are stored in the tb3_runs folder for later analysis.

## Diagramas de Bloques de los Controladores

El repositorio incluye documentación visual completa de los controladores mediante diagramas de bloques. Esta documentación permite entender la estructura y el flujo de cada algoritmo de control.

### 📚 Acceder a los Diagramas

Los diagramas de bloques están disponibles en formato Mermaid en el archivo:

**[`docs/controller_block_diagrams.md`](docs/controller_block_diagrams.md)**

Este archivo incluye:
- ✅ Diagramas de bloques detallados para PID, Lyapunov, Pure Pursuit y MPC
- ✅ Ecuaciones de control de cada algoritmo
- ✅ Parámetros de configuración y valores típicos
- ✅ Tabla comparativa de controladores
- ✅ Referencias teóricas y de implementación

Los diagramas se renderizan automáticamente en GitHub sin necesidad de instalación adicional.

### 🎨 Generar Diagramas PNG (Opcional)

Si deseas generar diagramas en formato PNG de alta calidad para presentaciones:

```bash
# Instalar dependencias
pip3 install -r requirements-docs.txt

# Generar diagramas
python3 scripts/generate_block_diagrams.py
```

Los diagramas PNG se generan en `docs/diagrams/`.

### 📊 Controladores Documentados

| Controlador | Tipo | Frecuencia | Complejidad | Características |
|------------|------|------------|-------------|-----------------|
| **PID** | Clásico | 100 Hz | Baja | Simple, empírico, anti-windup |
| **Lyapunov** | Teórico | 50-80 Hz | Media | Estabilidad probada, lookahead adaptativo |
| **Pure Pursuit** | Geométrico | 20 Hz | Media | Basado en curvatura, seguimiento suave |
| **MPC** | Predictivo | 10-14 Hz | Alta | Optimización, horizonte N pasos |

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

2. **Gráficas Individuales por Trayectoria** (ejemplos representativos):

   Cada gráfica muestra para una corrida específica:
   - Seguimiento de trayectoria (referencia vs real)
   - Error de seguimiento en el tiempo
   - Velocidad lineal (v) y angular (w)
   - Componentes de posición X e Y
   - Esfuerzo de control total

   **Trayectoria Recta** (4 gráficas):
   - `recta_PID_*_performance.png` - Ejemplo PID en línea recta
   - `recta_lyapunov_*_performance.png` - Ejemplo Lyapunov en línea recta
   - `recta_mpc_*_performance.png` - Ejemplo MPC en línea recta
   - `recta_pure_*_performance.png` - Ejemplo Pure Pursuit en línea recta

   **Trayectoria Compuesta** (4 gráficas):
   - `compuesta_pid_*_performance.png` - Ejemplo PID en trayectoria compuesta
   - `compuesta_lyapunov_*_performance.png` - Ejemplo Lyapunov en trayectoria compuesta
   - `compuesta_mpc_*_performance.png` - Ejemplo MPC en trayectoria compuesta
   - `compuesta_pure_*_performance.png` - Ejemplo Pure Pursuit en trayectoria compuesta

   **Trayectoria Cuadrada** (4 gráficas):
   - `cuadrada_pid_*_performance.png` - Ejemplo PID en trayectoria cuadrada
   - `cuadrada_lyapunov_*_performance.png` - Ejemplo Lyapunov en trayectoria cuadrada
   - `cuadrada_mpc_*_performance.png` - Ejemplo MPC en trayectoria cuadrada
   - `cuadrada_pure_*_performance.png` - Ejemplo Pure Pursuit en trayectoria cuadrada

   Total: **12 gráficas individuales** (1 por cada combinación controlador × trayectoria)

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
1. Se corre ros2 launch turtlebot3_gazebo empty_world.launch.py

2. Luego en otra pestaña se corre el controlador correspondiente.


# Para Robot físico
# En el Robot
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
