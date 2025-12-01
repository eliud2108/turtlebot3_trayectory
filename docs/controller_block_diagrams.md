# Diagramas de Bloques de los Controladores

Este documento presenta los diagramas de bloques de los cuatro controladores principales implementados en el proyecto TurtleBot3.

## Índice
1. [Controlador PID](#1-controlador-pid)
2. [Controlador Lyapunov](#2-controlador-lyapunov)
3. [Controlador Pure Pursuit](#3-controlador-pure-pursuit)
4. [Controlador MPC (Model Predictive Control)](#4-controlador-mpc)

---

## 1. Controlador PID

### Diagrama de Bloques

```mermaid
flowchart TB
    subgraph Inputs["📥 ENTRADAS"]
        odom["/odom<br/>(x, y, θ)"]
        ref["Waypoints<br/>Referencia"]
        battery["/battery_state<br/>(V, I, %)"]
    end

    subgraph ErrorCalc["⚙️ CÁLCULO DE ERROR"]
        closest["Waypoint<br/>más cercano"]
        angle_err["Error Angular<br/>α = atan2(Δy, Δx) - θ"]
        dist_err["Error de<br/>Distancia<br/>ρ = √(Δx² + Δy²)"]
    end

    subgraph PIDBlock["🎛️ CONTROLADOR PID"]
        prop["Proporcional<br/>P = kp × α"]
        integ["Integral<br/>I = ki × ∫α dt<br/>(con anti-windup)"]
        deriv["Derivada<br/>D = kd × dα/dt"]
        sum["Σ"]
        prop --> sum
        integ --> sum
        deriv --> sum
    end

    subgraph VelCalc["📐 CÁLCULO DE VELOCIDADES"]
        ang_vel["Velocidad Angular<br/>ω = angular_gain × (P + I + D)"]
        lin_vel["Velocidad Lineal<br/>v = v_max × (1 - |α|/(π/2)) × min(1, ρ/0.5)"]
    end

    subgraph Saturation["⚡ SATURACIÓN"]
        sat_lin["Saturar v<br/>[-0.18, 0.18] m/s"]
        sat_ang["Saturar ω<br/>[-1.3, 1.3] rad/s"]
    end

    subgraph Outputs["📤 SALIDAS"]
        cmd_vel["/cmd_vel<br/>(v, ω)"]
        paths["Publicar<br/>trayectorias"]
        log["Registro<br/>CSV"]
    end

    odom --> closest
    ref --> closest
    closest --> angle_err
    closest --> dist_err

    angle_err --> prop
    angle_err --> integ
    angle_err --> deriv

    sum --> ang_vel
    angle_err --> lin_vel
    dist_err --> lin_vel

    ang_vel --> sat_ang
    lin_vel --> sat_lin

    sat_lin --> cmd_vel
    sat_ang --> cmd_vel

    odom --> paths
    odom --> log
    battery --> log
    cmd_vel --> log

    style Inputs fill:#e1f5ff
    style ErrorCalc fill:#fff4e1
    style PIDBlock fill:#ffe1f5
    style VelCalc fill:#e1ffe1
    style Saturation fill:#ffe1e1
    style Outputs fill:#f0e1ff
```

### Ecuaciones de Control

**Error Angular:**
```
α = atan2(yref - y, xref - x) - θ
```

**Ley de Control PID:**
```
ω = angular_gain × (kp·α + ki·∫α dt + kd·dα/dt)
```

**Modulación de Velocidad Lineal:**
```
v = vmax × (1 - min(|α|/(π/2), 1)) × min(1, ρ/0.5)
```

**Parámetros típicos:**
- kp = 2.3 - 2.5
- ki = 0.25 - 0.3
- kd = 0.18 - 0.2
- angular_gain = 1.5
- vmax = 0.18 m/s
- ωmax = 1.0 - 1.3 rad/s
- Frecuencia: 100 Hz

---

## 2. Controlador Lyapunov

### Diagrama de Bloques

```mermaid
flowchart TB
    subgraph Inputs["📥 ENTRADAS"]
        odom["/odom<br/>(x, y, θ)"]
        ref["Waypoints<br/>Referencia"]
        battery["/battery_state"]
    end

    subgraph Transform["🔄 TRANSFORMACIÓN AL MARCO DEL ROBOT"]
        error_calc["Calcular errores:<br/>Δx = xref - x<br/>Δy = yref - y"]
        polar["Conversión a<br/>coordenadas polares"]
        rho["ρ = √(Δx² + Δy²)<br/>(distancia)"]
        alpha["α = atan2(Δy, Δx) - θ<br/>(ángulo al objetivo)"]
        beta["β = θdesired - θ<br/>(orientación final)"]
    end

    subgraph Lyapunov["🎯 FUNCIÓN DE LYAPUNOV"]
        lyap_func["V = ½(ρ² + α² + β²)<br/>Garantía de estabilidad"]
    end

    subgraph ControlLaw["⚙️ LEY DE CONTROL LYAPUNOV"]
        lin_vel["Velocidad Lineal:<br/>v = k_rho × ρ × cos(α)<br/>(con amortiguamiento exponencial<br/>si |α| > umbral)"]
        ang_vel["Velocidad Angular:<br/>ω = k_alpha × α +<br/>k_rho × sin(α)cos(α)/ρ +<br/>k_beta × β"]
    end

    subgraph Adaptive["🔧 ADAPTACIÓN (Variante Compuesta)"]
        lookahead["Lookahead Dinámico<br/>[0.18m - 0.35m]<br/>basado en curvatura"]
        curvature["Estimación de<br/>Curvatura del Camino"]
        filter["Filtro Pasa-Bajos<br/>α = 0.75"]
    end

    subgraph Saturation["⚡ SATURACIÓN Y FILTRADO"]
        sat_lin["Saturar v<br/>[-0.18, 0.18] m/s"]
        sat_ang["Saturar ω<br/>[-1.2, 1.2] rad/s"]
        smooth["Suavizado de<br/>comandos"]
    end

    subgraph Outputs["📤 SALIDAS"]
        cmd_vel["/cmd_vel"]
        paths["Trayectorias"]
        log["CSV Log"]
    end

    odom --> error_calc
    ref --> error_calc
    error_calc --> polar
    polar --> rho
    polar --> alpha
    polar --> beta

    rho --> lyap_func
    alpha --> lyap_func
    beta --> lyap_func

    rho --> lin_vel
    alpha --> lin_vel
    rho --> ang_vel
    alpha --> ang_vel
    beta --> ang_vel

    lin_vel --> lookahead
    ang_vel --> lookahead
    lookahead --> curvature
    curvature --> filter

    filter --> sat_lin
    filter --> sat_ang

    sat_lin --> smooth
    sat_ang --> smooth

    smooth --> cmd_vel
    odom --> paths
    battery --> log
    cmd_vel --> log

    style Inputs fill:#e1f5ff
    style Transform fill:#fff4e1
    style Lyapunov fill:#ffe1f5
    style ControlLaw fill:#e1ffe1
    style Adaptive fill:#f5e1ff
    style Saturation fill:#ffe1e1
    style Outputs fill:#f0e1ff
```

### Ecuaciones de Control

**Coordenadas Polares:**
```
ρ = √((xref - x)² + (yref - y)²)
α = atan2(yref - y, xref - x) - θ
β = θdesired - θ
```

**Función de Lyapunov:**
```
V = ½(ρ² + α² + β²)
```

**Leyes de Control:**
```
v = k_rho × ρ × cos(α) × e^(-|α|)   (si |α| > umbral)
v = k_rho × ρ × cos(α)              (en otro caso)

ω = k_alpha × α + k_rho × sin(α)cos(α)/ρ + k_beta × β
```

**Parámetros típicos:**
- k_rho = 0.3 - 0.45
- k_alpha = 1.5 - 1.9
- k_beta = -0.3
- vmax = 0.18 m/s
- ωmax = 1.0 - 1.2 rad/s
- Frecuencia: 50-80 Hz

---

## 3. Controlador Pure Pursuit

### Diagrama de Bloques

```mermaid
flowchart TB
    subgraph Inputs["📥 ENTRADAS"]
        odom["/odom<br/>(x, y, θ)"]
        ref["Path de<br/>Referencia<br/>(waypoints interpolados)"]
        battery["/battery_state"]
    end

    subgraph PathInterp["📍 INTERPOLACIÓN DE CAMINO"]
        interp["Interpolación lineal<br/>entre waypoints"]
        dense_path["Camino denso<br/>(puntos cada 0.01m)"]
    end

    subgraph Lookahead["🎯 PUNTO DE LOOKAHEAD"]
        search["Buscar punto adelante<br/>del robot"]
        la_dist["Distancia lookahead<br/>L = 0.25-0.40 m"]
        la_point["Punto objetivo<br/>(xla, yla)"]
    end

    subgraph Transform["🔄 TRANSFORMACIÓN AL MARCO DEL ROBOT"]
        robot_frame["Transformar a<br/>coordenadas del robot:<br/>Δx = xla - x<br/>Δy = yla - y"]
        alpha["α = atan2(Δy, Δx) - θ"]
    end

    subgraph Geometry["📐 GEOMETRÍA DE PURE PURSUIT"]
        curvature["Cálculo de Curvatura:<br/>κ = 2sin(α)/L"]
        wheelbase["Base de ruedas<br/>b = 0.287 m"]
    end

    subgraph Control["⚙️ LEY DE CONTROL"]
        lin_vel["Velocidad Lineal:<br/>v = vbase (constante)"]
        ang_vel["Velocidad Angular:<br/>ω = v × κ"]
        reduction["Reducción en curvas:<br/>si |ω| > 0.5:<br/>v = v × (0.5/|ω|)"]
    end

    subgraph Saturation["⚡ SATURACIÓN"]
        sat_lin["Saturar v<br/>[-0.18, 0.18] m/s"]
        sat_ang["Saturar ω<br/>[-1.2, 1.2] rad/s"]
    end

    subgraph Outputs["📤 SALIDAS"]
        cmd_vel["/cmd_vel"]
        la_pub["/lookahead_point"]
        paths["Trayectorias"]
        log["CSV Log"]
    end

    ref --> interp
    interp --> dense_path
    dense_path --> search
    odom --> search

    la_dist --> search
    search --> la_point

    odom --> robot_frame
    la_point --> robot_frame
    robot_frame --> alpha

    alpha --> curvature
    la_dist --> curvature

    curvature --> lin_vel
    curvature --> ang_vel
    lin_vel --> ang_vel

    ang_vel --> reduction
    lin_vel --> reduction

    reduction --> sat_lin
    reduction --> sat_ang

    sat_lin --> cmd_vel
    sat_ang --> cmd_vel
    la_point --> la_pub

    odom --> paths
    battery --> log
    cmd_vel --> log

    style Inputs fill:#e1f5ff
    style PathInterp fill:#fff4e1
    style Lookahead fill:#ffe1f5
    style Transform fill:#e1ffe1
    style Geometry fill:#f5e1ff
    style Control fill:#fff4e1
    style Saturation fill:#ffe1e1
    style Outputs fill:#f0e1ff
```

### Ecuaciones de Control

**Punto de Lookahead:**
```
Buscar punto en el camino a distancia L del robot
L = lookahead_distance (0.25-0.40 m)
```

**Ángulo al objetivo (en marco del robot):**
```
α = atan2(yla - y, xla - x) - θ
```

**Curvatura:**
```
κ = 2 × sin(α) / L
```

**Leyes de Control:**
```
v = vbase (velocidad constante)
ω = v × κ

Si |ω| > 0.5 rad/s:
    v = v × (0.5 / |ω|)  (reducción en curvas cerradas)
```

**Parámetros típicos:**
- L = 0.25 - 0.40 m (lookahead)
- vbase = 0.18 m/s
- ωmax = 1.0 - 1.2 rad/s
- wheelbase = 0.287 m (TurtleBot3 Waffle)
- Frecuencia: 20 Hz

---

## 4. Controlador MPC (Model Predictive Control)

### Diagrama de Bloques

```mermaid
flowchart TB
    subgraph Inputs["📥 ENTRADAS"]
        odom["/odom<br/>(x, y, θ)"]
        ref["Waypoints<br/>Referencia"]
        battery["/battery_state"]
    end

    subgraph State["📊 ESTADO ACTUAL"]
        current["Estado actual:<br/>[x, y, θ]"]
        target_sel["Selección de<br/>objetivo con lookahead<br/>(0.20-0.35m)"]
    end

    subgraph ModeSelection["🔀 SELECCIÓN DE MODO"]
        angle_check["¿|error_θ| > 0.35 rad?"]
        rotation_mode["MODO ROTACIÓN:<br/>Priorizar corrección angular"]
        advance_mode["MODO AVANCE:<br/>Búsqueda en grilla (v, ω)"]
    end

    subgraph Prediction["🔮 PREDICCIÓN DE HORIZONTE"]
        horizon["Horizonte N = 8-12 pasos"]
        dt["Δt = 0.07-0.1 s"]
        samples["Muestreo de controles:<br/>v: 5 muestras [0, vmax]<br/>ω: 7 muestras [-ωmax, ωmax]"]
    end

    subgraph Dynamics["⚙️ MODELO DINÁMICO"]
        model["Para cada par (v, ω):<br/>x[k+1] = x[k] + v×cos(θ)×Δt<br/>y[k+1] = y[k] + v×sin(θ)×Δt<br/>θ[k+1] = θ[k] + ω×Δt"]
        propagate["Propagar N pasos<br/>hacia el futuro"]
    end

    subgraph CostFunction["💰 FUNCIÓN DE COSTO"]
        pos_err["Error de posición:<br/>Wp × Σ||p[k] - pref||²"]
        ang_err["Error angular:<br/>Wa × Σ(θ[k] - θref)²"]
        vel_pen["Penalización velocidad:<br/>Wv × Σv²"]
        ang_pen["Penalización ω:<br/>Wω × Σω²"]
        curv_pen["Penalización curvatura:<br/>Wκ × Σκ²<br/>(variante compuesta)"]
        total_cost["Costo Total J"]

        pos_err --> total_cost
        ang_err --> total_cost
        vel_pen --> total_cost
        ang_pen --> total_cost
        curv_pen --> total_cost
    end

    subgraph Optimization["🎯 OPTIMIZACIÓN"]
        grid_search["Búsqueda en grilla<br/>sobre todas las<br/>combinaciones (v, ω)"]
        best_control["Seleccionar (v*, ω*)<br/>con menor costo J"]
    end

    subgraph Smoothing["🌊 SUAVIZADO"]
        ema["Promedio móvil exponencial<br/>cmd[k] = α×cmd[k-1] + (1-α)×cmd*<br/>α = 0.8-0.9"]
    end

    subgraph Saturation["⚡ SATURACIÓN"]
        sat_lin["Saturar v<br/>[-0.18, 0.18] m/s"]
        sat_ang["Saturar ω<br/>[-1.0, 1.0] rad/s"]
    end

    subgraph Outputs["📤 SALIDAS"]
        cmd_vel["/cmd_vel"]
        prediction["/mpc_prediction<br/>(trayectoria predicha)"]
        paths["Trayectorias"]
        log["CSV Log"]
    end

    odom --> current
    ref --> target_sel
    current --> target_sel

    target_sel --> angle_check
    angle_check -->|Sí| rotation_mode
    angle_check -->|No| advance_mode

    rotation_mode --> samples
    advance_mode --> samples

    samples --> model
    horizon --> model
    dt --> model
    current --> model

    model --> propagate
    propagate --> pos_err
    propagate --> ang_err
    propagate --> vel_pen
    propagate --> ang_pen
    propagate --> curv_pen

    total_cost --> grid_search
    grid_search --> best_control

    best_control --> ema
    ema --> sat_lin
    ema --> sat_ang

    sat_lin --> cmd_vel
    sat_ang --> cmd_vel
    propagate --> prediction

    odom --> paths
    battery --> log
    cmd_vel --> log

    style Inputs fill:#e1f5ff
    style State fill:#fff4e1
    style ModeSelection fill:#ffe1f5
    style Prediction fill:#e1ffe1
    style Dynamics fill:#f5e1ff
    style CostFunction fill:#fff4e1
    style Optimization fill:#ffe1e1
    style Smoothing fill:#e1f5ff
    style Saturation fill:#ffe1e1
    style Outputs fill:#f0e1ff
```

### Ecuaciones de Control

**Modelo Dinámico:**
```
x[k+1] = x[k] + v[k] × cos(θ[k]) × Δt
y[k+1] = y[k] + v[k] × sin(θ[k]) × Δt
θ[k+1] = θ[k] + ω[k] × Δt
```

**Función de Costo:**
```
J = Σ(k=0 to N-1) [
    Wp × ||p[k] - pref[k]||² +
    Wa × (θ[k] - θref[k])² +
    Wv × v[k]² +
    Wω × ω[k]² +
    Wκ × κ[k]²
]
```

**Optimización:**
```
(v*, ω*) = argmin J(v, ω)
           v ∈ [0, vmax]
           ω ∈ [-ωmax, ωmax]
```

**Suavizado:**
```
vcmd = α × vcmd_prev + (1-α) × v*
ωcmd = α × ωcmd_prev + (1-α) × ω*
```

**Parámetros típicos:**
- N = 8-12 pasos (horizonte)
- Δt = 0.07-0.1 s
- Wp = 20-28 (peso posición)
- Wa = 1.0-1.3 (peso ángulo)
- Wv = 0.008-0.02
- Wω = 0.2-0.3
- Wκ = 0.15
- α = 0.8-0.9 (suavizado)
- vmax = 0.18 m/s
- ωmax = 1.0 rad/s
- Frecuencia: 10-14 Hz

---

## Comparación de Controladores

| Característica | PID | Lyapunov | Pure Pursuit | MPC |
|---------------|-----|----------|--------------|-----|
| **Tipo** | Clásico | Teórico | Geométrico | Predictivo |
| **Complejidad** | Baja | Media | Media | Alta |
| **Frecuencia** | 100 Hz | 50-80 Hz | 20 Hz | 10-14 Hz |
| **Estabilidad** | Empírica | Probada | Empírica | Garantizada (horizonte corto) |
| **Curvas** | Regular | Buena | Excelente | Excelente |
| **Costo computacional** | Muy bajo | Bajo | Bajo | Alto |
| **Parámetros** | 6 | 3-4 | 2-3 | 8-10 |
| **Lookahead** | No | Sí (adaptativo) | Sí (fijo) | Sí (adaptativo) |
| **Predicción** | No | No | No | Sí (N pasos) |

---

## Flujo General Común a Todos los Controladores

```mermaid
flowchart LR
    subgraph Sensores["🔍 SENSORES"]
        A[Odometría]
        B[Batería]
    end

    subgraph Planificador["🗺️ PLANIFICACIÓN"]
        C[Waypoints]
        D[Trayectoria<br/>Referencia]
    end

    subgraph Control["🎛️ CONTROLADOR"]
        E[Algoritmo<br/>de Control]
        F[Saturación]
    end

    subgraph Actuadores["🤖 ACTUADORES"]
        G[cmd_vel]
        H[TurtleBot3]
    end

    subgraph Monitor["📊 MONITOREO"]
        I[Visualización<br/>RViz]
        J[Logs CSV]
        K[Métricas]
    end

    A --> E
    C --> D
    D --> E
    B --> J
    E --> F
    F --> G
    G --> H
    H --> A
    A --> I
    G --> J
    J --> K

    style Sensores fill:#e1f5ff
    style Planificador fill:#fff4e1
    style Control fill:#ffe1f5
    style Actuadores fill:#e1ffe1
    style Monitor fill:#f0e1ff
```

---

## Referencias

### Archivos de Implementación

**PID:**
- `turtlebot3_control/simple_pid_controller.py`
- `turtlebot3_control/simple_pid_controller_recta.py`
- `turtlebot3_control/simple_pid_controller_compuesta.py`

**Lyapunov:**
- `turtlebot3_control/simple_lyapunov_controller.py`
- `turtlebot3_control/simple_lyapunov_controller_recta.py`
- `turtlebot3_control/simple_lyapunov_controller_compuesta.py`

**Pure Pursuit:**
- `turtlebot3_control/pure_pursuit_controller_real.py`
- `turtlebot3_control/pure_pursuit_controller_real_recta.py`
- `turtlebot3_control/pure_pursuit_controller_real_compuesta.py`

**MPC:**
- `turtlebot3_control/mpc_controller_real.py`
- `turtlebot3_control/mpc_controller_real_recta.py`
- `turtlebot3_control/mpc_controller_real_compuesta.py`

### Referencias Teóricas

1. **PID Control:** Åström, K. J., & Hägglund, T. (2006). Advanced PID control.
2. **Lyapunov Control:** Siegwart, R., & Nourbakhsh, I. R. (2004). Introduction to autonomous mobile robots.
3. **Pure Pursuit:** Coulter, R. C. (1992). Implementation of the pure pursuit path tracking algorithm. Carnegie Mellon University.
4. **MPC:** Camacho, E. F., & Alba, C. B. (2013). Model predictive control.

---

**Última actualización:** 2025-12-01
