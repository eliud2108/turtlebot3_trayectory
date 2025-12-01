# Documentación de Controladores TurtleBot3

Esta carpeta contiene la documentación detallada de los controladores implementados en el proyecto.

## Contenido

### 📄 Documentos Disponibles

1. **`controller_block_diagrams.md`** - Diagramas de bloques de todos los controladores
   - Diagramas Mermaid que se renderizan automáticamente en GitHub
   - Ecuaciones de control para cada algoritmo
   - Parámetros de configuración
   - Tabla comparativa de controladores

## Diagramas de Bloques

Los diagramas de bloques están disponibles en formato Mermaid dentro de `controller_block_diagrams.md`. Estos diagramas se renderizan automáticamente cuando se visualiza el archivo en GitHub.

### Controladores Documentados

1. **PID (Proportional-Integral-Derivative)**
   - Control clásico con anti-windup
   - Frecuencia: 100 Hz
   - 3 variantes: cuadrada, recta, compuesta

2. **Lyapunov**
   - Control con estabilidad probada matemáticamente
   - Lookahead adaptativo en variante compuesta
   - Frecuencia: 50-80 Hz

3. **Pure Pursuit**
   - Control geométrico basado en curvatura
   - Lookahead fijo: 0.25-0.40 m
   - Frecuencia: 20 Hz

4. **MPC (Model Predictive Control)**
   - Control predictivo con optimización
   - Horizonte: 8-12 pasos
   - Frecuencia: 10-14 Hz

## Generación de Diagramas PNG (Opcional)

Si deseas generar diagramas en formato PNG de alta calidad, puedes usar el script `../scripts/generate_block_diagrams.py`.

### Requisitos

```bash
pip install matplotlib numpy
```

### Uso

```bash
# Desde el directorio raíz del proyecto
python3 scripts/generate_block_diagrams.py

# Los diagramas se generarán en: docs/diagrams/
```

### Diagramas PNG Generados

El script genera los siguientes archivos:

1. `pid_block_diagram.png` - Diagrama del controlador PID
2. `lyapunov_block_diagram.png` - Diagrama del controlador Lyapunov
3. `pure_pursuit_block_diagram.png` - Diagrama del controlador Pure Pursuit
4. `mpc_block_diagram.png` - Diagrama del controlador MPC
5. `comparison_block_diagram.png` - Comparación de todos los controladores

## Visualización

### En GitHub

Los diagramas Mermaid en `controller_block_diagrams.md` se renderizan automáticamente cuando visualizas el archivo en GitHub. No necesitas instalar nada.

### Localmente

Para visualizar los diagramas Mermaid localmente, puedes usar:

1. **VS Code**: Instala la extensión "Markdown Preview Mermaid Support"
2. **Navegador**: Usa herramientas como [Mermaid Live Editor](https://mermaid.live/)
3. **CLI**: Usa [mermaid-cli](https://github.com/mermaid-js/mermaid-cli)

## Estructura de los Diagramas

Cada diagrama de bloques incluye:

### 📥 Entradas
- Odometría (`/odom`)
- Waypoints de referencia
- Estado de la batería

### ⚙️ Procesamiento
- Cálculo de errores
- Transformaciones de coordenadas
- Algoritmo de control específico

### 📤 Salidas
- Comandos de velocidad (`/cmd_vel`)
- Trayectorias para visualización
- Logs en formato CSV

### 🎛️ Parámetros
- Ganancias del controlador
- Límites de velocidad
- Umbrales de convergencia

## Ecuaciones Principales

### PID
```
ω = angular_gain × (kp·α + ki·∫α dt + kd·dα/dt)
v = vmax × (1 - min(|α|/(π/2), 1)) × min(1, ρ/0.5)
```

### Lyapunov
```
V = ½(ρ² + α² + β²)
v = k_rho × ρ × cos(α) × e^(-|α|)
ω = k_alpha × α + k_rho × sin(α)cos(α)/ρ + k_beta × β
```

### Pure Pursuit
```
κ = 2 × sin(α) / L
v = vbase
ω = v × κ
```

### MPC
```
J = Σ[Wp·||p-pref||² + Wa·(θ-θref)² + Wv·v² + Wω·ω² + Wκ·κ²]
(v*, ω*) = argmin J(v, ω)
```

## Referencias

### Implementación
- Directorio: `turtlebot3_control/`
- 12 controladores (4 algoritmos × 3 trayectorias)

### Literatura
1. **PID**: Åström & Hägglund (2006) - Advanced PID Control
2. **Lyapunov**: Siegwart & Nourbakhsh (2004) - Autonomous Mobile Robots
3. **Pure Pursuit**: Coulter (1992) - CMU Technical Report
4. **MPC**: Camacho & Alba (2013) - Model Predictive Control

## Contribuir

Para agregar o mejorar la documentación:

1. Edita `controller_block_diagrams.md` para cambios en diagramas Mermaid
2. Modifica `../scripts/generate_block_diagrams.py` para diagramas PNG
3. Actualiza este README con nueva información

## Contacto

Para preguntas o sugerencias sobre la documentación, abre un issue en el repositorio del proyecto.

---

**Última actualización**: 2025-12-01
