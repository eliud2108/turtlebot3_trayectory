
#!/usr/bin/env python3
"""
mpc_controller_real_fixed.py
Controlador MPC sin dependencia de scipy para TurtleBot3 Waffle Real
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import BatteryState
import math
import numpy as np
import time
import json
import csv
from datetime import datetime
import os

class MPCControllerReal(Node):
    def __init__(self):
        super().__init__('mpc_controller_real')
        
        # Crear directorio para resultados
        self.results_dir = os.path.expanduser('~/tb3_runs')
        os.makedirs(self.results_dir, exist_ok=True)
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.json_path = os.path.join(self.results_dir, f'mpc_{self.timestamp}.json')
        self.csv_path = os.path.join(self.results_dir, f'mpc_{self.timestamp}.csv')
        
        # Waypoints del cuadrado de 1.75m
        self.waypoints = [
            [0.0, 0.0],
            [1.75, 0.0],
            [1.75, 1.75],
            [0.0, 1.75],
            [0.0, 0.0]
        ]
        
        self.current_waypoint_idx = 0
        
        # Parámetros MPC simplificados
        self.horizon = 8                # Horizonte aumentado para mejor predicción
        self.dt_mpc = 0.1               # Paso de tiempo
        self.max_linear = 0.20          # Velocidad lineal máxima
        self.max_angular = 1.0          # Velocidad angular máxima
        self.waypoint_threshold = 0.08  # Umbral ligeramente mayor para transiciones suaves
        
        # Pesos para función de costo - Ajustados para mejor seguimiento
        self.weight_position = 20.0
        self.weight_angle = 1.0         # Aumentado para mejor orientación
        self.weight_velocity = 0.02     # Reducido para permitir más velocidad
        self.weight_angular_vel = 0.3   # Reducido para giros más rápidos
        
        # Estado
        self.current_pose = None
        self.robot_path = []  # [[t, x, y], ...]
        self.ref_path = self._build_reference_path(self.waypoints, points_per_edge=40)  # [[x, y], ...]
        self.initialized = False
        self.start_time = None
        self.finished = False
        self.metrics_printed = False
        
        # Variables MPC
        self.current_state = np.zeros(3)  # [x, y, theta]
        self.last_v = 0.0
        self.last_w = 0.0
        
        # Métricas
        self.total_distance = 0.0
        self.prev_position = None
        self.err_log = []  # [[t, err], ...]
        
        # Batería
        self.batt_log = []  # [[t, voltage, current, percentage], ...]
        
        # Datos de control
        self.cmd_log = []  # [[t, v, w], ...]
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.ref_path_pub = self.create_publisher(Path, '/reference_path', 10)
        self.prediction_pub = self.create_publisher(Path, '/mpc_prediction', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        
        # Timers
        self.dt = 0.1  # 10 Hz
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_paths)
        
        # Timer de inicialización
        self.init_timer = self.create_timer(1.0, self.check_initialization)
        self.init_attempts = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('CONTROLADOR MPC SIMPLIFICADO - ROBOT REAL WAFFLE')
        self.get_logger().info(f'Cuadrado: 1.75m x 1.75m')
        self.get_logger().info(f'Horizonte: {self.horizon}, dt: {self.dt_mpc}')
        self.get_logger().info(f'v_max={self.max_linear} m/s, w_max={self.max_angular} rad/s')
        self.get_logger().info('=' * 60)
    
    def battery_callback(self, msg):
        if self.start_time:
            t = time.time() - self.start_time
            voltage = float(msg.voltage) if msg.voltage is not None else float('nan')
            current = float(msg.current) if msg.current is not None else float('nan')
            percentage = float(msg.percentage) if msg.percentage is not None else float('nan')
            self.batt_log.append([t, voltage, current, percentage])
    
    def check_initialization(self):
        self.init_attempts += 1
        if self.current_pose is not None and not self.initialized:
            self.initialized = True
            self.start_time = time.time()
            self.init_timer.cancel()
            self.get_logger().info(f'✓ Inicializado tras {self.init_attempts}s')
            self.get_logger().info(f'Posición: ({self.current_pose.position.x:.3f}, {self.current_pose.position.y:.3f})')
    
    def get_robot_angle(self):
        if self.current_pose is None:
            return 0.0
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _build_reference_path(self, wps, points_per_edge=40):
        pts = []
        for i in range(len(wps)-1):
            x0, y0 = wps[i]
            x1, y1 = wps[i+1]
            for j in range(points_per_edge):
                t = j / points_per_edge
                pts.append([x0 + t * (x1 - x0), y0 + t * (y1 - y0)])
        pts.append(wps[-1][:])
        return pts
    
    def _polyline_length(self, pts):
        s = 0.0
        for i in range(1, len(pts)):
            x0, y0 = pts[i-1]
            x1, y1 = pts[i]
            s += math.hypot(x1 - x0, y1 - y0)
        return s
    
    def _closest_on_polyline(self, x, y, pts):
        best_d = float('inf')
        best = (pts[0][0], pts[0][1])
        for i in range(1, len(pts)):
            x0, y0 = pts[i-1]
            x1, y1 = pts[i]
            ux, uy = x1 - x0, y1 - y0
            L2 = ux * ux + uy * uy
            if L2 == 0.0:
                px, py = x0, y0
            else:
                t = max(0.0, min(1.0, ((x - x0) * ux + (y - y0) * uy) / L2))
                px, py = x0 + t * ux, y0 + t * uy
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d = d
                best = (px, py)
        return best
    
    def _lookup_time_series(self, t, series, *idxs):
        if not series:
            return tuple(float('nan') for _ in idxs)
        best = min(series, key=lambda row: abs(row[0] - t))
        return tuple(best[i] if i < len(best) else float('nan') for i in idxs)
    
    def _estimate_energy_j(self):
        if len(self.batt_log) < 2:
            return None
        E = 0.0
        for i in range(1, len(self.batt_log)):
            t0, V0, I0, _ = self.batt_log[i-1]
            t1, V1, I1, _ = self.batt_log[i]
            if any(math.isnan(v) for v in (V0, V1, I0, I1)):
                continue
            P0 = V0 * I0
            P1 = V1 * I1
            E += 0.5 * (P0 + P1) * (t1 - t0)
        return E if E > 0 else None
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
        if self.initialized:
            current_time = time.time() - self.start_time
            
            # Actualizar estado
            self.current_state[0] = self.current_pose.position.x
            self.current_state[1] = self.current_pose.position.y
            self.current_state[2] = self.get_robot_angle()
            
            # Guardar trayectoria del robot
            self.robot_path.append([current_time, self.current_state[0], self.current_state[1]])
            
            # Calcular distancia
            if self.prev_position is not None:
                dx = self.current_state[0] - self.prev_position[0]
                dy = self.current_state[1] - self.prev_position[1]
                dist = math.hypot(dx, dy)
                if dist > 1e-3:
                    self.total_distance += dist
            
            self.prev_position = [self.current_state[0], self.current_state[1]]
            
            # Calcular error de seguimiento
            rx, ry = self._closest_on_polyline(self.current_state[0], self.current_state[1], self.ref_path)
            err = math.hypot(rx - self.current_state[0], ry - self.current_state[1])
            self.err_log.append([current_time, err])
    
    def dynamics_model(self, state, control, dt):
        """Modelo cinemático del robot diferencial"""
        x, y, theta = state
        v, w = control
        
        x_next = x + v * np.cos(theta) * dt
        y_next = y + v * np.sin(theta) * dt
        theta_next = theta + w * dt
        
        return np.array([x_next, y_next, self.normalize_angle(theta_next)])
    
    def simple_mpc_control(self):
        """MPC simplificado sin usar scipy"""
        # Verificar límites
        if self.current_waypoint_idx >= len(self.waypoints):
            return 0.0, 0.0
        
        goal = self.waypoints[self.current_waypoint_idx]
        
        # Error actual al objetivo
        dx = goal[0] - self.current_state[0]
        dy = goal[1] - self.current_state[1]
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.current_state[2])
        
        # Si estamos muy cerca, parar
        if distance_to_goal < 0.005:
            return 0.0, 0.0
        
        # ESTRATEGIA: Primero girar, luego avanzar
        # Si el error angular es grande, priorizar giro
        if abs(angle_error) > 0.35:  # Reducido de 0.4 a 0.35 (~20 grados)
            # Modo de giro puro
            self.get_logger().debug(f'Modo giro: error_angular={math.degrees(angle_error):.1f}°')
            
            # Solo girar, no avanzar (o muy poco)
            if distance_to_goal < 0.2:
                best_v = 0.0  # No avanzar cerca del objetivo mientras gira
            else:
                best_v = 0.02  # Avance mínimo si está lejos
            
            best_w = np.clip(angle_error * 1.8, -self.max_angular, self.max_angular)
            
        else:
            # Modo de avance con corrección angular
            self.get_logger().debug(f'Modo avance: dist={distance_to_goal:.3f}m')
            
            # Mejor control encontrado
            best_v = 0.0
            best_w = 0.0
            best_cost = float('inf')
            
            # Discretización adaptativa
            v_samples = 5
            w_samples = 7
            
            # Rango de velocidades
            v_min = 0.02  # Velocidad mínima para asegurar movimiento
            v_max = min(self.max_linear, distance_to_goal * 0.5)
            
            for v_idx in range(v_samples):
                v_test = v_min + (v_idx / max(1, v_samples - 1)) * (v_max - v_min)
                
                for w_idx in range(w_samples):
                    # Sesgar w hacia la corrección del error angular
                    w_center = angle_error * 0.8
                    w_range = self.max_angular * 0.5
                    w_test = w_center + (w_idx - w_samples//2) * (2 * w_range / w_samples)
                    w_test = np.clip(w_test, -self.max_angular, self.max_angular)
                    
                    # Simular trayectoria
                    state = self.current_state.copy()
                    total_cost = 0.0
                    
                    for step in range(self.horizon):
                        # Propagar dinámica
                        state = self.dynamics_model(state, [v_test, w_test], self.dt_mpc)
                        
                        # Error de posición
                        pos_error = math.sqrt((goal[0] - state[0])**2 + (goal[1] - state[1])**2)
                        total_cost += self.weight_position * pos_error * (1 + step * 0.1)
                        
                        # Error de orientación
                        desired_angle = math.atan2(goal[1] - state[1], goal[0] - state[0])
                        angle_err = abs(self.normalize_angle(desired_angle - state[2]))
                        total_cost += self.weight_angle * angle_err
                        
                        # Favorecer velocidad hacia adelante
                        total_cost -= 0.1 * v_test  # Bonus por moverse
                        
                        # Penalización por velocidad angular excesiva
                        total_cost += self.weight_angular_vel * abs(w_test) * 0.5
                    
                    # Actualizar mejor control
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_v = v_test
                        best_w = w_test
            
            # Asegurar movimiento mínimo
            if best_v < 0.02 and distance_to_goal > 0.2:
                best_v = 0.05
        
        # Suavizado (menos agresivo para permitir giros rápidos)
        if abs(angle_error) > 0.4:
            # Menos suavizado en giros
            alpha = 0.9
        else:
            # Suavizado normal en avance
            alpha = 0.8
            
        smooth_v = alpha * best_v + (1 - alpha) * self.last_v
        smooth_w = alpha * best_w + (1 - alpha) * self.last_w
        
        # Actualizar últimos comandos
        self.last_v = smooth_v
        self.last_w = smooth_w
        
        return smooth_v, smooth_w
    
    def publish_mpc_prediction(self):
        """Publicar predicción MPC para visualización"""
        prediction_path = Path()
        prediction_path.header.frame_id = "odom"
        prediction_path.header.stamp = self.get_clock().now().to_msg()
        
        state = self.current_state.copy()
        v, w = self.last_v, self.last_w
        
        for i in range(self.horizon):
            pose = PoseStamped()
            pose.header = prediction_path.header
            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            prediction_path.poses.append(pose)
            
            state = self.dynamics_model(state, [v, w], self.dt_mpc)
        
        self.prediction_pub.publish(prediction_path)
    
    def control_loop(self):
        if not self.initialized or self.current_pose is None or self.finished:
            return
        
        current_time = time.time() - self.start_time
        
        # Verificar límites de waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.finished = True
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self._stop_and_export()
            return
        
        # Obtener waypoint actual
        goal = self.waypoints[self.current_waypoint_idx]
        distance = math.sqrt(
            (goal[0] - self.current_state[0])**2 + 
            (goal[1] - self.current_state[1])**2
        )
        
        # Calcular ángulo al objetivo
        dx = goal[0] - self.current_state[0]
        dy = goal[1] - self.current_state[1]
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.current_state[2])
        
        # Verificar si llegamos al waypoint
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            # Reset velocidades para nuevo segmento
            self.last_v = 0.0
            self.last_w = 0.0
            self.get_logger().info(f'✓ Waypoint {self.current_waypoint_idx}/{len(self.waypoints)} alcanzado')
            
            # Pausa más larga en esquinas para permitir giro
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            time.sleep(0.3)  # Pausa más larga
            
            # Log del siguiente objetivo si existe
            if self.current_waypoint_idx < len(self.waypoints):
                next_goal = self.waypoints[self.current_waypoint_idx]
                self.get_logger().info(f'Siguiente objetivo: ({next_goal[0]:.2f}, {next_goal[1]:.2f})')
            return
        
        # Calcular control MPC
        v_cmd, w_cmd = self.simple_mpc_control()
        
        # Guardar comandos
        self.cmd_log.append([current_time, v_cmd, w_cmd])
        
        # Publicar predicción
        self.publish_mpc_prediction()
        
        # Publicar comando
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)
        
        # Debug mejorado
        if int(current_time * 2) % 4 == 0:
            self.get_logger().info(
                f'WP[{self.current_waypoint_idx}→{goal[0]:.2f},{goal[1]:.2f}]: '
                f'dist={distance:.3f}m, θ_err={math.degrees(angle_error):.1f}°, '
                f'v={v_cmd:.3f}, w={w_cmd:.3f}'
            )
    
    def publish_paths(self):
        # Referencia
        ref_path = Path()
        ref_path.header.frame_id = "odom"
        ref_path.header.stamp = self.get_clock().now().to_msg()
        
        for (rx, ry) in self.ref_path:
            pose = PoseStamped()
            pose.header = ref_path.header
            pose.pose.position.x = rx
            pose.pose.position.y = ry
            pose.pose.orientation.w = 1.0
            ref_path.poses.append(pose)
        
        self.ref_path_pub.publish(ref_path)
        
        # Robot path
        if len(self.robot_path) > 1:
            robot_path = Path()
            robot_path.header = ref_path.header
            
            for _, x, y in self.robot_path:
                pose = PoseStamped()
                pose.header = robot_path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                robot_path.poses.append(pose)
            
            self.robot_path_pub.publish(robot_path)
    
    def _stop_and_export(self):
        self.cmd_pub.publish(Twist())
        if not self.metrics_printed:
            self.metrics_printed = True
            self.save_results()
    
    def save_results(self):
        elapsed = (time.time() - self.start_time) if self.start_time else 0.0
        ideal_len = self._polyline_length(self.ref_path)
        avg_err = float(np.mean([e for _, e in self.err_log])) if self.err_log else 0.0
        max_err = float(np.max([e for _, e in self.err_log])) if self.err_log else 0.0
        
        # JSON
        data = {
            "meta": {
                "controller": "mpc",
                "side_m": 1.75,
                "rate_hz": 10.0,
                "horizon": self.horizon,
                "dt_mpc": self.dt_mpc,
                "weight_position": self.weight_position,
                "weight_angle": self.weight_angle,
                "weight_velocity": self.weight_velocity,
                "weight_angular_vel": self.weight_angular_vel,
                "max_linear": self.max_linear,
                "max_angular": self.max_angular,
                "waypoint_threshold": self.waypoint_threshold,
                "start_time_unix": self.start_time,
                "elapsed_s": elapsed,
            },
            "reference_path": [{"x": x, "y": y} for (x, y) in self.ref_path],
            "robot_path": [{"t": t, "x": x, "y": y} for (t, x, y) in self.robot_path],
            "commands": [{"t": t, "v": v, "w": w} for (t, v, w) in self.cmd_log],
            "battery": [{"t": t, "voltage": V, "current": I, "percentage": p}
                        for (t, V, I, p) in self.batt_log],
            "metrics": {
                "elapsed_s": elapsed,
                "path_length_real_m": self.total_distance,
                "path_length_ref_m": ideal_len,
                "mean_tracking_error_m": avg_err,
                "max_tracking_error_m": max_err,
                "energy_joules_est": self._estimate_energy_j()
            }
        }
        with open(self.json_path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f'✓ JSON guardado en {self.json_path}')
        
        # CSV
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'ref_x', 'ref_y', 'err', 'v', 'w', 'voltage', 'current', 'percentage'])
            for (t, x, y) in self.robot_path:
                rx, ry = self._closest_on_polyline(x, y, self.ref_path)
                err = math.hypot(rx - x, ry - y)
                v, w_cmd = self._lookup_time_series(t, self.cmd_log, 1, 2)
                V, I, P = self._lookup_time_series(t, self.batt_log, 1, 2, 3)
                w.writerow([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}',
                            f'{rx:.4f}', f'{ry:.4f}', f'{err:.4f}',
                            f'{v:.3f}', f'{w_cmd:.3f}',
                            f'{V:.3f}' if V == V else '',
                            f'{I:.3f}' if I == I else '',
                            f'{P:.3f}' if P == P else ''])
        self.get_logger().info(f'✓ CSV guardado en {self.csv_path}')
        
        # Imprimir resumen
        self._print_summary(data["metrics"])
        
        # Cerrar el nodo después de exportar
        rclpy.shutdown()
    
    def _print_summary(self, m):
        self.get_logger().info('='*60)
        self.get_logger().info('MÉTRICAS FINALES')
        self.get_logger().info(f"Tiempo total: {m['elapsed_s']:.2f} s")
        self.get_logger().info(f"Longitud real: {m['path_length_real_m']:.3f} m")
        self.get_logger().info(f"Longitud referencia: {m['path_length_ref_m']:.3f} m")
        self.get_logger().info(f"Error medio: {m['mean_tracking_error_m']:.4f} m")
        self.get_logger().info(f"Error máx.: {m['max_tracking_error_m']:.4f} m")
        if m['energy_joules_est'] is not None:
            self.get_logger().info(f"Energía estimada: {m['energy_joules_est']:.1f} J")
        self.get_logger().info('='*60)

def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop_and_export()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
