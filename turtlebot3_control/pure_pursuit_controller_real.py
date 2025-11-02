#!/usr/bin/env python3
"""
pure_pursuit_controller_real.py
Controlador Pure Pursuit para TurtleBot3 Waffle Real
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

class PurePursuitControllerReal(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller_real')
        
        # Crear directorio para resultados
        self.results_dir = os.path.expanduser('~/tb3_runs')
        os.makedirs(self.results_dir, exist_ok=True)
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.results_dir, f'pure_{self.timestamp}.csv')
        
        # Waypoints del cuadrado de 1.75m con interpolación
        self.waypoints = []
        corners = [
            [0.0, 0.0],
            [1.75, 0.0],
            [1.75, 1.75],
            [0.0, 1.75],
            [0.0, 0.0]
        ]
        
        # Interpolar puntos entre esquinas para mejor seguimiento
        for i in range(len(corners)-1):
            start = corners[i]
            end = corners[i+1]
            num_points = 40  # Puntos por segmento, ajustado para coincidir con Lyapunov
            for j in range(num_points):
                t = j / float(num_points)
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                self.waypoints.append([x, y])
        self.waypoints.append(corners[-1])
        
        # Guardar trayectoria de referencia
        self.ref_path = self.waypoints  # [[x, y], ...]
        
        # Parámetros Pure Pursuit
        self.lookahead_distance = 0.25  # Distancia de lookahead para Waffle
        self.linear_velocity = 0.18     # Velocidad lineal constante
        self.max_angular = 1.2          # Velocidad angular máxima
        self.goal_threshold = 0.06      # Umbral para considerar objetivo alcanzado
        self.wheelbase = 0.287          # Distancia entre ejes del Waffle
        
        # Estado
        self.current_pose = None
        self.current_target_idx = 0
        self.robot_path = []  # [[t, x, y], ...]
        self.initialized = False
        self.start_time = None
        self.finished = False
        self.metrics_printed = False
        
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
        self.lookahead_pub = self.create_publisher(PoseStamped, '/lookahead_point', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        
        # Timers
        self.dt = 0.05  # 20 Hz
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_paths)
        
        # Timer de inicialización
        self.init_timer = self.create_timer(1.0, self.check_initialization)
        self.init_attempts = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('CONTROLADOR PURE PURSUIT - ROBOT REAL WAFFLE')
        self.get_logger().info(f'Cuadrado: 1.75m x 1.75m')
        self.get_logger().info(f'Lookahead: {self.lookahead_distance}m')
        self.get_logger().info(f'Velocidad: {self.linear_velocity} m/s')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
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
    
    def _polyline_length(self, pts):
        s = 0.0
        for i in range(1, len(pts)):
            x0, y0 = pts[i-1]
            x1, y1 = pts[i]
            s += math.hypot(x1 - x0, y1 - y0)
        return s
    
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
            
            # Guardar trayectoria del robot
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            self.robot_path.append([current_time, x, y])
            
            # Calcular distancia
            if self.prev_position is not None:
                dx = x - self.prev_position[0]
                dy = y - self.prev_position[1]
                dist = math.hypot(dx, dy)
                if dist > 1e-3:
                    self.total_distance += dist
            
            self.prev_position = [x, y]
            
            # Calcular error de seguimiento
            rx, ry = self._closest_on_polyline(x, y, self.ref_path)
            err = math.hypot(rx - x, ry - y)
            self.err_log.append([current_time, err])
    
    def find_lookahead_point(self, robot_x, robot_y):
        """Encontrar punto de lookahead en la trayectoria"""
        lookahead_point = None
        lookahead_idx = self.current_target_idx
        
        # Buscar punto que esté a la distancia de lookahead
        for i in range(self.current_target_idx, len(self.waypoints)):
            wp = self.waypoints[i]
            dist = math.sqrt((wp[0] - robot_x)**2 + (wp[1] - robot_y)**2)
            
            if dist >= self.lookahead_distance:
                if i == 0:
                    lookahead_point = wp
                    lookahead_idx = i
                else:
                    # Interpolar entre waypoints
                    prev_wp = self.waypoints[i-1]
                    prev_dist = math.sqrt((prev_wp[0] - robot_x)**2 + 
                                         (prev_wp[1] - robot_y)**2)
                    
                    if dist > prev_dist:
                        t = (self.lookahead_distance - prev_dist) / (dist - prev_dist)
                        t = np.clip(t, 0, 1)
                        
                        lookahead_point = [
                            prev_wp[0] + t * (wp[0] - prev_wp[0]),
                            prev_wp[1] + t * (wp[1] - prev_wp[1])
                        ]
                    else:
                        lookahead_point = wp
                    lookahead_idx = i
                break
        
        # Si no encontramos punto, usar el último waypoint
        if lookahead_point is None and len(self.waypoints) > 0:
            lookahead_point = self.waypoints[-1]
            lookahead_idx = len(self.waypoints) - 1
        
        # Actualizar índice actual
        for i in range(self.current_target_idx, min(lookahead_idx + 1, len(self.waypoints))):
            wp = self.waypoints[i]
            dist = math.sqrt((wp[0] - robot_x)**2 + (wp[1] - robot_y)**2)
            if dist < self.goal_threshold:
                self.current_target_idx = i + 1
        
        return lookahead_point
    
    def pure_pursuit_control(self, robot_x, robot_y, robot_theta, lookahead_point):
        """Calcular comando de velocidad usando Pure Pursuit"""
        # Vector al punto de lookahead
        dx = lookahead_point[0] - robot_x
        dy = lookahead_point[1] - robot_y
        
        # Transformar a coordenadas del robot
        alpha = math.atan2(dy, dx) - robot_theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # Normalizar
        
        # Distancia al punto
        ld = math.sqrt(dx**2 + dy**2)
        
        # Calcular curvatura (2*sin(alpha) / ld)
        if ld > 0.001:
            curvature = 2.0 * math.sin(alpha) / ld
        else:
            curvature = 0.0
        
        # Velocidades
        linear_vel = self.linear_velocity
        angular_vel = linear_vel * curvature
        
        # Reducir velocidad en curvas cerradas
        if abs(angular_vel) > 0.5:
            linear_vel *= (0.5 / abs(angular_vel))
        
        # Límites
        linear_vel = np.clip(linear_vel, 0, 0.20)
        angular_vel = np.clip(angular_vel, -self.max_angular, self.max_angular)
        
        return linear_vel, angular_vel
    
    def control_loop(self):
        if not self.initialized or self.current_pose is None or self.finished:
            return
        
        current_time = time.time() - self.start_time
        
        # Posición actual
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_theta = self.get_robot_angle()
        
        # Verificar si llegamos al final
        if self.current_target_idx >= len(self.waypoints):
            self.finished = True
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self._stop_and_export()
            return
        
        # Encontrar punto de lookahead
        lookahead_point = self.find_lookahead_point(robot_x, robot_y)
        
        if lookahead_point is None:
            return
        
        # Publicar punto de lookahead para visualización
        lookahead_msg = PoseStamped()
        lookahead_msg.header.frame_id = "odom"
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.pose.position.x = lookahead_point[0]
        lookahead_msg.pose.position.y = lookahead_point[1]
        lookahead_msg.pose.position.z = 0.05
        lookahead_msg.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_msg)
        
        # Calcular control Pure Pursuit
        v, w = self.pure_pursuit_control(robot_x, robot_y, robot_theta, lookahead_point)
        
        # Guardar comandos
        self.cmd_log.append([current_time, v, w])
        
        # Publicar comando
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        
        # Log ocasional
        if int(current_time * 2) % 4 == 0:
            self.get_logger().debug(
                f'Waypoint {self.current_target_idx}/{len(self.waypoints)} | '
                f'v={v:.3f} m/s, w={w:.3f} rad/s'
            )
    
    def publish_paths(self):
        # Referencia
        ref_path = Path()
        ref_path.header.frame_id = "odom"
        ref_path.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.ref_path:
            pose = PoseStamped()
            pose.header = ref_path.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.01
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
                pose.pose.position.z = 0.0
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
        
        # Datos para métricas
        data = {
            "meta": {
                "controller": "pure_pursuit",
                "side_m": 1.75,
                "rate_hz": 20.0,
                "lookahead_distance": self.lookahead_distance,
                "linear_velocity": self.linear_velocity,
                "max_angular": self.max_angular,
                "goal_threshold": self.goal_threshold,
                "wheelbase": self.wheelbase,
                "start_time_unix": self.start_time,
                "elapsed_s": elapsed,
            },
            "reference_path": [{"x": x, "y": y} for x, y in self.ref_path],
            "robot_path": [{"t": t, "x": x, "y": y} for t, x, y in self.robot_path],
            "commands": [{"t": t, "v": v, "w": w} for t, v, w in self.cmd_log],
            "battery": [{"t": t, "voltage": V, "current": I, "percentage": p}
                        for t, V, I, p in self.batt_log],
            "metrics": {
                "elapsed_s": elapsed,
                "path_length_real_m": self.total_distance,
                "path_length_ref_m": ideal_len,
                "mean_tracking_error_m": avg_err,
                "max_tracking_error_m": max_err,
                "energy_joules_est": self._estimate_energy_j()
            }
        }
        
        # CSV
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'ref_x', 'ref_y', 'err', 'v', 'w', 'voltage', 'current', 'percentage'])
            for t, x, y in self.robot_path:
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
    node = PurePursuitControllerReal()
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
