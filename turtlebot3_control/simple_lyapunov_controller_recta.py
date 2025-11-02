#!/usr/bin/env python3
"""
simple_lyapunov_controller_export.py
Lyapunov controller for TurtleBot3 with JSON/CSV export of trajectories, metrics and battery.
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
import os
from datetime import datetime


class SimpleLyapunovController(Node):
    def __init__(self):
        super().__init__('simple_lyapunov_controller')

        # ---- Waypoints (línea recta de 1.75m; ajusta con param 'side_m') ----
        self.declare_parameter('side_m', 1.75)
        L = float(self.get_parameter('side_m').value)
        self.waypoints = [[0.0, 0.0], [L, 0.0]]
        self.current_waypoint_idx = 0

        # ---- Parámetros del controlador ----
        self.declare_parameter('k_rho', 0.3)      # distancia
        self.declare_parameter('k_alpha', 1.5)    # ángulo
        self.declare_parameter('k_beta', -0.3)   # orientación final (opcional)
        self.declare_parameter('max_linear', 0.18)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('waypoint_threshold', 0.06)
        self.declare_parameter('angle_threshold', 0.2)
        self.declare_parameter('rate_hz', 50.0)

        self.k_rho = float(self.get_parameter('k_rho').value)
        self.k_alpha = float(self.get_parameter('k_alpha').value)
        self.k_beta = float(self.get_parameter('k_beta').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.waypoint_threshold = float(self.get_parameter('waypoint_threshold').value)
        self.angle_threshold = float(self.get_parameter('angle_threshold').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # ---- Estado ----
        self.current_pose = None
        self.initialized = False
        self.start_time = None
        self.metrics_printed = False

        # ---- Buffers para exportación ----
        self.robot_path = []        # [[t,x,y], ...]
        self.ref_path = self._build_reference_path(self.waypoints, points_per_edge=40)  # [[x,y], ...]
        self.cmd_log = []           # [[t,v,w], ...]
        self.err_log = []           # [[t,err], ...]
        self.batt_log = []          # [[t,voltage,current,percentage], ...]

        # Distancia recorrida incremental (m)
        self.total_distance = 0.0
        self.prev_position = None

        # ---- Pub/Sub ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.ref_path_pub = self.create_publisher(Path, '/reference_path', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.batt_sub = self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 10)

        # ---- Timers ----
        self.control_timer = self.create_timer(max(1.0/self.rate_hz, 0.01), self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_paths)
        self.init_timer = self.create_timer(0.5, self.try_init)

        self.get_logger().info('Lyapunov controller with export READY')

        # Archivos de salida
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_dir = os.path.expanduser('~/tb3_runs')
        os.makedirs(self.base_dir, exist_ok=True)
        
        self.csv_path = os.path.join(self.base_dir, f'recta_lyapunov_{stamp}.csv')

    # ---------- Callbacks ----------
    def try_init(self):
        if not self.initialized and self.current_pose is not None:
            self.initialized = True
            self.start_time = time.time()
            self.get_logger().info('✓ Inicializado, comenzando control')
            self.init_timer.cancel()

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        if not self.initialized:
            return

        t = time.time() - self.start_time
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        self.robot_path.append([t, x, y])

        # distancia incremental
        if self.prev_position is not None:
            dx = x - self.prev_position[0]
            dy = y - self.prev_position[1]
            d = math.hypot(dx, dy)
            if d > 1e-3:
                self.total_distance += d
        self.prev_position = [x, y]

        # error vs trayectoria de referencia (distancia al punto de la polilínea más cercano)
        rx, ry = self._closest_on_polyline(x, y, self.ref_path)
        err = math.hypot(rx - x, ry - y)
        self.err_log.append([t, err])

    def batt_cb(self, msg: BatteryState):
        if not self.initialized:
            return
        t = time.time() - self.start_time if self.start_time else 0.0
        voltage = float(msg.voltage) if msg.voltage is not None else float('nan')
        current = float(msg.current) if msg.current is not None else float('nan')  # A (suele ser 0 si no está disponible)
        pct = float(msg.percentage) if msg.percentage is not None else float('nan')
        self.batt_log.append([t, voltage, current, pct])

    # ---------- Control ----------
    def control_loop(self):
        if not self.initialized or self.current_pose is None:
            return

        # ¿fin de la ruta?
        if self.current_waypoint_idx >= len(self.waypoints):
            self._stop_and_export()
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self._yaw_from_quat(self.current_pose.orientation)

        gx, gy = self.waypoints[self.current_waypoint_idx]
        dx, dy = gx - x, gy - y
        rho = math.hypot(dx, dy)

        if rho < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self._stop_and_export()
            else:
                # breve pausa en vértice
                self.cmd_pub.publish(Twist())
                time.sleep(0.1)
            return

        desired = math.atan2(dy, dx)
        alpha = self._wrap(desired - theta)

        # v (lineal)
        if abs(alpha) > self.angle_threshold:
            v = self.k_rho * rho * math.cos(alpha) * math.exp(-abs(alpha))
        else:
            v = self.k_rho * rho * math.cos(alpha)

        # w (angular)
        if rho > 1e-2:
            w = self.k_alpha * alpha + self.k_rho * math.sin(alpha) * math.cos(alpha) / rho
            if self.k_beta != 0.0:
                beta = self._wrap(theta - desired)
                w += self.k_beta * beta
        else:
            w = self.k_alpha * alpha

        # límites y seguridad
        v = max(0.0, min(v, self.max_linear))
        w = max(-self.max_angular, min(w, self.max_angular))
        if rho > self.waypoint_threshold and v < 0.05:
            v = 0.05

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # log de comandos
        t = time.time() - self.start_time if self.start_time else 0.0
        self.cmd_log.append([t, v, w])

    # ---------- Publicación de paths ----------
    def publish_paths(self):
        # referencia
        ref_path = Path()
        ref_path.header.frame_id = 'odom'
        ref_path.header.stamp = self.get_clock().now().to_msg()
        for (rx, ry) in self.ref_path:
            ps = PoseStamped()
            ps.header = ref_path.header
            ps.pose.position.x = rx
            ps.pose.position.y = ry
            ps.pose.orientation.w = 1.0
            ref_path.poses.append(ps)
        self.ref_path_pub.publish(ref_path)

        # robot
        if len(self.robot_path) > 1:
            rp = Path()
            rp.header.frame_id = 'odom'
            rp.header.stamp = self.get_clock().now().to_msg()
            for _, x, y in self.robot_path:
                ps = PoseStamped()
                ps.header = rp.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                rp.poses.append(ps)
            self.robot_path_pub.publish(rp)

    # ---------- Utilidades ----------
    def _stop_and_export(self):
        self.cmd_pub.publish(Twist())
        if not self.metrics_printed:
            self.metrics_printed = True
            self._export_results()

    def _export_results(self):
        elapsed = (time.time() - self.start_time) if self.start_time else 0.0
        ideal_len = self._polyline_length(self.ref_path)
        avg_err = float(np.mean([e for _, e in self.err_log])) if self.err_log else 0.0
        max_err = float(np.max([e for _, e in self.err_log])) if self.err_log else 0.0

        # JSON
        data = {
            "meta": {
                "controller": "recta_lyapunov",
                "side_m": float(self.get_parameter('side_m').value),
                "rate_hz": self.rate_hz,
                "k_rho": self.k_rho, "k_alpha": self.k_alpha, "k_beta": self.k_beta,
                "max_linear": self.max_linear, "max_angular": self.max_angular,
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
                # energía estimada simple (si corriente disponible): ∫ V*I dt
                "energy_joules_est": self._estimate_energy_j()
            }
        }
        

        # CSV
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t','x','y','ref_x','ref_y','err','v','w','voltage','current','percentage'])
            # indexamos por tiempo de robot_path
            for (t, x, y) in self.robot_path:
                rx, ry = self._closest_on_polyline(x, y, self.ref_path)
                err = math.hypot(rx - x, ry - y)
                # encontrar comando y batería más cercanos en tiempo
                v, w_cmd = self._lookup_time_series(t, self.cmd_log, 1, 2)
                V, I, P = self._lookup_time_series(t, self.batt_log, 1, 2, 3)
                w.writerow([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}',
                            f'{rx:.4f}', f'{ry:.4f}', f'{err:.4f}',
                            f'{v:.3f}', f'{w_cmd:.3f}',
                            f'{V:.3f}' if V==V else '',  # NaN -> vacío
                            f'{I:.3f}' if I==I else '',
                            f'{P:.3f}' if P==P else ''])
        self.get_logger().info(f'✓ CSV guardado en {self.csv_path}')
        self._print_summary(data["metrics"])

        # Cerrar el nodo después de exportar
        rclpy.shutdown()

    def _print_summary(self, m):
        self.get_logger().info('='*60)
        self.get_logger().info('MÉTRICAS FINALES - Lyapunov (Línea recta)')
        self.get_logger().info(f"Tiempo total: {m['elapsed_s']:.2f} s")
        self.get_logger().info(f"Longitud real: {m['path_length_real_m']:.3f} m")
        self.get_logger().info(f"Longitud referencia: {m['path_length_ref_m']:.3f} m")
        self.get_logger().info(f"Error medio: {m['mean_tracking_error_m']:.4f} m")
        self.get_logger().info(f"Error máx.: {m['max_tracking_error_m']:.4f} m")
        if m['energy_joules_est'] is not None:
            self.get_logger().info(f"Energía estimada: {m['energy_joules_est']:.1f} J")
        self.get_logger().info('='*60)

    # ---- helpers geométricos / series ----
    def _yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _wrap(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def _build_reference_path(self, wps, points_per_edge=40):
        pts = []
        for i in range(len(wps)-1):
            x0,y0 = wps[i]; x1,y1 = wps[i+1]
            for j in range(points_per_edge):
                t = j/points_per_edge
                pts.append([x0 + t*(x1-x0), y0 + t*(y1-y0)])
        pts.append(wps[-1][:])
        return pts

    def _polyline_length(self, pts):
        s = 0.0
        for i in range(1,len(pts)):
            x0,y0 = pts[i-1]; x1,y1 = pts[i]
            s += math.hypot(x1-x0, y1-y0)
        return s

    def _closest_on_polyline(self, x, y, pts):
        # búsqueda lineal simple; suficiente para 100-200 pts
        best_d = float('inf'); best = (pts[0][0], pts[0][1])
        for i in range(1, len(pts)):
            x0,y0 = pts[i-1]; x1,y1 = pts[i]
            ux, uy = x1-x0, y1-y0
            L2 = ux*ux + uy*uy
            if L2 == 0.0:
                px, py = x0, y0
            else:
                t = max(0.0, min(1.0, ((x-x0)*ux + (y-y0)*uy)/L2))
                px, py = x0 + t*ux, y0 + t*uy
            d = (px-x)**2 + (py-y)**2
            if d < best_d:
                best_d = d
                best = (px, py)
        return best

    def _lookup_time_series(self, t, series, *idxs):
        """Busca en 'series' (lista de [t,...]) la fila con t más cercano y retorna columnas idxs."""
        if not series:
            return tuple(float('nan') for _ in idxs)
        # serie está en orden temporal
        best = min(series, key=lambda row: abs(row[0]-t))
        return tuple(best[i] if i < len(best) else float('nan') for i in idxs)

    def _estimate_energy_j(self):
        """Integración trapezoidal de V*I dt si hay datos de corriente (A) y voltaje (V)."""
        if len(self.batt_log) < 2:
            return None
        E = 0.0
        for i in range(1, len(self.batt_log)):
            t0,V0,I0,_ = self.batt_log[i-1]
            t1,V1,I1,_ = self.batt_log[i]
            if any(math.isnan(v) for v in (V0,V1,I0,I1)):
                continue
            P0 = V0*I0; P1 = V1*I1
            E += 0.5*(P0+P1)*(t1-t0)
        return E if E>0 else None


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLyapunovController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # asegurar parada y exportación si no se alcanzó el final
        try:
            node._stop_and_export()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

