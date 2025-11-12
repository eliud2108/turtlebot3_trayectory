#!/usr/bin/env python3
"""
simple_lyapunov_controller_compuesta.py
Controlador Lyapunov SUAVE, RÁPIDO y ESTABLE
Radio 1.75 m | Curva ultra fluida | Sin paradas
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import BatteryState
import math
import numpy as np
import time
import csv
import os
from datetime import datetime


class SmoothLyapunovController(Node):
    def __init__(self):
        super().__init__('smooth_lyapunov_controller')

        # --- Parámetros ---
        self.declare_parameter('side_m', 1.75)
        self.declare_parameter('k_rho', 0.45)
        self.declare_parameter('k_alpha', 1.9)
        self.declare_parameter('k_beta', -0.3)
        self.declare_parameter('max_linear', 0.18)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('waypoint_threshold', 0.05)  # Más amplio
        self.declare_parameter('rate_hz', 80.0)

        L = float(self.get_parameter('side_m').value)
        radio = L
        center_x = L
        center_y = -L

        # --- Waypoints ---
        base_waypoints = [[0.0, 0.0], [L, 0.0]]
        num_arc_points = 60
        for i in range(1, num_arc_points + 1):
            angle = math.pi / 2 - (math.pi / 2) * (i / num_arc_points)
            wx = center_x + radio * math.cos(angle)
            wy = center_y + radio * math.sin(angle)
            base_waypoints.append([wx, wy])
        base_waypoints.append([2 * L, -L])

        self.ref_path = self._build_reference_path(base_waypoints, points_per_edge=30)
        self.waypoints = base_waypoints
        self.current_waypoint_idx = 0  # Índice del waypoint actual

        # --- Parámetros ---
        self.k_rho = float(self.get_parameter('k_rho').value)
        self.k_alpha = float(self.get_parameter('k_alpha').value)
        self.k_beta = float(self.get_parameter('k_beta').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.waypoint_threshold = float(self.get_parameter('waypoint_threshold').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # --- Lookahead ---
        self.min_lookahead = 0.18
        self.max_lookahead = 0.35
        self.curvature_buffer = []
        self.buffer_size = 5
        self.prev_v = 0.0
        self.alpha_filter = 0.75

        # --- Estado ---
        self.current_pose = None
        self.initialized = False
        self.start_time = None
        self.prev_position = None
        self.total_distance = 0.0
        self.robot_path = []
        self.cmd_log = []
        self.err_log = []
        self.batt_log = []

        # --- IO ---
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_dir = os.path.expanduser('~/tb3_runs')
        os.makedirs(self.base_dir, exist_ok=True)
        self.csv_path = os.path.join(self.base_dir, f'compuesta_lyapunov_{stamp}.csv')

        # --- Pub/Sub ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.ref_path_pub = self.create_publisher(Path, '/reference_path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.batt_sub = self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 10)

        # --- Timers ---
        self.dt = max(1.0 / self.rate_hz, 0.008)
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_paths)
        self.init_timer = self.create_timer(0.5, self.try_init)

        self.get_logger().info('LYAPUNOV SUAVE INICIADO')


    # ---------- Callbacks ----------
    def try_init(self):
        if not self.initialized and self.current_pose is not None:
            self.initialized = True
            self.start_time = time.time()
            self.get_logger().info('INICIALIZADO - INICIANDO CONTROL')
            self.init_timer.cancel()

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        if not self.initialized: return

        t = time.time() - self.start_time
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        self.robot_path.append([t, x, y])

        if self.prev_position is not None:
            d = math.hypot(x - self.prev_position[0], y - self.prev_position[1])
            if d > 1e-3: self.total_distance += d
        self.prev_position = [x, y]

        rx, ry = self._closest_on_polyline(x, y, self.ref_path)
        err = math.hypot(rx - x, ry - y)
        self.err_log.append([t, err])

    def batt_cb(self, msg: BatteryState):
        if not self.initialized: return
        t = time.time() - self.start_time
        V = float(msg.voltage) if msg.voltage is not None else float('nan')
        I = float(msg.current) if msg.current is not None else float('nan')
        P = float(msg.percentage) if msg.percentage is not None else float('nan')
        self.batt_log.append([t, V, I, P])


    # ---------- Control ----------
    def control_loop(self):
        if not self.initialized or self.current_pose is None:
            return

        if self.current_waypoint_idx >= len(self.waypoints) - 1:
            self._stop_and_export()
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self._yaw_from_quat(self.current_pose.orientation)

        # --- 1. Verificar si pasamos el waypoint actual ---
        wx, wy = self.waypoints[self.current_waypoint_idx]
        dist_to_wp = math.hypot(wx - x, wy - y)
        if dist_to_wp < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().debug(f"Waypoint {self.current_waypoint_idx} alcanzado")
            return  # Salir y esperar próximo ciclo

        # --- 2. Lookahead dinámico ---
        curvature = self._estimate_curvature(x, y)
        self.curvature_buffer.append(curvature)
        if len(self.curvature_buffer) > self.buffer_size:
            self.curvature_buffer.pop(0)
        avg_curvature = np.mean(self.curvature_buffer) if self.curvature_buffer else 0.0
        lookahead = self.max_lookahead - (self.max_lookahead - self.min_lookahead) * min(avg_curvature * 2.5, 1.0)
        lookahead = np.clip(lookahead, self.min_lookahead, self.max_lookahead)

        # --- 3. Encontrar punto lookahead ---
        target = self._find_lookahead_point(x, y, lookahead)
        if target is None:
            return

        gx, gy = target
        dx, dy = gx - x, gy - y
        rho = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        alpha = self._wrap(desired - theta)

        # --- 4. Lyapunov ---
        v_base = self.k_rho * rho
        if abs(alpha) > 0.1:
            v_base *= math.cos(alpha) ** 2

        w = self.k_alpha * alpha
        if rho > 1e-2:
            w += self.k_rho * math.sin(alpha) * math.cos(alpha) / rho
            if self.k_beta != 0.0:
                beta = self._wrap(theta - desired)
                w += self.k_beta * beta

        # --- 5. Suavizado de velocidad ---
        speed_factor = 1.0
        if rho > 0.2:
            angle_penalty = min(abs(alpha) / (math.pi / 2.5), 1.0)
            speed_factor = 1.0 - 0.5 * angle_penalty
        v = v_base * speed_factor

        v = self.alpha_filter * self.prev_v + (1 - self.alpha_filter) * v
        self.prev_v = v

        v = np.clip(v, 0.06, self.max_linear)
        w = np.clip(w, -self.max_angular, self.max_angular)

        # --- 6. Publicar ---
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        t = time.time() - self.start_time
        self.cmd_log.append([t, v, w])


    # ---------- Lookahead ----------
    def _find_lookahead_point(self, x, y, dist):
        idx = max(self.current_waypoint_idx, 0)
        accum = 0.0
        for i in range(idx, len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if accum + seg_len >= dist:
                t = (dist - accum) / seg_len
                return [p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1])]
            accum += seg_len
        return self.waypoints[-1]

    # ---------- Curvatura ----------
    def _estimate_curvature(self, x, y):
        if len(self.ref_path) < 3: return 0.0
        idx = self._find_closest_index(x, y, self.ref_path)
        if idx < 1 or idx >= len(self.ref_path) - 1: return 0.0
        p0 = self.ref_path[idx - 1]
        p1 = self.ref_path[idx]
        p2 = self.ref_path[idx + 1]
        a = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
        b = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        c = math.hypot(p2[0]-p0[0], p2[1]-p0[1])
        if a == 0 or b == 0: return 0.0
        cos_angle = (a*a + b*b - c*c) / (2*a*b)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = math.acos(cos_angle)
        radius = a * b * math.sin(angle) / c if c > 1e-6 else float('inf')
        return 1.0 / radius if radius > 1e-6 else 0.0

    def _find_closest_index(self, x, y, path):
        min_d = float('inf')
        idx = 0
        for i, (px, py) in enumerate(path):
            d = (px - x)**2 + (py - y)**2
            if d < min_d:
                min_d = d
                idx = i
        return idx


    # ---------- Paths ----------
    def publish_paths(self):
        ref = Path(); ref.header.frame_id = "odom"; ref.header.stamp = self.get_clock().now().to_msg()
        for rx, ry in self.ref_path:
            ps = PoseStamped(); ps.header = ref.header
            ps.pose.position.x = rx; ps.pose.position.y = ry; ps.pose.orientation.w = 1.0
            ref.poses.append(ps)
        self.ref_path_pub.publish(ref)

        if len(self.robot_path) > 1:
            rp = Path(); rp.header.frame_id = "odom"; rp.header.stamp = self.get_clock().now().to_msg()
            for _, x, y in self.robot_path:
                ps = PoseStamped(); ps.header = rp.header
                ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.orientation.w = 1.0
                rp.poses.append(ps)
            self.robot_path_pub.publish(rp)


    # ---------- Export ----------
    def _stop_and_export(self):
        self.cmd_pub.publish(Twist())
        if not hasattr(self, 'exported'):
            self.exported = True
            self._export_results()

    def _export_results(self):
        elapsed = time.time() - self.start_time
        ideal = self._polyline_length(self.ref_path)
        err_mean = np.mean([e for _, e in self.err_log]) if self.err_log else 0.0
        err_max = np.max([e for _, e in self.err_log]) if self.err_log else 0.0

        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t','x','y','ref_x','ref_y','err','v','w','V','I','%'])
            for t, x, y in self.robot_path:
                rx, ry = self._closest_on_polyline(x, y, self.ref_path)
                err = math.hypot(rx - x, ry - y)
                v, w_cmd = self._lookup(t, self.cmd_log, 1, 2)
                V, I, P = self._lookup(t, self.batt_log, 1, 2, 3)
                w.writerow([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{rx:.4f}', f'{ry:.4f}', f'{err:.4f}',
                            f'{v:.3f}', f'{w_cmd:.3f}', f'{V:.2f}' if not math.isnan(V) else '',
                            f'{I:.3f}' if not math.isnan(I) else '', f'{P:.1f}' if not math.isnan(P) else ''])
        self.get_logger().info(f'CSV: {self.csv_path}')
        self._print_summary(elapsed, self.total_distance, ideal, err_mean, err_max)
        rclpy.shutdown()

    def _print_summary(self, t, d, ideal, em, ex):
        e = self._estimate_energy_j()
        self.get_logger().info('='*60)
        self.get_logger().info('LYAPUNOV SUAVE - MÉTRICAS')
        self.get_logger().info(f"Tiempo: {t:.2f}s | Dist: {d:.3f}m | Ref: {ideal:.3f}m")
        self.get_logger().info(f"Error medio: {em:.4f}m | Máx: {ex:.4f}m")
        if e: self.get_logger().info(f"Energía: {e:.1f}J")
        self.get_logger().info('='*60)

    def _lookup(self, t, series, *idxs):
        if not series: return tuple(float('nan') for _ in idxs)
        row = min(series, key=lambda r: abs(r[0] - t))
        return tuple(row[i] if i < len(row) else float('nan') for i in idxs)

    def _estimate_energy_j(self):
        if len(self.batt_log) < 2: return None
        E = 0.0
        for i in range(1, len(self.batt_log)):
            t0, V0, I0, _ = self.batt_log[i-1]
            t1, V1, I1, _ = self.batt_log[i]
            if any(math.isnan(v) for v in (V0,V1,I0,I1)): continue
            E += 0.5 * (V0*I0 + V1*I1) * (t1 - t0)
        return E if E > 0 else None

    def _yaw_from_quat(self, q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def _wrap(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def _build_reference_path(self, wps, points_per_edge=30):
        pts = []
        for i in range(len(wps)-1):
            x0, y0 = wps[i]; x1, y1 = wps[i+1]
            for j in range(points_per_edge):
                t = j / points_per_edge
                pts.append([x0 + t*(x1-x0), y0 + t*(y1-y0)])
        pts.append(wps[-1])
        return pts

    def _polyline_length(self, pts):
        return sum(math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]) for i in range(1, len(pts)))

    def _closest_on_polyline(self, x, y, pts):
        best = (pts[0][0], pts[0][1])
        best_d = float('inf')
        for i in range(1, len(pts)):
            x0, y0 = pts[i-1]; x1, y1 = pts[i]
            ux, uy = x1-x0, y1-y0
            L2 = ux*ux + uy*uy
            if L2 == 0: px, py = x0, y0
            else:
                t = max(0.0, min(1.0, ((x-x0)*ux + (y-y0)*uy) / L2))
                px, py = x0 + t*ux, y0 + t*uy
            d = (px-x)**2 + (py-y)**2
            if d < best_d: best_d, best = d, (px, py)
        return best


def main():
    rclpy.init()
    node = SmoothLyapunovController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node._stop_and_export()
        except: pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
