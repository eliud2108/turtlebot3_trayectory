#!/usr/bin/env python3
"""
mpc_controller_compuesta.py
MPC con LOOKAHEAD CONTINUO → Curva ULTRA SUAVE
Radio 1.75 m | Sin intermitencia | Rápido y estable
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


class MPCControllerCompuesta(Node):
    def __init__(self):
        super().__init__('mpc_controller_compuesta')

        # --- Parámetros ---
        self.declare_parameter('side_m', 1.75)
        L = float(self.get_parameter('side_m').value)
        radio = L
        center_x = L
        center_y = -L

        # --- Waypoints base ---
        base_waypoints = [[0.0, 0.0], [L, 0.0]]
        num_arc_points = 80  # MÁS PUNTOS
        for i in range(1, num_arc_points + 1):
            angle = math.pi / 2 - (math.pi / 2) * (i / num_arc_points)
            wx = center_x + radio * math.cos(angle)
            wy = center_y + radio * math.sin(angle)
            base_waypoints.append([wx, wy])
        base_waypoints.append([2 * L, -L])

        # --- Referencia fina ---
        self.ref_path = self._build_reference_path(base_waypoints, points_per_edge=25)
        self.waypoints = base_waypoints
        self.current_idx = 0  # Índice en ref_path (continuo)

        # --- Parámetros MPC ---
        self.horizon = 12
        self.dt_mpc = 0.07
        self.max_linear = 0.18
        self.max_angular = 1.0
        self.lookahead_base = 0.35
        self.lookahead_min = 0.20

        self.weight_position = 28.0
        self.weight_angle = 1.3
        self.weight_velocity = 0.008
        self.weight_angular_vel = 0.20
        self.weight_curvature = 0.15  # NUEVO: penaliza giros bruscos

        # --- Estado ---
        self.current_pose = None
        self.current_state = np.zeros(3)
        self.initialized = False
        self.start_time = None
        self.prev_position = None
        self.total_distance = 0.0

        # --- Buffers ---
        self.robot_path = []
        self.cmd_log = []
        self.err_log = []
        self.batt_log = []

        # --- Suavizado ---
        self.last_v = 0.0
        self.last_w = 0.0
        self.v_filter = 0.8
        self.w_filter = 0.8

        # --- IO ---
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_dir = os.path.expanduser('~/tb3_runs')
        os.makedirs(self.base_dir, exist_ok=True)
        self.csv_path = os.path.join(self.base_dir, f'compuesta_mpc_{stamp}.csv')

        # --- Pub/Sub ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.ref_path_pub = self.create_publisher(Path, '/reference_path', 10)
        self.prediction_pub = self.create_publisher(Path, '/mpc_prediction', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.batt_sub = self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 10)

        # --- Timers ---
        self.control_timer = self.create_timer(0.07, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_paths)
        self.init_timer = self.create_timer(0.5, self.try_init)

        self.get_logger().info('MPC compuesta- Lookahead continuo')


    # ---------- Inicialización ----------
    def try_init(self):
        if not self.initialized and self.current_pose is not None:
            self.initialized = True
            self.start_time = time.time()
            self.current_state[0] = self.current_pose.position.x
            self.current_state[1] = self.current_pose.position.y
            self.current_state[2] = self._yaw_from_quat(self.current_pose.orientation)
            self.get_logger().info('MPC inicializado')
            self.init_timer.cancel()


    # ---------- Callbacks ----------
    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        if not self.initialized: return

        t = time.time() - self.start_time
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self._yaw_from_quat(self.current_pose.orientation)

        self.current_state = np.array([x, y, theta])
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


    # ---------- MPC con Lookahead Continuo ----------
    def control_loop(self):
        if not self.initialized or self.current_pose is None:
            return

        # --- DETECCIÓN DE FIN ---
        final_x, final_y = self.ref_path[-1]
        dist_to_end = math.hypot(final_x - self.current_state[0], final_y - self.current_state[1])
        if dist_to_end < 0.05:
            self.get_logger().info('META ALCANZADA - Deteniendo')
            self._stop_and_export()
            return

        # --- LOOKAHEAD ---
        lookahead = self._adaptive_lookahead()
        target = self._find_lookahead_point(lookahead)
        if target is None:
            return

        gx, gy = target
        v_cmd, w_cmd = self.mpc_step(gx, gy)

        # --- SUAVIZADO ---
        v_cmd = self.v_filter * v_cmd + (1 - self.v_filter) * self.last_v
        w_cmd = self.w_filter * w_cmd + (1 - self.w_filter) * self.last_w
        self.last_v, self.last_w = v_cmd, w_cmd

        # --- COMANDO ---
        cmd = Twist()
        cmd.linear.x = max(0.05, min(v_cmd, self.max_linear))
        cmd.angular.z = np.clip(w_cmd, -self.max_angular, self.max_angular)
        self.cmd_pub.publish(cmd)

        t = time.time() - self.start_time
        self.cmd_log.append([t, cmd.linear.x, cmd.angular.z])
        self.publish_prediction(cmd.linear.x, cmd.angular.z)

        # --- DEBUG: actualizar índice ---
        self.current_idx = self._find_closest_index(self.current_state[0], self.current_state[1], self.ref_path)


    def _adaptive_lookahead(self):
        # Más lookahead en curva
        curv = self._estimate_curvature(self.current_state[0], self.current_state[1])
        return np.clip(self.lookahead_base - 0.15 * curv, self.lookahead_min, self.lookahead_base)


    def _find_lookahead_point(self, dist):
        idx = self._find_closest_index(self.current_state[0], self.current_state[1], self.ref_path)
        accum = 0.0
        for i in range(idx, len(self.ref_path) - 1):
            p1, p2 = self.ref_path[i], self.ref_path[i+1]
            seg = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if accum + seg >= dist:
                t = (dist - accum) / seg
                return [p1[0] + t*(p2[0]-p1[0]), p1[1] + t*(p2[1]-p1[1])]
            accum += seg
        return self.ref_path[-1]


    def mpc_step(self, gx, gy):
        best_v, best_w = 0.0, 0.0
        best_cost = float('inf')

        v_samples = 6
        w_samples = 9
        v_min = 0.06
        v_max = self.max_linear

        for vi in range(v_samples):
            v = v_min + (vi / max(1, v_samples-1)) * (v_max - v_min)
            for wi in range(w_samples):
                w_center = self._desired_w(gx, gy)
                w_range = self.max_angular * 0.7
                w = w_center + (wi - w_samples//2) * (2 * w_range / w_samples)
                w = np.clip(w, -self.max_angular, self.max_angular)

                cost = self._evaluate_path(v, w, gx, gy)
                if cost < best_cost:
                    best_cost = cost
                    best_v, best_w = v, w

        return best_v, best_w


    def _desired_w(self, gx, gy):
        dx = gx - self.current_state[0]
        dy = gy - self.current_state[1]
        desired = math.atan2(dy, dx)
        alpha = self._wrap(desired - self.current_state[2])
        return np.clip(alpha * 1.6, -self.max_angular, self.max_angular)


    def _evaluate_path(self, v, w, gx, gy):
        state = self.current_state.copy()
        cost = 0.0
        prev_w = w
        for k in range(self.horizon):
            state = self._dynamics(state, v, w, self.dt_mpc)
            pos_err = math.hypot(gx - state[0], gy - state[1])
            ang_err = abs(self._wrap(math.atan2(gy - state[1], gx - state[0]) - state[2]))
            cost += self.weight_position * pos_err
            cost += self.weight_angle * ang_err
            cost += self.weight_angular_vel * abs(w)
            cost += self.weight_curvature * abs(w - prev_w)  # SUAVIZA GIROS
            cost -= self.weight_velocity * v * 0.7
            prev_w = w
        return cost


    def _dynamics(self, state, v, w, dt):
        x, y, theta = state
        return np.array([
            x + v * math.cos(theta) * dt,
            y + v * math.sin(theta) * dt,
            self._wrap(theta + w * dt)
        ])


    def publish_prediction(self, v, w):
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        state = self.current_state.copy()
        for _ in range(self.horizon):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = state[0]
            ps.pose.position.y = state[1]
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
            state = self._dynamics(state, v, w, self.dt_mpc)
        self.prediction_pub.publish(path)


    # ---------- Utils ----------
    def _estimate_curvature(self, x, y):
        if len(self.ref_path) < 3: return 0.0
        idx = self._find_closest_index(x, y, self.ref_path)
        if idx < 1 or idx >= len(self.ref_path) - 1: return 0.0
        p0, p1, p2 = self.ref_path[idx-1:idx+2]
        a = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
        b = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        c = math.hypot(p2[0]-p0[0], p2[1]-p0[1])
        if a == 0 or b == 0: return 0.0
        cos_angle = (a*a + b*b - c*c) / (2*a*b)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        radius = a * b * math.sin(math.acos(cos_angle)) / c if c > 1e-6 else float('inf')
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

    def _yaw_from_quat(self, q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def _wrap(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def _build_reference_path(self, wps, points_per_edge=25):
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

    def _lookup(self, t, series, *idxs):
        if not series: return tuple(float('nan') for _ in idxs)
        row = min(series, key=lambda r: abs(r[0] - t))
        return tuple(row[i] if i < len(row) else float('nan') for i in idxs)

    def _estimate_energy(self):
        if len(self.batt_log) < 2: return None
        E = 0.0
        for i in range(1, len(self.batt_log)):
            t0, V0, I0, _ = self.batt_log[i-1]
            t1, V1, I1, _ = self.batt_log[i]
            if any(math.isnan(v) for v in (V0,V1,I0,I1)): continue
            E += 0.5 * (V0*I0 + V1*I1) * (t1 - t0)
        return E if E > 0 else None


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
        if hasattr(self, 'exported'): return
        self.exported = True
        self._save_results()

    def _save_results(self):
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
        e = self._estimate_energy()
        self.get_logger().info('='*60)
        self.get_logger().info('MPC SUAVE - MÉTRICAS')
        self.get_logger().info(f"Tiempo: {t:.2f}s | Dist: {d:.3f}m | Ref: {ideal:.3f}m")
        self.get_logger().info(f"Error medio: {em:.4f}m | Máx: {ex:.4f}m")
        if e: self.get_logger().info(f"Energía: {e:.1f}J")
        self.get_logger().info('='*60)


def main():
    rclpy.init()
    node = MPCControllerCompuesta()
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
