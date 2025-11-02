#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import BatteryState
import math, numpy as np, time, json, csv, os
from datetime import datetime

class SimplePIDController(Node):
    def __init__(self):
        super().__init__('simple_pid_controller')

        # --- Parámetros ---
        self.declare_parameter('side_m', 1.75)
        self.declare_parameter('kp', 2.3)
        self.declare_parameter('ki', 0.3)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('max_linear', 0.18)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('waypoint_threshold', 0.05)
        self.declare_parameter('rate_hz', 100.0)

        L = float(self.get_parameter('side_m').value)
        self.waypoints = [[0.0,0.0],[L,0.0],[L,L],[0.0,L],[0.0,0.0]]

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.waypoint_threshold = float(self.get_parameter('waypoint_threshold').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # --- Estado ---
        self.current_pose = None
        self.current_waypoint_idx = 0
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.started = False
        self.init_time = self.get_clock().now()

        # --- Buffers de exportación ---
        self.ref_path = self._build_reference_path(self.waypoints, 40)
        self.robot_path = []     # [t,x,y]
        self.cmd_log   = []      # [t,v,w]
        self.err_log   = []      # [t,err]
        self.batt_log  = []      # [t,V,I,pct]

        self.total_distance = 0.0
        self.prev_position = None

        self.start_time = None
        self.end_time = None
        self.metrics_printed = False

        # --- IO (archivos) ---
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_dir = os.path.expanduser('~/tb3_runs')
        os.makedirs(self.base_dir, exist_ok=True)
        self.csv_path  = os.path.join(self.base_dir, f'pid_run_{stamp}.csv')

        # --- Pub/Sub ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.ref_path_pub   = self.create_publisher(Path, '/reference_path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.batt_sub = self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 10)

        # --- Timers ---
        self.dt = max(1.0/self.rate_hz, 0.005)
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.path_timer    = self.create_timer(0.5, self.publish_paths)

        self.get_logger().info('PID con exportación listo')

    # ---------- Callbacks ----------
    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

        if self.started:
            t = time.time() - self.start_time
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            self.robot_path.append([t, x, y])

            if self.prev_position is not None:
                self.total_distance += math.hypot(x - self.prev_position[0],
                                                  y - self.prev_position[1])
            self.prev_position = [x, y]

            rx, ry = self._closest_on_polyline(x, y, self.ref_path)
            err = math.hypot(rx - x, ry - y)
            self.err_log.append([t, err])

    def batt_cb(self, msg: BatteryState):
        if not self.started: return
        t = time.time() - self.start_time
        V = float(msg.voltage) if msg.voltage is not None else float('nan')
        I = float(msg.current) if msg.current is not None else float('nan')
        P = float(msg.percentage) if msg.percentage is not None else float('nan')
        self.batt_log.append([t, V, I, P])

    # ---------- Control ----------
    def control_loop(self):
        if not self.started:
            elapsed = (self.get_clock().now() - self.init_time).nanoseconds / 1e9
            if elapsed < 3.0:
                return
            self.started = True
            self.start_time = time.time()
            self.get_logger().info('Iniciando seguimiento del cuadrado (PID)')

        if self.current_pose is None:
            return

        # Fin de ruta
        if self.current_waypoint_idx >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            if not self.metrics_printed:
                self.end_time = time.time()
                self._export_and_finish()
                self.metrics_printed = True
            return

        # Pose actual
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        q = self.current_pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Objetivo actual
        gx, gy = self.waypoints[self.current_waypoint_idx]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            self.integral_error = 0.0
            return

        desired = math.atan2(dy, dx)
        angle_error = desired - theta
        while angle_error > math.pi:  angle_error -= 2*math.pi
        while angle_error < -math.pi: angle_error += 2*math.pi

        self.integral_error += angle_error * self.dt
        self.integral_error = float(np.clip(self.integral_error, -0.5, 0.5))
        derivative = (angle_error - self.prev_error) / self.dt
        self.prev_error = angle_error

        w = self.angular_gain * (self.kp*angle_error + self.ki*self.integral_error + self.kd*derivative)
        v = self.max_linear * (1 - min(abs(angle_error)/(math.pi/2), 1.0)) * min(1.0, dist/0.5)
        v = float(np.clip(v, 0.0, self.max_linear))
        w = float(np.clip(w, -self.max_angular, self.max_angular))

        cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # log comandos
        t = time.time() - self.start_time
        self.cmd_log.append([t, v, w])

    # ---------- Paths ----------
    def publish_paths(self):
        # referencia
        ref = Path(); ref.header.frame_id = "odom"; ref.header.stamp = self.get_clock().now().to_msg()
        for (rx, ry) in self.ref_path:
            ps = PoseStamped(); ps.header = ref.header
            ps.pose.position.x = rx; ps.pose.position.y = ry; ps.pose.orientation.w = 1.0
            ref.poses.append(ps)
        self.ref_path_pub.publish(ref)

        # robot
        if len(self.robot_path) > 1:
            rp = Path(); rp.header.frame_id = "odom"; rp.header.stamp = self.get_clock().now().to_msg()
            for (_, x, y) in self.robot_path:
                ps = PoseStamped(); ps.header = rp.header
                ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.orientation.w = 1.0
                rp.poses.append(ps)
            self.robot_path_pub.publish(rp)

    # ---------- Exportación ----------
    def _export_and_finish(self):
        elapsed = self.end_time - self.start_time if self.start_time else 0.0
        ideal_len = self._polyline_length(self.ref_path)
        avg_err = float(np.mean([e for _,e in self.err_log])) if self.err_log else 0.0
        max_err = float(np.max([e for _,e in self.err_log])) if self.err_log else 0.0

        data = {
            "meta": {
                "controller": "pid",
                "side_m": float(self.get_parameter('side_m').value),
                "kp": self.kp, "ki": self.ki, "kd": self.kd, "angular_gain": self.angular_gain,
                "max_linear": self.max_linear, "max_angular": self.max_angular,
                "waypoint_threshold": self.waypoint_threshold,
                "rate_hz": self.rate_hz,
                "elapsed_s": elapsed
            },
            "reference_path": [{"x":x,"y":y} for (x,y) in self.ref_path],
            "robot_path": [{"t":t,"x":x,"y":y} for (t,x,y) in self.robot_path],
            "commands": [{"t":t,"v":v,"w":w} for (t,v,w) in self.cmd_log],
            "battery": [{"t":t,"voltage":V,"current":I,"percentage":P} for (t,V,I,P) in self.batt_log],
            "metrics": {
                "elapsed_s": elapsed,
                "path_length_real_m": self.total_distance,
                "path_length_ref_m": ideal_len,
                "mean_tracking_error_m": avg_err,
                "max_tracking_error_m": max_err,
                "energy_joules_est": self._estimate_energy_j()
            }
        }

        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t','x','y','ref_x','ref_y','err','v','w','voltage','current','percentage'])
            for (t,x,y) in self.robot_path:
                rx,ry = self._closest_on_polyline(x,y,self.ref_path)
                err = math.hypot(rx-x, ry-y)
                v,w_cmd = self._nearest_series(t, self.cmd_log, (1,2))
                V,I,P = self._nearest_series(t, self.batt_log, (1,2,3))
                w.writerow([f'{t:.3f}',f'{x:.4f}',f'{y:.4f}',
                            f'{rx:.4f}',f'{ry:.4f}',f'{err:.4f}',
                            f'{v:.3f}',f'{w_cmd:.3f}',
                            f'{V:.3f}' if V==V else '',
                            f'{I:.3f}' if I==I else '',
                            f'{P:.3f}' if P==P else ''])
        self.get_logger().info(f"✓ CSV: {self.csv_path}")
        self._print_summary(data['metrics'])

    # ---------- Utils ----------
    def _build_reference_path(self, wps, pts_per_edge=40):
        pts=[]
        for i in range(len(wps)-1):
            x0,y0=wps[i]; x1,y1=wps[i+1]
            for j in range(pts_per_edge):
                t=j/pts_per_edge
                pts.append([x0+t*(x1-x0), y0+t*(y1-y0)])
        pts.append(wps[-1][:]); return pts

    def _polyline_length(self, pts):
        s=0.0
        for i in range(1,len(pts)):
            x0,y0=pts[i-1]; x1,y1=pts[i]
            s+=math.hypot(x1-x0,y1-y0)
        return s

    def _closest_on_polyline(self, x,y,pts):
        best_d=float('inf'); best=(pts[0][0],pts[0][1])
        for i in range(1,len(pts)):
            x0,y0=pts[i-1]; x1,y1=pts[i]
            ux,uy=x1-x0,y1-y0; L2=ux*ux+uy*uy
            if L2==0: px,py=x0,y0
            else:
                t=max(0.0,min(1.0, ((x-x0)*ux+(y-y0)*uy)/L2))
                px,py=x0+t*ux, y0+t*uy
            d=(px-x)**2+(py-y)**2
            if d<best_d: best_d=d; best=(px,py)
        return best

    def _nearest_series(self, t, series, idxs):
        if not series: return tuple(float('nan') for _ in idxs)
        row=min(series, key=lambda r: abs(r[0]-t))
        return tuple(row[i] if i<len(row) else float('nan') for i in idxs)

    def _estimate_energy_j(self):
        if len(self.batt_log)<2: return None
        E=0.0
        for i in range(1,len(self.batt_log)):
            t0,V0,I0,_=self.batt_log[i-1]; t1,V1,I1,_=self.batt_log[i]
            if any(math.isnan(v) for v in (V0,V1,I0,I1)): continue
            P0=V0*I0; P1=V1*I1; E+=0.5*(P0+P1)*(t1-t0)
        return E if E>0 else None

    def _print_summary(self, m):
        self.get_logger().info('='*60)
        self.get_logger().info('MÉTRICAS FINALES - PID')
        self.get_logger().info(f"Tiempo total: {m['elapsed_s']:.2f} s")
        self.get_logger().info(f"Longitud real: {m['path_length_real_m']:.3f} m")
        self.get_logger().info(f"Longitud ref.: {m['path_length_ref_m']:.3f} m")
        self.get_logger().info(f"Error medio: {m['mean_tracking_error_m']:.4f} m")
        self.get_logger().info(f"Error máx.: {m['max_tracking_error_m']:.4f} m")
        if m['energy_joules_est'] is not None:
            self.get_logger().info(f"Energía estimada: {m['energy_joules_est']:.1f} J")
        self.get_logger().info('='*60)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node._export_and_finish()
        except Exception: pass
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
