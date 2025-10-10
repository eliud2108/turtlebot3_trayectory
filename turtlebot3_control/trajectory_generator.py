import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math
import time

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.publisher_ = self.create_publisher(Pose2D, '/desired_pose', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # publica cada 0.5 s

        # Definir la trayectoria cuadrada: 4 puntos de 1m de lado
        self.trajectory = [
            (0.0, 0.0, 0.0),           # inicio
            (1.0, 0.0, 0.0),           # avanzar en x
            (1.0, 1.0, math.pi/2),     # avanzar en y y girar 90°
            (0.0, 1.0, math.pi),       # regresar en x
            (0.0, 0.0, -math.pi/2)     # regresar en y y girar a posición inicial
        ]

        self.current_index = 0
        self.get_logger().info("Trajectory Generator iniciado. Publicando en /desired_pose")

    def timer_callback(self):
        """Publica el siguiente punto de la trayectoria."""
        pose = Pose2D()
        pose.x, pose.y, pose.theta = self.trajectory[self.current_index]

        # Publicar la pose
        self.publisher_.publish(pose)
        self.get_logger().info(f"Publicando punto: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}")

        # Avanzar al siguiente punto
        self.current_index += 1
        if self.current_index >= len(self.trajectory):
            self.get_logger().info("Trayectoria completada, reiniciando desde el inicio...")
            self.current_index = 0


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

