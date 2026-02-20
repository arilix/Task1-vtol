#!/usr/bin/env python3
"""
TurtleBot3 Autonomous Square Mission - ROS 2 (Foxy / Humble)
==============================================================
Misi navigasi otonom membentuk pola persegi sempurna (2x2 meter).
Setelah kembali ke titik awal, robot berhenti secara otomatis.

Logic: Distance-Based Waypoint (BUKAN time.sleep)
- Menghitung Euclidean distance antara posisi saat ini dengan target
- Threshold: target tercapai jika jarak < 0.3 meter
- Frequency: mengirim cmd_vel pada 20Hz
- Safety: berhenti jika emergency stop diaktifkan
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# Fungsi untuk konversi quaternion ke euler (tanpa dependency tf)
def euler_from_quaternion(x, y, z, w):
    """Konversi quaternion ke euler angles (roll, pitch, yaw)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(+1.0, t2))
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class SquareMission(Node):
    def __init__(self):
        super().__init__('turtlebot3_square_mission')

        # ==================== PARAMETER ====================
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('threshold', 0.3)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('angle_threshold', 0.05)
        self.declare_parameter('rate_hz', 20)

        self.side_length = self.get_parameter('side_length').value
        self.threshold = self.get_parameter('threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.angle_threshold = self.get_parameter('angle_threshold').value
        self.rate_hz = self.get_parameter('rate_hz').value

        # ==================== STATE ====================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.emergency_stop = False
        self.mission_complete = False

        # ==================== WAYPOINTS (Persegi 2x2m) ====================
        self.waypoints = [
            (self.side_length, 0.0),
            (self.side_length, self.side_length),
            (0.0, self.side_length),
            (0.0, 0.0),
        ]
        self.current_wp_index = 0
        self.adjusted_waypoints = []

        # Mission state machine
        self.state = 'WAIT_ODOM'  # WAIT_ODOM -> ROTATE -> MOVE -> DONE
        self.target_x = 0.0
        self.target_y = 0.0

        # ==================== PUBLISHER & SUBSCRIBER ====================
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.estop_sub = self.create_subscription(Bool, '/emergency_stop', self.estop_callback, qos)

        # Timer untuk control loop (20Hz)
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info("=" * 50)
        self.get_logger().info("TurtleBot3 Square Mission - ROS 2")
        self.get_logger().info(f"Pola: Persegi {self.side_length:.1f} x {self.side_length:.1f} meter")
        self.get_logger().info(f"Threshold jarak: {self.threshold:.2f} meter")
        self.get_logger().info(f"Frekuensi: {self.rate_hz} Hz")
        self.get_logger().info("=" * 50)

    # ==================== CALLBACK ====================
    def odom_callback(self, msg):
        """Callback odometry: ambil posisi dan orientasi saat ini."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.odom_received = True

    def estop_callback(self, msg):
        """Callback emergency stop: hentikan misi jika True."""
        if msg.data and not self.emergency_stop:
            self.get_logger().warn("EMERGENCY STOP ACTIVATED! Misi dihentikan.")
            self.emergency_stop = True
            self.stop_robot()
            self.state = 'STOPPED'

    # ==================== UTILITY ====================
    def euclidean_distance(self, target_x, target_y):
        """Hitung jarak Euclidean antara posisi saat ini dan target."""
        return math.sqrt(
            (target_x - self.current_x) ** 2 +
            (target_y - self.current_y) ** 2
        )

    def angle_to_target(self, target_x, target_y):
        """Hitung sudut dari posisi saat ini ke target."""
        return math.atan2(
            target_y - self.current_y,
            target_x - self.current_x
        )

    def normalize_angle(self, angle):
        """Normalisasi sudut ke range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        """Kirim perintah berhenti ke robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    # ==================== CONTROL LOOP (State Machine) ====================
    def control_loop(self):
        """
        Main control loop - dijalankan pada frekuensi rate_hz.
        Menggunakan state machine untuk navigasi.
        """
        if self.emergency_stop or self.state in ('DONE', 'STOPPED'):
            return

        # ---------- STATE: WAIT_ODOM ----------
        if self.state == 'WAIT_ODOM':
            if not self.odom_received:
                return

            self.get_logger().info(f"Odometry diterima! Posisi awal: ({self.current_x:.2f}, {self.current_y:.2f})")

            # Adjust waypoints relatif ke posisi awal
            start_x = self.current_x
            start_y = self.current_y
            self.adjusted_waypoints = [
                (start_x + wp[0], start_y + wp[1]) for wp in self.waypoints
            ]

            self.get_logger().info("\n===== MEMULAI MISI PERSEGI =====")
            for i, wp in enumerate(self.adjusted_waypoints):
                self.get_logger().info(f"  WP{i+1}: ({wp[0]:.2f}, {wp[1]:.2f})")

            # Set target pertama
            self.current_wp_index = 0
            self.target_x, self.target_y = self.adjusted_waypoints[0]
            self.state = 'ROTATE'

            self.get_logger().info("-" * 40)
            self.get_logger().info(f"WAYPOINT {self.current_wp_index+1}/{len(self.adjusted_waypoints)}: "
                                   f"({self.target_x:.2f}, {self.target_y:.2f})")

        # ---------- STATE: ROTATE ----------
        elif self.state == 'ROTATE':
            target_angle = self.angle_to_target(self.target_x, self.target_y)
            angle_diff = self.normalize_angle(target_angle - self.current_yaw)

            if abs(angle_diff) < self.angle_threshold:
                self.stop_robot()
                self.get_logger().info(f"  Rotasi selesai! (error: {abs(angle_diff):.4f} rad)")
                self.state = 'MOVE'
                self.get_logger().info(f"  Bergerak menuju ({self.target_x:.2f}, {self.target_y:.2f})...")
                return

            twist = Twist()
            twist.angular.z = self.angular_speed * (1.0 if angle_diff > 0 else -1.0)
            if abs(angle_diff) < 0.3:
                twist.angular.z *= 0.5
            self.cmd_vel_pub.publish(twist)

        # ---------- STATE: MOVE ----------
        elif self.state == 'MOVE':
            distance = self.euclidean_distance(self.target_x, self.target_y)

            # Cek apakah sudah sampai (threshold < 0.3m)
            if distance < self.threshold:
                self.stop_robot()
                self.get_logger().info(f"  Target tercapai! (jarak: {distance:.4f} m)")
                self.get_logger().info(f"WAYPOINT {self.current_wp_index+1} TERCAPAI!")

                # Next waypoint
                self.current_wp_index += 1

                if self.current_wp_index >= len(self.adjusted_waypoints):
                    # Misi selesai
                    self.mission_complete = True
                    self.state = 'DONE'
                    self.get_logger().info("\n" + "=" * 50)
                    self.get_logger().info("MISI SELESAI! Robot kembali ke titik awal.")
                    self.get_logger().info(f"Posisi akhir: ({self.current_x:.2f}, {self.current_y:.2f})")
                    self.get_logger().info("=" * 50)
                    return

                # Set target berikutnya
                self.target_x, self.target_y = self.adjusted_waypoints[self.current_wp_index]
                self.state = 'ROTATE'
                self.get_logger().info("-" * 40)
                self.get_logger().info(f"WAYPOINT {self.current_wp_index+1}/{len(self.adjusted_waypoints)}: "
                                       f"({self.target_x:.2f}, {self.target_y:.2f})")
                return

            # Hitung angle correction saat bergerak
            target_angle = self.angle_to_target(self.target_x, self.target_y)
            angle_diff = self.normalize_angle(target_angle - self.current_yaw)

            twist = Twist()
            # Linear speed - slow down saat mendekati target
            if distance < 0.5:
                twist.linear.x = self.linear_speed * 0.5
            else:
                twist.linear.x = self.linear_speed

            # Angular correction saat bergerak
            twist.angular.z = 1.5 * angle_diff

            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SquareMission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
