#!/usr/bin/env python3
"""
TurtleBot3 Autonomous Square Mission - ROS 1 Noetic
====================================================
Misi navigasi otonom membentuk pola persegi sempurna (2x2 meter).
Setelah kembali ke titik awal, robot berhenti secara otomatis.

Logic: Distance-Based Waypoint (BUKAN time.sleep)
- Menghitung Euclidean distance antara posisi saat ini dengan target
- Threshold: target tercapai jika jarak < 0.3 meter
- Frequency: mengirim cmd_vel pada 20Hz
- Safety: berhenti jika emergency stop diaktifkan
"""

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


class SquareMission:
    def __init__(self):
        rospy.init_node('turtlebot3_square_mission', anonymous=False)

        # ==================== PARAMETER ====================
        self.side_length = rospy.get_param('~side_length', 2.0)       # Panjang sisi persegi (meter)
        self.threshold = rospy.get_param('~threshold', 0.3)           # Jarak threshold waypoint (meter)
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)     # Kecepatan maju (m/s)
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)   # Kecepatan belok (rad/s)
        self.angle_threshold = rospy.get_param('~angle_threshold', 0.05)  # Threshold sudut (rad)
        self.rate_hz = rospy.get_param('~rate_hz', 20)                # Frekuensi publish (Hz)

        # ==================== STATE ====================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.emergency_stop = False
        self.mission_complete = False

        # ==================== WAYPOINTS (Persegi 2x2m) ====================
        # Titik awal (0,0) -> kanan -> atas -> kiri -> kembali
        self.waypoints = [
            (self.side_length, 0.0),                          # Waypoint 1: maju 2m
            (self.side_length, self.side_length),              # Waypoint 2: belok kiri, maju 2m
            (0.0, self.side_length),                           # Waypoint 3: belok kiri, maju 2m
            (0.0, 0.0),                                        # Waypoint 4: kembali ke awal
        ]
        self.current_wp_index = 0

        # ==================== PUBLISHER & SUBSCRIBER ====================
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.estop_sub = rospy.Subscriber('/emergency_stop', Bool, self.estop_callback)

        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("=" * 50)
        rospy.loginfo("TurtleBot3 Square Mission - ROS 1 Noetic")
        rospy.loginfo("Pola: Persegi %.1f x %.1f meter" % (self.side_length, self.side_length))
        rospy.loginfo("Threshold jarak: %.2f meter" % self.threshold)
        rospy.loginfo("Frekuensi: %d Hz" % self.rate_hz)
        rospy.loginfo("=" * 50)

    # ==================== CALLBACK ====================
    def odom_callback(self, msg):
        """Callback odometry: ambil posisi dan orientasi saat ini."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.odom_received = True

    def estop_callback(self, msg):
        """Callback emergency stop: hentikan misi jika True."""
        if msg.data and not self.emergency_stop:
            rospy.logwarn("EMERGENCY STOP ACTIVATED! Misi dihentikan.")
            self.emergency_stop = True
            self.stop_robot()

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

    # ==================== NAVIGASI ====================
    def rotate_to_target(self, target_x, target_y):
        """
        Rotate robot menghadap target waypoint.
        Menggunakan proportional control.
        """
        rospy.loginfo("  Rotating ke arah target (%.2f, %.2f)..." % (target_x, target_y))

        while not rospy.is_shutdown() and not self.emergency_stop:
            target_angle = self.angle_to_target(target_x, target_y)
            angle_diff = self.normalize_angle(target_angle - self.current_yaw)

            if abs(angle_diff) < self.angle_threshold:
                self.stop_robot()
                rospy.loginfo("  Rotasi selesai! (error: %.4f rad)" % abs(angle_diff))
                return True

            twist = Twist()
            # Proportional control untuk rotasi
            twist.angular.z = self.angular_speed * (angle_diff / abs(angle_diff))
            # Slow down saat mendekati target angle
            if abs(angle_diff) < 0.3:
                twist.angular.z *= 0.5
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        return False

    def move_to_target(self, target_x, target_y):
        """
        Gerakkan robot menuju target waypoint.
        Menggunakan proportional control untuk linear dan angular.
        """
        rospy.loginfo("  Bergerak menuju (%.2f, %.2f)..." % (target_x, target_y))

        while not rospy.is_shutdown() and not self.emergency_stop:
            distance = self.euclidean_distance(target_x, target_y)

            # Cek apakah sudah sampai (threshold < 0.3m)
            if distance < self.threshold:
                self.stop_robot()
                rospy.loginfo("  Target tercapai! (jarak: %.4f m)" % distance)
                return True

            # Hitung angle correction saat bergerak
            target_angle = self.angle_to_target(target_x, target_y)
            angle_diff = self.normalize_angle(target_angle - self.current_yaw)

            twist = Twist()

            # Linear speed - slow down saat mendekati target
            if distance < 0.5:
                twist.linear.x = self.linear_speed * 0.5
            else:
                twist.linear.x = self.linear_speed

            # Angular correction saat bergerak (keep heading)
            twist.angular.z = 1.5 * angle_diff

            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        return False

    # ==================== MISI UTAMA ====================
    def run_mission(self):
        """Jalankan misi persegi otonom."""

        # Tunggu odometry data
        rospy.loginfo("Menunggu data odometry...")
        while not self.odom_received and not rospy.is_shutdown():
            self.rate.sleep()
        rospy.loginfo("Odometry diterima! Posisi awal: (%.2f, %.2f)" % (self.current_x, self.current_y))

        # Simpan posisi awal sebagai offset
        start_x = self.current_x
        start_y = self.current_y

        # Adjust waypoints relatif ke posisi awal
        adjusted_waypoints = [
            (start_x + wp[0], start_y + wp[1]) for wp in self.waypoints
        ]

        rospy.loginfo("\n===== MEMULAI MISI PERSEGI =====")
        rospy.loginfo("Waypoints:")
        for i, wp in enumerate(adjusted_waypoints):
            rospy.loginfo("  WP%d: (%.2f, %.2f)" % (i + 1, wp[0], wp[1]))
        rospy.loginfo("")

        for i, (target_x, target_y) in enumerate(adjusted_waypoints):
            if rospy.is_shutdown() or self.emergency_stop:
                break

            rospy.loginfo("-" * 40)
            rospy.loginfo("WAYPOINT %d/%d: (%.2f, %.2f)" % (
                i + 1, len(adjusted_waypoints), target_x, target_y))
            rospy.loginfo("Posisi saat ini: (%.2f, %.2f)" % (self.current_x, self.current_y))
            rospy.loginfo("Jarak ke target: %.2f m" % self.euclidean_distance(target_x, target_y))

            # Step 1: Rotate menghadap target
            if not self.rotate_to_target(target_x, target_y):
                rospy.logwarn("Rotasi gagal/dihentikan!")
                break

            # Step 2: Maju ke target
            if not self.move_to_target(target_x, target_y):
                rospy.logwarn("Pergerakan gagal/dihentikan!")
                break

            rospy.loginfo("WAYPOINT %d TERCAPAI!" % (i + 1))
            self.current_wp_index = i + 1

        # Misi selesai
        self.stop_robot()

        if self.emergency_stop:
            rospy.logwarn("\n===== MISI DIHENTIKAN (EMERGENCY STOP) =====")
        elif self.current_wp_index == len(adjusted_waypoints):
            self.mission_complete = True
            rospy.loginfo("\n" + "=" * 50)
            rospy.loginfo("MISI SELESAI! Robot kembali ke titik awal.")
            rospy.loginfo("Posisi akhir: (%.2f, %.2f)" % (self.current_x, self.current_y))
            rospy.loginfo("=" * 50)
        else:
            rospy.logwarn("\n===== MISI TIDAK SELESAI =====")


def main():
    try:
        mission = SquareMission()
        mission.run_mission()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mission interrupted!")


if __name__ == '__main__':
    main()
