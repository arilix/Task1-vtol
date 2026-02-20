/**
 * TurtleBot3 Autonomous Square Mission - ROS 1 Noetic (C++)
 * ===========================================================
 * Misi navigasi otonom membentuk pola persegi sempurna (2x2 meter).
 * Setelah kembali ke titik awal, robot berhenti secara otomatis.
 *
 * Logic: Distance-Based Waypoint (BUKAN time.sleep)
 * - Menghitung Euclidean distance antara posisi saat ini dengan target
 * - Threshold: target tercapai jika jarak < 0.3 meter
 * - Frequency: mengirim cmd_vel pada 20Hz
 * - Safety: berhenti jika emergency stop diaktifkan
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>
#include <utility>

class SquareMission
{
public:
    SquareMission() : nh_("~")
    {
        // ==================== PARAMETER ====================
        nh_.param("side_length", side_length_, 2.0);
        nh_.param("threshold", threshold_, 0.3);
        nh_.param("linear_speed", linear_speed_, 0.2);
        nh_.param("angular_speed", angular_speed_, 0.5);
        nh_.param("angle_threshold", angle_threshold_, 0.05);
        nh_.param("rate_hz", rate_hz_, 20);

        // ==================== STATE ====================
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        odom_received_ = false;
        emergency_stop_ = false;
        mission_complete_ = false;
        current_wp_index_ = 0;

        // ==================== WAYPOINTS (Persegi 2x2m) ====================
        waypoints_.push_back(std::make_pair(side_length_, 0.0));
        waypoints_.push_back(std::make_pair(side_length_, side_length_));
        waypoints_.push_back(std::make_pair(0.0, side_length_));
        waypoints_.push_back(std::make_pair(0.0, 0.0));

        // ==================== PUBLISHER & SUBSCRIBER ====================
        cmd_vel_pub_ = nh_global_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        odom_sub_ = nh_global_.subscribe("/odom", 10, &SquareMission::odomCallback, this);
        estop_sub_ = nh_global_.subscribe("/emergency_stop", 10, &SquareMission::estopCallback, this);

        ROS_INFO("==================================================");
        ROS_INFO("TurtleBot3 Square Mission - ROS 1 Noetic (C++)");
        ROS_INFO("Pola: Persegi %.1f x %.1f meter", side_length_, side_length_);
        ROS_INFO("Threshold jarak: %.2f meter", threshold_);
        ROS_INFO("Frekuensi: %d Hz", rate_hz_);
        ROS_INFO("==================================================");
    }

    // ==================== CALLBACK ====================
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        odom_received_ = true;
    }

    void estopCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data && !emergency_stop_)
        {
            ROS_WARN("EMERGENCY STOP ACTIVATED! Misi dihentikan.");
            emergency_stop_ = true;
            stopRobot();
        }
    }

    // ==================== UTILITY ====================
    double euclideanDistance(double target_x, double target_y)
    {
        return std::sqrt(
            std::pow(target_x - current_x_, 2) +
            std::pow(target_y - current_y_, 2)
        );
    }

    double angleToTarget(double target_x, double target_y)
    {
        return std::atan2(target_y - current_y_, target_x - current_x_);
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void stopRobot()
    {
        geometry_msgs::Twist twist;
        cmd_vel_pub_.publish(twist);
    }

    // ==================== NAVIGASI ====================
    bool rotateToTarget(double target_x, double target_y, ros::Rate& rate)
    {
        ROS_INFO("  Rotating ke arah target (%.2f, %.2f)...", target_x, target_y);

        while (ros::ok() && !emergency_stop_)
        {
            ros::spinOnce();

            double target_angle = angleToTarget(target_x, target_y);
            double angle_diff = normalizeAngle(target_angle - current_yaw_);

            if (std::abs(angle_diff) < angle_threshold_)
            {
                stopRobot();
                ROS_INFO("  Rotasi selesai! (error: %.4f rad)", std::abs(angle_diff));
                return true;
            }

            geometry_msgs::Twist twist;
            twist.angular.z = angular_speed_ * (angle_diff > 0 ? 1.0 : -1.0);
            if (std::abs(angle_diff) < 0.3)
                twist.angular.z *= 0.5;

            cmd_vel_pub_.publish(twist);
            rate.sleep();
        }
        return false;
    }

    bool moveToTarget(double target_x, double target_y, ros::Rate& rate)
    {
        ROS_INFO("  Bergerak menuju (%.2f, %.2f)...", target_x, target_y);

        while (ros::ok() && !emergency_stop_)
        {
            ros::spinOnce();

            double distance = euclideanDistance(target_x, target_y);

            if (distance < threshold_)
            {
                stopRobot();
                ROS_INFO("  Target tercapai! (jarak: %.4f m)", distance);
                return true;
            }

            double target_angle = angleToTarget(target_x, target_y);
            double angle_diff = normalizeAngle(target_angle - current_yaw_);

            geometry_msgs::Twist twist;

            if (distance < 0.5)
                twist.linear.x = linear_speed_ * 0.5;
            else
                twist.linear.x = linear_speed_;

            twist.angular.z = 1.5 * angle_diff;

            cmd_vel_pub_.publish(twist);
            rate.sleep();
        }
        return false;
    }

    // ==================== MISI UTAMA ====================
    void runMission()
    {
        ros::Rate rate(rate_hz_);

        // Tunggu odometry
        ROS_INFO("Menunggu data odometry...");
        while (!odom_received_ && ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Odometry diterima! Posisi awal: (%.2f, %.2f)", current_x_, current_y_);

        // Simpan posisi awal sebagai offset
        double start_x = current_x_;
        double start_y = current_y_;

        // Adjust waypoints relatif ke posisi awal
        std::vector<std::pair<double, double>> adjusted_waypoints;
        for (const auto& wp : waypoints_)
        {
            adjusted_waypoints.push_back(
                std::make_pair(start_x + wp.first, start_y + wp.second));
        }

        ROS_INFO("\n===== MEMULAI MISI PERSEGI =====");
        ROS_INFO("Waypoints:");
        for (size_t i = 0; i < adjusted_waypoints.size(); ++i)
        {
            ROS_INFO("  WP%zu: (%.2f, %.2f)", i + 1,
                     adjusted_waypoints[i].first, adjusted_waypoints[i].second);
        }

        for (size_t i = 0; i < adjusted_waypoints.size(); ++i)
        {
            if (!ros::ok() || emergency_stop_)
                break;

            double target_x = adjusted_waypoints[i].first;
            double target_y = adjusted_waypoints[i].second;

            ROS_INFO("----------------------------------------");
            ROS_INFO("WAYPOINT %zu/%zu: (%.2f, %.2f)",
                     i + 1, adjusted_waypoints.size(), target_x, target_y);
            ROS_INFO("Posisi saat ini: (%.2f, %.2f)", current_x_, current_y_);
            ROS_INFO("Jarak ke target: %.2f m", euclideanDistance(target_x, target_y));

            // Step 1: Rotate menghadap target
            if (!rotateToTarget(target_x, target_y, rate))
            {
                ROS_WARN("Rotasi gagal/dihentikan!");
                break;
            }

            // Step 2: Maju ke target
            if (!moveToTarget(target_x, target_y, rate))
            {
                ROS_WARN("Pergerakan gagal/dihentikan!");
                break;
            }

            ROS_INFO("WAYPOINT %zu TERCAPAI!", i + 1);
            current_wp_index_ = i + 1;
        }

        // Misi selesai
        stopRobot();

        if (emergency_stop_)
        {
            ROS_WARN("\n===== MISI DIHENTIKAN (EMERGENCY STOP) =====");
        }
        else if (current_wp_index_ == static_cast<int>(adjusted_waypoints.size()))
        {
            mission_complete_ = true;
            ROS_INFO("\n==================================================");
            ROS_INFO("MISI SELESAI! Robot kembali ke titik awal.");
            ROS_INFO("Posisi akhir: (%.2f, %.2f)", current_x_, current_y_);
            ROS_INFO("==================================================");
        }
        else
        {
            ROS_WARN("\n===== MISI TIDAK SELESAI =====");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_global_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber estop_sub_;

    // Parameter
    double side_length_;
    double threshold_;
    double linear_speed_;
    double angular_speed_;
    double angle_threshold_;
    int rate_hz_;

    // State
    double current_x_;
    double current_y_;
    double current_yaw_;
    bool odom_received_;
    bool emergency_stop_;
    bool mission_complete_;
    int current_wp_index_;

    // Waypoints
    std::vector<std::pair<double, double>> waypoints_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlebot3_square_mission_cpp");

    try
    {
        SquareMission mission;
        mission.runMission();
    }
    catch (const ros::Exception& e)
    {
        ROS_ERROR("Mission error: %s", e.what());
    }

    return 0;
}
