/**
 * TurtleBot3 Autonomous Square Mission - ROS 2 (Foxy / Humble) (C++)
 * ====================================================================
 * Misi navigasi otonom membentuk pola persegi sempurna (2x2 meter).
 * Setelah kembali ke titik awal, robot berhenti secara otomatis.
 *
 * Logic: Distance-Based Waypoint (BUKAN time.sleep)
 * - Menghitung Euclidean distance antara posisi saat ini dengan target
 * - Threshold: target tercapai jika jarak < 0.3 meter
 * - Frequency: mengirim cmd_vel pada 20Hz
 * - Safety: berhenti jika emergency stop diaktifkan
 *
 * Arsitektur: State Machine via Timer Callback
 * States: WAIT_ODOM -> ROTATE -> MOVE -> DONE / STOPPED
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <vector>
#include <utility>
#include <string>

// Fungsi untuk konversi quaternion ke euler (tanpa dependency tf2)
static void eulerFromQuaternion(double x, double y, double z, double w,
                                double& roll, double& pitch, double& yaw)
{
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = std::max(-1.0, std::min(+1.0, t2));
    pitch = std::asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(t3, t4);
}


class SquareMission : public rclcpp::Node
{
public:
    SquareMission() : Node("turtlebot3_square_mission_cpp")
    {
        // ==================== PARAMETER ====================
        this->declare_parameter("side_length", 2.0);
        this->declare_parameter("threshold", 0.3);
        this->declare_parameter("linear_speed", 0.2);
        this->declare_parameter("angular_speed", 0.5);
        this->declare_parameter("angle_threshold", 0.05);
        this->declare_parameter("rate_hz", 20);

        side_length_ = this->get_parameter("side_length").as_double();
        threshold_ = this->get_parameter("threshold").as_double();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        angle_threshold_ = this->get_parameter("angle_threshold").as_double();
        rate_hz_ = this->get_parameter("rate_hz").as_int();

        // ==================== STATE ====================
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        odom_received_ = false;
        emergency_stop_ = false;
        mission_complete_ = false;
        current_wp_index_ = 0;
        state_ = "WAIT_ODOM";

        target_x_ = 0.0;
        target_y_ = 0.0;

        // ==================== WAYPOINTS (Persegi 2x2m) ====================
        waypoints_.push_back(std::make_pair(side_length_, 0.0));
        waypoints_.push_back(std::make_pair(side_length_, side_length_));
        waypoints_.push_back(std::make_pair(0.0, side_length_));
        waypoints_.push_back(std::make_pair(0.0, 0.0));

        // ==================== PUBLISHER & SUBSCRIBER ====================
        auto qos = rclcpp::QoS(10).reliable();

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos,
            std::bind(&SquareMission::odomCallback, this, std::placeholders::_1));

        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", qos,
            std::bind(&SquareMission::estopCallback, this, std::placeholders::_1));

        // Timer untuk control loop
        double timer_period = 1.0 / static_cast<double>(rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period),
            std::bind(&SquareMission::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), "TurtleBot3 Square Mission - ROS 2 (C++)");
        RCLCPP_INFO(this->get_logger(), "Pola: Persegi %.1f x %.1f meter", side_length_, side_length_);
        RCLCPP_INFO(this->get_logger(), "Threshold jarak: %.2f meter", threshold_);
        RCLCPP_INFO(this->get_logger(), "Frekuensi: %d Hz", rate_hz_);
        RCLCPP_INFO(this->get_logger(), "==================================================");
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist twist;
        cmd_vel_pub_->publish(twist);
    }

private:
    // ==================== CALLBACK ====================
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        double roll, pitch;
        eulerFromQuaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w,
            roll, pitch, current_yaw_);

        odom_received_ = true;
    }

    void estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !emergency_stop_)
        {
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED! Misi dihentikan.");
            emergency_stop_ = true;
            stopRobot();
            state_ = "STOPPED";
        }
    }

    // ==================== UTILITY ====================
    double euclideanDistance(double target_x, double target_y)
    {
        return std::sqrt(
            std::pow(target_x - current_x_, 2) +
            std::pow(target_y - current_y_, 2));
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

    // ==================== CONTROL LOOP (State Machine) ====================
    void controlLoop()
    {
        if (emergency_stop_ || state_ == "DONE" || state_ == "STOPPED")
            return;

        // ---------- STATE: WAIT_ODOM ----------
        if (state_ == "WAIT_ODOM")
        {
            if (!odom_received_)
                return;

            RCLCPP_INFO(this->get_logger(),
                        "Odometry diterima! Posisi awal: (%.2f, %.2f)", current_x_, current_y_);

            // Adjust waypoints relatif ke posisi awal
            double start_x = current_x_;
            double start_y = current_y_;
            adjusted_waypoints_.clear();
            for (const auto& wp : waypoints_)
            {
                adjusted_waypoints_.push_back(
                    std::make_pair(start_x + wp.first, start_y + wp.second));
            }

            RCLCPP_INFO(this->get_logger(), "\n===== MEMULAI MISI PERSEGI =====");
            for (size_t i = 0; i < adjusted_waypoints_.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "  WP%zu: (%.2f, %.2f)",
                            i + 1, adjusted_waypoints_[i].first, adjusted_waypoints_[i].second);
            }

            // Set target pertama
            current_wp_index_ = 0;
            target_x_ = adjusted_waypoints_[0].first;
            target_y_ = adjusted_waypoints_[0].second;
            state_ = "ROTATE";

            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            RCLCPP_INFO(this->get_logger(), "WAYPOINT %d/%zu: (%.2f, %.2f)",
                        current_wp_index_ + 1, adjusted_waypoints_.size(), target_x_, target_y_);
        }
        // ---------- STATE: ROTATE ----------
        else if (state_ == "ROTATE")
        {
            double target_angle = angleToTarget(target_x_, target_y_);
            double angle_diff = normalizeAngle(target_angle - current_yaw_);

            if (std::abs(angle_diff) < angle_threshold_)
            {
                stopRobot();
                RCLCPP_INFO(this->get_logger(),
                            "  Rotasi selesai! (error: %.4f rad)", std::abs(angle_diff));
                state_ = "MOVE";
                RCLCPP_INFO(this->get_logger(),
                            "  Bergerak menuju (%.2f, %.2f)...", target_x_, target_y_);
                return;
            }

            geometry_msgs::msg::Twist twist;
            twist.angular.z = angular_speed_ * (angle_diff > 0 ? 1.0 : -1.0);
            if (std::abs(angle_diff) < 0.3)
                twist.angular.z *= 0.5;

            cmd_vel_pub_->publish(twist);
        }
        // ---------- STATE: MOVE ----------
        else if (state_ == "MOVE")
        {
            double distance = euclideanDistance(target_x_, target_y_);

            // Cek apakah sudah sampai
            if (distance < threshold_)
            {
                stopRobot();
                RCLCPP_INFO(this->get_logger(),
                            "  Target tercapai! (jarak: %.4f m)", distance);
                RCLCPP_INFO(this->get_logger(),
                            "WAYPOINT %d TERCAPAI!", current_wp_index_ + 1);

                // Next waypoint
                current_wp_index_++;

                if (current_wp_index_ >= static_cast<int>(adjusted_waypoints_.size()))
                {
                    // Misi selesai
                    mission_complete_ = true;
                    state_ = "DONE";
                    RCLCPP_INFO(this->get_logger(), "\n==================================================");
                    RCLCPP_INFO(this->get_logger(), "MISI SELESAI! Robot kembali ke titik awal.");
                    RCLCPP_INFO(this->get_logger(),
                                "Posisi akhir: (%.2f, %.2f)", current_x_, current_y_);
                    RCLCPP_INFO(this->get_logger(), "==================================================");
                    return;
                }

                // Set target berikutnya
                target_x_ = adjusted_waypoints_[current_wp_index_].first;
                target_y_ = adjusted_waypoints_[current_wp_index_].second;
                state_ = "ROTATE";
                RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                RCLCPP_INFO(this->get_logger(), "WAYPOINT %d/%zu: (%.2f, %.2f)",
                            current_wp_index_ + 1, adjusted_waypoints_.size(),
                            target_x_, target_y_);
                return;
            }

            // Hitung angle correction saat bergerak
            double target_angle = angleToTarget(target_x_, target_y_);
            double angle_diff = normalizeAngle(target_angle - current_yaw_);

            geometry_msgs::msg::Twist twist;
            if (distance < 0.5)
                twist.linear.x = linear_speed_ * 0.5;
            else
                twist.linear.x = linear_speed_;

            twist.angular.z = 1.5 * angle_diff;

            cmd_vel_pub_->publish(twist);
        }
    }

    // ==================== MEMBER VARIABLES ====================
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

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
    std::string state_;

    double target_x_;
    double target_y_;

    // Waypoints
    std::vector<std::pair<double, double>> waypoints_;
    std::vector<std::pair<double, double>> adjusted_waypoints_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SquareMission>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    }

    node->stopRobot();
    rclcpp::shutdown();
    return 0;
}
