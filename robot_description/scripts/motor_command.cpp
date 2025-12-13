#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MotorCommandNode : public rclcpp::Node
{
public:
    MotorCommandNode() : Node("motor_command_node")
    {
        // Declare parameters
        this->declare_parameter<double>("wheel_radius", 0.1);
        this->declare_parameter<double>("wheel_separation", 0.45);
        this->declare_parameter<double>("rpm_to_rad_per_sec", 0.10472); // Conversion factor: 1 RPM = 2π/60 rad/s
        
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        rpm_to_rad_per_sec_ = this->get_parameter("rpm_to_rad_per_sec").as_double();

        // Publishers
        left_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_motor_rpm", 10);
        right_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_motor_rpm", 10);

        // Subscriber to cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&MotorCommandNode::cmdVelCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Motor Command Node started.");
        RCLCPP_INFO(this->get_logger(), "Wheel radius: %f, Wheel separation: %f", wheel_radius_, wheel_separation_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract linear and angular velocities
        double v = msg->linear.x;  // Linear velocity in m/s
        double w = msg->angular.z; // Angular velocity in rad/s

        // For skid-steering robot, convert to wheel velocities
        // v_left = v - (w * wheel_separation / 2)
        // v_right = v + (w * wheel_separation / 2)
        double v_left = v - (w * wheel_separation_ / 2.0);
        double v_right = v + (w * wheel_separation_ / 2.0);

        // Convert linear velocities to angular velocities (rad/s)
        double w_left_rad = v_left / wheel_radius_;
        double w_right_rad = v_right / wheel_radius_;

        // Convert rad/s to RPM
        // RPM = (rad/s) * (60 / 2π) = (rad/s) / 0.10472
        double left_rpm = w_left_rad / rpm_to_rad_per_sec_;
        double right_rpm = w_right_rad / rpm_to_rad_per_sec_;

        // Create and publish messages
        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;

        left_msg.data = left_rpm;
        right_msg.data = right_rpm;

        left_motor_pub_->publish(left_msg);
        right_motor_pub_->publish(right_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    double wheel_radius_;
    double wheel_separation_;
    double rpm_to_rad_per_sec_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
