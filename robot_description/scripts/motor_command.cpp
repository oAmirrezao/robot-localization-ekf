#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MotorCommandNode : public rclcpp::Node
{
public:
    MotorCommandNode() : Node("motor_command_node")
    {
        // Parameters (مطابق با Prediction Node)
        this->declare_parameter("wheel_separation", 0.45);
        this->declare_parameter("wheel_radius", 0.1);

        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();

        // Publishers
        left_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_motor_rpm", 10);
        right_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_motor_rpm", 10);

        // Subscriber to cmd_vel (تغییر مهم: دریافت تویست به جای آرایه)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&MotorCommandNode::cmdVelCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Motor Command Node started (Kinematics enabled).");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        // Differential Drive Inverse Kinematics
        // محاسبه سرعت خطی هر چرخ
        double left_vel = linear - (angular * wheel_separation_ / 2.0);
        double right_vel = linear + (angular * wheel_separation_ / 2.0);

        // تبدیل سرعت خطی (m/s) به سرعت زاویه‌ای (rad/s) برای گازبو
        // توجه: نام تاپیک rpm است اما گازبو معمولا rad/s می‌گیرد
        double left_rad_s = left_vel / wheel_radius_;
        double right_rad_s = right_vel / wheel_radius_;

        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;

        left_msg.data = left_rad_s;
        right_msg.data = right_rad_s;

        left_motor_pub_->publish(left_msg);
        right_motor_pub_->publish(right_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    double wheel_separation_;
    double wheel_radius_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
