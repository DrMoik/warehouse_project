#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode()
      : Node("pre_approach_node"), obstacle_(0.3), degrees_(-90),
        turning_(false), start_time_(0) {

    // Initialize parameters with descriptions
    auto obstacle_desc = rcl_interfaces::msg::ParameterDescriptor{};
    obstacle_desc.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";
    this->declare_parameter<double>("obstacle", 0.3, obstacle_desc);

    auto degrees_desc = rcl_interfaces::msg::ParameterDescriptor{};
    degrees_desc.description =
        "Number of degrees for the rotation of the robot after stopping.";
    this->declare_parameter<int>("degrees", -90, degrees_desc);

    // Get parameters
    this->get_parameter("obstacle", obstacle_);
    this->get_parameter("degrees", degrees_);

    // Initialize publishers and subscribers
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PreApproachNode::laser_scan_callback, this,
                      std::placeholders::_1));
  }

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Check if obstacle is detected within specified distance
    if (pre_approach_running) {
      for (const auto &range : msg->ranges) {
        if (range <= obstacle_ && !turning_) {
          // Stop the robot
          geometry_msgs::msg::Twist stop_msg;
          stop_msg.linear.x = 0.0;
          stop_msg.angular.z = 0.0;
          cmd_vel_publisher_->publish(stop_msg);

          // Start turning
          turning_ = true;
          start_time_ = this->now().seconds();

          // Rotate the robot
          geometry_msgs::msg::Twist rotate_msg;
          rotate_msg.linear.x = 0.0;
          rotate_msg.angular.z =
              (degrees_ > 0 ? 0.3 : -0.3); // Set the rotation direction
          cmd_vel_publisher_->publish(rotate_msg);
          // return;
        }
      }

      if (turning_) {
        // Check if the robot has rotated the desired angle
        double elapsed_time = this->now().seconds() - start_time_;
        double rotated_angle = elapsed_time * 0.3 * 180 / 3.14159265;

        if (rotated_angle >= std::abs(degrees_)) {
          // Stop the robot
          geometry_msgs::msg::Twist stop_msg;
          stop_msg.linear.x = 0.0;
          stop_msg.angular.z = 0.0;
          cmd_vel_publisher_->publish(stop_msg);
          pre_approach_running = false;
          return;
        }
      } else {
        // Move the robot forward
        geometry_msgs::msg::Twist forward_msg;
        forward_msg.linear.x = 0.5;
        forward_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(forward_msg);
      }
    }
  }
  bool pre_approach_running = true;
  double obstacle_;
  int degrees_;
  bool turning_;
  double start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachNode>());
  rclcpp::shutdown();
  return 0;
}