#include "custom_interfaces/srv/go_to_loading.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode()
      : Node("pre_approach_node"), obstacle_(0.3), degrees_(-90),
        turning_(false), start_time_(0), final_approach_(false) {

    // Initialize parameters with descriptions
    auto obstacle_desc = rcl_interfaces::msg::ParameterDescriptor{};
    obstacle_desc.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";
    this->declare_parameter<double>("obstacle", 0.3, obstacle_desc);

    auto degrees_desc = rcl_interfaces::msg::ParameterDescriptor{};
    degrees_desc.description =
        "Number of degrees for the rotation of the robot after stopping.";
    this->declare_parameter<int>("degrees", -90, degrees_desc);

    auto final_approach_desc = rcl_interfaces::msg::ParameterDescriptor{};
    final_approach_desc.description =
        "Boolean flag to control the final approach behavior.";
    this->declare_parameter<bool>("final_approach", false, final_approach_desc);

    // Get parameters
    this->get_parameter("obstacle", obstacle_);
    this->get_parameter("degrees", degrees_);
    this->get_parameter("final_approach", final_approach_);

    // Initialize publishers and subscribers
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PreApproachNode::laser_scan_callback, this,
                      std::placeholders::_1));
    approach_shelf_client_ =
        this->create_client<custom_interfaces::srv::GoToLoading>(
            "/approach_shelf");
  }

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (pre_approach_running) {
      // Check if obstacle is detected within specified distance
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
              (degrees_ > 0 ? 0.4 : -0.4); // Set the rotation direction
          cmd_vel_publisher_->publish(rotate_msg);
        }
      }

      if (turning_) {
        // Check if the robot has rotated the desired angle
        double elapsed_time = this->now().seconds() - start_time_;
        double rotated_angle = elapsed_time * 0.4 * 180 / 3.14159265;

        if (rotated_angle >= std::abs(degrees_)) {
          // Stop the robot
          geometry_msgs::msg::Twist stop_msg;
          stop_msg.linear.x = 0.0;
          stop_msg.angular.z = 0.0;
          cmd_vel_publisher_->publish(stop_msg);

          // Call approach_shelf service
          call_approach_shelf_service(final_approach_);
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

  void call_approach_shelf_service(bool final_approach) {
    if (!approach_shelf_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service /approach_shelf not available.");
      return;
    }

    auto request =
        std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
    request->attach_to_shelf = final_approach;

    auto future_result = approach_shelf_client_->async_send_request(request);
    if (future_result.wait_for(std::chrono::seconds(5)) !=
        std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service call to /approach_shelf timed out.");
      return;
    }

    auto response = future_result.get();
    if (response->complete) {
      RCLCPP_INFO(this->get_logger(), "Final approach successful.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to complete final approach.");
    }
  }

  bool pre_approach_running = true;
  double obstacle_;
  int degrees_;
  bool turning_;
  double start_time_;
  bool final_approach_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr
      approach_shelf_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachNode>());
  rclcpp::shutdown();
  return 0;
}
