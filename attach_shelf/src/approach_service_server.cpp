#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "custom_interfaces/srv/go_to_loading.hpp"

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {
    // Callback group
    cbg_laser =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cbg_service =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    //
    rclcpp::SubscriptionOptions options;
    options.callback_group = cbg_laser;

    // Initialize the publishers subscribers
    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&ApproachServiceServer::laser_scan_callback, this,
                      std::placeholders::_1),
            options);

    // Initialize the service server
    approach_shelf_service_ =
        this->create_service<custom_interfaces::srv::GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachServiceServer::handle_approach_shelf_request,
                      this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_default, // Pass the default QoS profile
            cbg_service);            // Pass the callback group directly

    // Initialize the publishers
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    elevator_up_publisher_ =
        this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);
    elevator_down_publisher_ =
        this->create_publisher<std_msgs::msg::Empty>("/elevator_down", 10);

    // Initialize the TF2 broadcaster
    stf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void handle_approach_shelf_request(
      const std::shared_ptr<custom_interfaces::srv::GoToLoading::Request>
          request,
      std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response) {
    //-----------------Wait for laser scan
    // data--------------------------------------------
    bool received_data = false;
    while (!received_data) {
      if (scan_data_received_) {
        received_data = true;
        scan_data_received_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for laser scan data...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    //---------------------Check for
    // legs--------------------------------------------------------

    if (found_shelf_legs_) {

      if (request->attach_to_shelf) {
        RCLCPP_INFO(this->get_logger(), "attach_to_shelf:True");
        geometry_msgs::msg::Twist cmd_vel_msg;
        double linear_error = 100;
        double angular_error = 100;

        // Implement your logic for moving the robot using cmd_vel_publisher_
        while (linear_error > 0.05) {

          // Publish the cart_frame transform
          tf2::Quaternion quaternion;
          quaternion.setRPY(0, 0, cart_frame_yaw);
          geometry_msgs::msg::TransformStamped ctf;
          ctf.header.stamp = now();
          ctf.header.frame_id = "robot_front_laser_base_link";
          ctf.child_frame_id = "cart_frame";
          ctf.transform.translation.x = cart_frame_x;
          ctf.transform.translation.y = -cart_frame_y;
          ctf.transform.translation.z = 0.0;
          ctf.transform.rotation = tf2::toMsg(quaternion);
          tf_broadcaster_->sendTransform(ctf);

          // Calculate the position and orientation errors

          linear_error = sqrt(pow(cart_frame_x, 2) + pow(cart_frame_y, 2));
          angular_error = atan2(cart_frame_y, cart_frame_x);

          // Calculate the control output using the PID controller equation
          double linear_output = Kp_linear * linear_error;
          double angular_output = Kp_angular * angular_error;

          angular_output = std::max(-0.2, std::min(0.2, angular_output));
          linear_output = std::max(-0.3, std::min(0.3, linear_output));

          // Publish the control output

          cmd_vel_msg.linear.x = linear_output;
          cmd_vel_msg.angular.z = angular_output;
          // progam crashes here
          cmd_vel_publisher_->publish(cmd_vel_msg);
        }
        double last_yaw = cart_frame_yaw;
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = (last_yaw < 0 ? 0.3 : -0.3);
        cmd_vel_publisher_->publish(cmd_vel_msg);

        rclcpp::sleep_for(
            std::chrono::milliseconds(int(1000 * abs(last_yaw) / 0.3)));

        RCLCPP_INFO(this->get_logger(), "Done");

        cmd_vel_msg.linear.x = 0.1;
        cmd_vel_msg.angular.z = 0;
        cmd_vel_publisher_->publish(cmd_vel_msg);

        rclcpp::sleep_for(std::chrono::milliseconds(9000));

        cmd_vel_msg.linear.x = 0;
        cmd_vel_publisher_->publish(cmd_vel_msg);
        std_msgs::msg::Empty empty_msg;
        elevator_up_publisher_->publish(empty_msg);

        response->complete = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "false");
        // Publish the cart_frame transform
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, cart_frame_yaw);
        geometry_msgs::msg::TransformStamped sctf;
        sctf.header.stamp = now();
        sctf.header.frame_id = "robot_front_laser_base_link";
        sctf.child_frame_id = "cart_frame";
        sctf.transform.translation.x = cart_frame_x;
        sctf.transform.translation.y = -cart_frame_y;
        sctf.transform.translation.z = 0.0;
        sctf.transform.rotation = tf2::toMsg(quaternion);
        tf_broadcaster_->sendTransform(sctf);
        response->complete = true;
      }

    } else {
      response->complete = false;
    }
  }

  //---------------------------------------------------------------------------------------
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_data_received_ = true;
    // Initialize variables

    float intensity_threshold = 7000.0;
    size_t leg1_index = 0;
    size_t leg2_index = 0;
    float leg_distance_threshold = 50;
    // Iterate over intensities to find the reflective plates
    for (size_t i = 0; i < msg->intensities.size(); ++i) {

      if (msg->intensities[i] > intensity_threshold) {
        if (leg1_index == 0) {
          leg1_index = i;
        } else if (leg2_index == 0 &&
                   (i - leg1_index) > leg_distance_threshold) {
          leg2_index = i;
          break;
        }
      }
    }

    // Check if both legs are found
    if (leg1_index != 0 && leg2_index != 0) {
      found_shelf_legs_ = true;

      // Calculate the average angle and distance to the center point
      float angle_leg1 =
          -1 * (msg->angle_min + msg->angle_increment * leg1_index);
      float angle_leg2 =
          -1 * (msg->angle_min + msg->angle_increment * leg2_index);

      // Calculate the position of the cart_frame in the laser frame
      cart_frame_x = (msg->ranges[leg1_index] * std::cos(angle_leg1) +
                      msg->ranges[leg2_index] * std::cos(angle_leg2)) /
                     2.0;
      cart_frame_y = (msg->ranges[leg1_index] * std::sin(angle_leg1) +
                      msg->ranges[leg2_index] * std::sin(angle_leg2)) /
                     2.0;
      cart_frame_yaw = atan2(msg->ranges[leg2_index] * std::cos(angle_leg1) -
                                 msg->ranges[leg1_index] * std::cos(angle_leg2),
                             msg->ranges[leg2_index] * std::sin(angle_leg1) -
                                 msg->ranges[leg1_index] * std::sin(angle_leg2)

      );

    } else {
      found_shelf_legs_ = false;
    }
  }

  // PID controller gains
  double Kp_linear = 0.5;
  double Kp_angular = 5;

  float cart_frame_x;
  float cart_frame_y;
  float cart_frame_yaw;

  bool found_shelf_legs_ = false;
  bool scan_data_received_ = false;

  rclcpp::CallbackGroup::SharedPtr cbg_laser;
  rclcpp::CallbackGroup::SharedPtr cbg_service;

  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr
      approach_shelf_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_down_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> stf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto approach_service_server = std::make_shared<ApproachServiceServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(approach_service_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}