#! /usr/bin/env python3
import time
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from custom_interfaces.srv import GoToLoading

class ApproachShelfClient(Node):
    def __init__(self):
        super().__init__('approach_shelf_client')
        self.approach_shelf_client = self.create_client(GoToLoading, '/approach_shelf')
    def call_approach_shelf_service(self, final_approach):
        if not self.approach_shelf_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Service /approach_shelf not available.')
            return

        request = GoToLoading.Request()
        request.attach_to_shelf = final_approach

        future = self.approach_shelf_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            if response is not None:
                if response.complete:
                    self.get_logger().info('Final approach successful.')
                else:
                    self.get_logger().warn('Failed to complete final approach.')
            else:
                self.get_logger().error('Service call to /approach_shelf failed: No response received.')
        except Exception as e:
            self.get_logger().error(f'Service call to /approach_shelf failed: {str(e)}')

def main():
    rclpy.init()
    navigator = BasicNavigator()
    approach_shelf_client = ApproachShelfClient()

    navigator.waitUntilNav2Active()

    target_pose1 = PoseStamped()
    target_pose1.header.frame_id = 'map'
    target_pose1.header.stamp = navigator.get_clock().now().to_msg()
    target_pose1.pose.position.x = 0.18
    target_pose1.pose.position.y = 0.18
    target_pose1.pose.orientation.z = 0.0
    target_pose1.pose.orientation.w = 1.0
    print('Moving to init_position...')
    navigator.goToPose(target_pose1)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached init_position successfully!')
    elif result == TaskResult.CANCELED:
        print('Task was canceled. Returning to the staging point...')
        navigator.goToPose(initial_pose)
    elif result == TaskResult.FAILED:
        print('Task failed!')
        exit(-1)

    # Add a second destination point
    target_pose2 = PoseStamped()
    target_pose2.header.frame_id = 'map'
    target_pose2.header.stamp = navigator.get_clock().now().to_msg()
    target_pose2.pose.position.x = 5.7
    target_pose2.pose.position.y = 0.18
    target_pose2.pose.orientation.z = -0.70710678118
    target_pose2.pose.orientation.w = 0.70710678118
    print('Moving to the loading_position...')
    navigator.goToPose(target_pose2)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached the loading_position successfully!')
        approach_shelf_client.call_approach_shelf_service(final_approach=True)
        reached_second_destination = True
    elif result == TaskResult.CANCELED:
        print('Task was canceled. Returning to the staging point...')
        navigator.goToPose(initial_pose)
    elif result == TaskResult.FAILED:
        print('Task failed!')
        exit(-1)

    if reached_second_destination:
        target_pose3 = PoseStamped()
        target_pose3.header.frame_id = 'map'
        target_pose3.header.stamp = navigator.get_clock().now().to_msg()
        target_pose3.pose.position.x = 0.25
        target_pose3.pose.position.y = -3.0
        target_pose3.pose.orientation.z = 0.70710678118
        target_pose3.pose.orientation.w = 0.70710678118
        print('Moving to shipping_position...')
        navigator.goToPose(target_pose3)

        while not navigator.isTaskComplete():
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Reached shipping_position successfully!')
        elif result == TaskResult.CANCELED:
            print('Task was canceled. Returning to the staging point...')
            navigator.goToPose(initial_pose)
        elif result == TaskResult.FAILED:
            print('Task failed!')
            exit(-1)
    exit(0)

if __name__ == '__main__':
    main()
