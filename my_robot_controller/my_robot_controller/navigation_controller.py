#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.navigate_to_goal, 10)
        self.status_pub = self.create_publisher(String, 'status_topic', 10)

        self.current_goal = None

    def navigate_to_goal(self, msg):
        self.current_goal = msg
        self.get_logger().info(f"Received goal: x={msg.pose.position.x}, y={msg.pose.position.y}")

        # Simulate navigation process
        self.simulate_navigation()

    def simulate_navigation(self):
        if self.current_goal:
            self.get_logger().info(f"Navigating to x={self.current_goal.pose.position.x}, y={self.current_goal.pose.position.y}")

            # Simulate navigation time
            self.get_clock().sleep_for(2.0)

            # Publish status
            self.status_pub.publish(String(data="Goal Reached"))
            self.get_logger().info("Goal Reached")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()