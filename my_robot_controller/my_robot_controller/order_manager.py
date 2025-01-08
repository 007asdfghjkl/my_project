#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.order_sub = self.create_subscription(String, 'order_topic', self.handle_order, 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Define predefined positions for kitchen and tables
        self.positions = {
            "kitchen": {"x": 1.0, "y": 0.0},
            "table1": {"x": 2.0, "y": 1.0},
            "table2": {"x": 2.0, "y": -1.0},
            "table3": {"x": 3.0, "y": 0.0},
        }

    def handle_order(self, msg):
        order = msg.data  # Example format: "table1"
        if order not in self.positions:
            self.get_logger().warn(f"Invalid order: {order}")
            return

        # Navigate to kitchen first
        self.send_goal('kitchen')

        # Then navigate to the table
        self.send_goal(order)

        # Return to home position
        self.send_goal('home')

    def send_goal(self, location):
        if location not in self.positions:
            self.get_logger().warn(f"Invalid location: {location}")
            return

        # Create a goal pose message
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Set goal position
        goal.pose.position.x = self.positions[location]['x']
        goal.pose.position.y = self.positions[location]['y']
        goal.pose.orientation.w = 1.0  # Facing forward

        # Publish goal
        self.get_logger().info(f"Navigating to {location}")
        self.goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = OrderManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()