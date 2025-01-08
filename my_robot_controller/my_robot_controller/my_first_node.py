#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import yaml
from pathlib import Path


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')  # Node name corrected

        # Load locations and timeouts
        config_path = Path(__file__).parent / 'locations.yaml'
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        self.locations = self.config['locations']
        self.timeouts = self.config['timeouts']

        # ROS Publishers and Subscribers
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.order_sub = self.create_subscription(String, 'order_topic', self.handle_order, 10)
        self.status_sub = self.create_subscription(String, 'status_topic', self.status_callback, 10)

        # State tracking
        self.current_task = None
        self.is_busy = False

        # Add a timer for periodic logging
        self.timer = self.create_timer(1.0, self.log_hello)
        self.hello_count = 0

    def log_hello(self):
        self.get_logger().info(f"Hello {self.hello_count}")
        self.hello_count += 1

    def handle_order(self, msg):
        order = msg.data.strip()
        if order not in self.locations:
            self.get_logger().error(f"Invalid order received: {order}")
            return

        if self.is_busy:
            self.get_logger().warn("Robot is currently busy. Please wait.")
            return

        self.is_busy = True
        self.current_task = {"destination": order, "status": "pending"}

        self.navigate_to("kitchen")  # Step 1: Go to the kitchen

    def navigate_to(self, location):
        if location not in self.locations:
            self.get_logger().error(f"Invalid location: {location}")
            self.is_busy = False
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.locations[location]["x"]
        goal.pose.position.y = self.locations[location]["y"]
        goal.pose.orientation.w = 1.0  # Facing forward

        self.get_logger().info(f"Navigating to {location}...")
        self.goal_pub.publish(goal)

    def status_callback(self, msg):
        status = msg.data.strip()
        if status == "Goal Reached":
            if self.current_task["status"] == "pending":
                self.current_task["status"] = "delivering"
                self.navigate_to(self.current_task["destination"])
            elif self.current_task["status"] == "delivering":
                self.get_logger().info(f"Order delivered to {self.current_task['destination']}")
                self.current_task = None
                self.is_busy = False
                self.navigate_to("home")
        elif status == "Goal Failed":
            self.get_logger().error("Navigation failed. Returning to home.")
            self.is_busy = False
            self.navigate_to("home")


def main(args=None):
    rclpy.init(args=args)
    node = OrderManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

