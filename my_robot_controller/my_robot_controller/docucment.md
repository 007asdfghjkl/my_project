This ROS2 package implements a robot controller for automating food delivery in a café. The robot is capable of collecting orders from the kitchen and delivering them to different tables. The package includes Python nodes for managing orders and controlling navigation, as well as a launch file to start the robot system.

Table of Contents

    Installation
    Setup
    Running the Package
    Launch File
    Build System
    Cleaning Build
    Troubleshooting


To start the robot controller with the specified nodes, use the following command:

ros2 launch my_robot_controller my_robot_launch.py

This will launch the following nodes:

    order_manager: Handles the order management and delivery.
    navigation_controller: Manages the navigation of the robot.

Launch File

The launch file my_robot_launch.py defines how the ROS2 nodes should be launched together. It is located in the launch/ directory of the package.


import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='order_manager',
            name='order_manager_node',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='navigation_controller',
            name='navigation_controller_node',
            output='screen'
        ),
    ])



Run the following command to build the package:

python3 Makefile.py build

This will:

    Install dependencies via setup.py
    Prepare the package for running with ROS2.


Usage

    Order Manager: The order_manager node will handle the reception of orders and dispatch them to the navigation controller.
    Navigation Controller: The navigation_controller node will control the robot's movements, navigating to the kitchen, tables, and returning to the home position.
Tasks

The robot is designed to perform the following tasks in the café:

    Order Collection:
        When an order is received, the robot will move from its home position to the kitchen.
        It will then collect the food and deliver it to the correct table.
        Once the food is delivered, the robot will return to its home position.

    Wait for Confirmation:
        If no one attends to the robot at the kitchen or table, the robot will wait for confirmation.
        If no confirmation is received after a timeout, the robot will return to the home position.

    Order Cancellation:
        If the order is canceled while the robot is on the way to the table or kitchen, the robot will return to the previous position or home.

    Multiple Orders:
        When multiple orders are received, the robot will collect all orders from the kitchen and deliver them to the respective tables in sequence.
        