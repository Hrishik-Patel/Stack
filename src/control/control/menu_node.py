#!/usr/bin/env python3
import sys
import subprocess
import rclpy
from rclpy.node import Node

class ControlMenuNode(Node):
    def __init__(self):
        super().__init__('control_menu_node')
        self.get_logger().info("Control Menu Node Initialized.")
        self.run_menu()

    def run_menu(self):
        while True:
            print("\n" + "="*30)
            print("      ROS 2 CONTROL MENU      ")
            print("="*30)
            print("1. ArUco Following")
            print("2. ATT (Attitude/Target Tracking)")
            print("3. Exit")
            print("="*30)
            
            choice = input("Select an option (1-3): ").strip()

            if choice == '1':
                self.handle_aruco_menu()
            elif choice == '2':
                self.launch_node('att', 'att_node')
            elif choice == '3':
                print("Exiting Control Menu...")
                sys.exit(0)
            else:
                print("[Invalid Choice] Please enter a number between 1 and 3.")

    def handle_aruco_menu(self):
        while True:
            print("\n--- ArUco Following Options ---")
            print("1. Search Pattern: Right Skew")
            print("2. Search Pattern: Left Skew")
            print("3. Back to Main Menu")
            print("-" * 31)
            
            choice = input("Select an option (1-3): ").strip()

            if choice == '1':
                print("\n[Launching] ArUco with Right Skew Search Pattern...")
                self.launch_node('aruco', 'aruco_node', extra_args=['--ros-args', '-p', 'search_pattern:=right_skew'])
                break
            elif choice == '2':
                print("\n[Launching] ArUco with Left Skew Search Pattern...")
                self.launch_node('aruco', 'aruco_node', extra_args=['--ros-args', '-p', 'search_pattern:=left_skew'])
                break
            elif choice == '3':
                break
            else:
                print("[Invalid Choice] Please enter a number between 1 and 3.")

    def launch_node(self, package_name, executable_name, extra_args=None):
        command = ['ros2', 'run', package_name, executable_name]
        if extra_args:
            command.extend(extra_args)
            
        try:
            subprocess.run(command, check=True)
        except KeyboardInterrupt:
            print(f"\n[Stopped] {executable_name} terminated by user.")
        except Exception as e:
            self.get_logger().error(f"Failed to launch {executable_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ControlMenuNode()
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
