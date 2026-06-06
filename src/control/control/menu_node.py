#!/usr/bin/env python3
import sys
import os
import subprocess
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class ControlMenuNode(Node):
    def __init__(self):
        super().__init__('control_menu_node')
        self.get_logger().info("Control Menu Node Initialized.")
        
        # 1. Start the ZED launch file in the background and wait for it
        self.launch_zed_and_wait()
        
        # 2. Drop cleanly into the blocking interactive menu loop
        self.run_menu()

    def launch_zed_and_wait(self):
        print("\n[System] Initializing ZED 2i Camera Driver...")
        try:
            # Safely start the ZED driver as a background subprocess
            self.zed_process = subprocess.Popen(
                ['ros2', 'launch', 'zed_wrapper', 'zed.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True
            )
            
            print("[System] Waiting for ZED 2i lifecycle initialization confirmation...")
            
            # Read the camera log line-by-line until the success target phrase matches
            while True:
                if self.zed_process.poll() is not None:
                    raise Exception("ZED launch process terminated unexpectedly.")
                    
                line = self.zed_process.stdout.readline()
                if not line:
                    continue
                
                # Check for the distinct topic setup completion line from your logs
                if "=== TOPIC selection parameters ===" in line:
                    break
            
            # Print your requested success confirmation message
            print("\n" + "="*40)
            print("       Zed2i Launched Successfully!       ")
            print("=========================================")
            
        except Exception as e:
            self.get_logger().error(f"Critical error launching ZED camera: {e}")
            sys.exit(1)

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
                print("Shutting down ZED Camera and exiting Control Menu...")
                self.zed_process.terminate()
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
                self.launch_node('aruco', 'camera_publisher')
                self.launch_node('arucop', 'aruco_node', extra_args=['--ros-args', '-p', 'search_skew:=1'])
                break
            elif choice == '2':
                print("\n[Launching] ArUco with Left Skew Search Pattern...")
                self.launch_node('arucop', 'aruco_follower', extra_args=['--ros-args', '-p', 'search_skew:=-1'])
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
    node = None
    try:
        node = ControlMenuNode()
    except (SystemExit, KeyboardInterrupt):
        if node and hasattr(node, 'zed_process'):
            node.zed_process.terminate()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
