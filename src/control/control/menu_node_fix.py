#!/usr/bin/env python3

import sys
import subprocess
import signal

import rclpy
from rclpy.node import Node


class ControlMenuNode(Node):

    def __init__(self):
        super().__init__('control_menu_node')

        self.get_logger().info("Control Menu Node Initialized.")

        self.zed_process = None
        self.running_processes = []

        self.launch_zed_and_wait()
        self.run_menu()

    def launch_zed_and_wait(self):
        print("\n[System] Initializing ZED 2i Camera Driver...")

        try:
            self.zed_process = subprocess.Popen(
                ['ros2', 'launch', 'zed_wrapper', 'zed.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            print("[System] Waiting for ZED initialization...")

            while True:

                if self.zed_process.poll() is not None:
                    raise RuntimeError(
                        "ZED launch process terminated unexpectedly."
                    )

                line = self.zed_process.stdout.readline()

                if not line:
                    continue

                print(line.strip())

                if "=== TOPIC selection parameters ===" in line:
                    break

            print("\n" + "=" * 45)
            print("       ZED 2i Launched Successfully!")
            print("=" * 45)

        except Exception as e:
            self.get_logger().error(
                f"Failed to launch ZED camera: {e}"
            )
            sys.exit(1)

    def run_menu(self):

        while True:

            print("\n" + "=" * 35)
            print("         ROS2 CONTROL MENU")
            print("=" * 35)
            print("1. ArUco Following")
            print("2. ATT")
            print("3. Exit")
            print("=" * 35)

            choice = input("Select option (1-3): ").strip()

            if choice == '1':
                self.handle_aruco_menu()

            elif choice == '2':
                print("\n[Launching] ATT Node...")
                self.launch_node('att', 'att_node')

            elif choice == '3':
                self.shutdown_all()
                sys.exit(0)

            else:
                print("[Invalid Choice]")

    def handle_aruco_menu(self):

        while True:

            print("\n--- ArUco Following Options ---")
            print("1. Right Skew")
            print("2. Left Skew")
            print("3. No Skew")
            print("4. Back")
            print("--------------------------------")

            choice = input("Select option (1-4): ").strip()

            if choice == '1':

                print("\n[Launching] ArUco Right Skew")

                self.launch_node(
                    'aruco',
                    'search_test_node'
                )

                self.launch_node(
                    'arucop',
                    'aruco_follower',
                    [
                        '--ros-args',
                        '-p',
                        'search_skew:=1'
                    ]
                )

                break

            elif choice == '2':

                print("\n[Launching] ArUco Left Skew")

                self.launch_node(
                    'aruco',
                    'search_test_node'
                )

                self.launch_node(
                    'arucop',
                    'aruco_follower',
                    [
                        '--ros-args',
                        '-p',
                        'search_skew:=-1'
                    ]
                )

                break

            elif choice == '3':

                print("\n[Launching] ArUco No Skew")

                self.launch_node(
                    'aruco',
                    'search_test_node'
                )

                self.launch_node(
                    'arucop',
                    'aruco_follower',
                    [
                        '--ros-args',
                        '-p',
                        'search_skew:=0'
                    ]
                )

                break

            elif choice == '4':
                return

            else:
                print("[Invalid Choice]")

    def launch_node(
        self,
        package_name,
        executable_name,
        extra_args=None
    ):

        command = [
            'ros2',
            'run',
            package_name,
            executable_name
        ]

        if extra_args:
            command.extend(extra_args)

        try:

            process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            self.running_processes.append(process)

            print(
                f"[Started] {package_name}/{executable_name}"
            )

            return process

        except Exception as e:

            self.get_logger().error(
                f"Failed to launch "
                f"{package_name}/{executable_name}: {e}"
            )

            return None

    def shutdown_all(self):

        print("\n[System] Shutting down all processes...")

        for proc in self.running_processes:

            try:

                if proc.poll() is None:

                    proc.send_signal(signal.SIGINT)

                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()

            except Exception:
                pass

        if self.zed_process:

            try:

                if self.zed_process.poll() is None:

                    self.zed_process.send_signal(signal.SIGINT)

                    try:
                        self.zed_process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.zed_process.kill()

            except Exception:
                pass

        print("[System] Shutdown complete.")


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = ControlMenuNode()

    except KeyboardInterrupt:
        print("\n[Ctrl+C] Detected")

    finally:

        if node:
            node.shutdown_all()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
