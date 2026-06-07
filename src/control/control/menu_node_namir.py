#!/usr/bin/env python3
import sys
import subprocess
import rclpy
from rclpy.node import Node

class ControlMenuNode(Node):
    def __init__(self):
        super().__init__('control_menu_node')
        self.get_logger().info("Control Menu Node Initialized.")
        self.active_processes = []

        self.launch_zed_and_wait()
        self.run_menu()

    def launch_zed_and_wait(self):
        print("\n[System] Initializing ZED 2i Camera Driver...")
        try:
            self.zed_process = subprocess.Popen(
                ['ros2', 'launch', 'zed_wrapper', 'zed.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True
            )
            print("[System] Waiting for ZED 2i lifecycle initialization...")
            while True:
                if self.zed_process.poll() is not None:
                    raise Exception("ZED launch process terminated unexpectedly.")
                line = self.zed_process.stdout.readline()
                if not line:
                    continue
                if "=== TOPIC selection parameters ===" in line:
                    break
            print("\n" + "="*40)
            print("        Zed2i Launched Successfully!       ")
            print("="*40)
        except Exception as e:
            self.get_logger().error(f"Critical error launching ZED camera: {e}")
            sys.exit(1)

    def run_menu(self):
        while True:
            print("\n" + "="*30)
            print("      ROS 2 CONTROL MENU      ")
            print("="*30)
            print("1. ArUco Search + Follow")
            print("2. ATT (Attitude/Target Tracking)")
            print("3. Exit")
            print("="*30)

            choice = input("Select an option (1-3): ").strip()

            if choice == '1':
                self.handle_aruco_menu()
            elif choice == '2':
                self.launch_node_foreground('att', 'att_node')
            elif choice == '3':
                print("Shutting down...")
                self.cleanup()
                sys.exit(0)
            else:
                print("[Invalid] Please enter 1-3.")

    def handle_aruco_menu(self):
        try:
            target_id = int(input("Enter Target ArUco ID to track: ").strip())
        except ValueError:
            print("[Error] Invalid ID. Aborting.")
            return

        while True:
            print("\n--- ArUco Search + Follow Options ---")
            print("1. Right Skew | 2. Left Skew | 3. No Skew | 4. Back")
            choice = input("Select an option (1-4): ").strip()
            
            if choice in ('1', '2', '3'):
                skew = {'1': 1, '2': -1, '3': 0}[choice]
                self.handle_search_params(skew, target_id)
                break
            elif choice == '4':
                break
            else:
                print("[Invalid] Please enter 1-4.")

    def handle_search_params(self, skew, target_id):
        print(f"\n--- Search Parameters for ID {target_id} ---")
        
        try:
            fwd_time = float(input("Enter forward search time (seconds): ").strip())
            spot_turn_input = input("Enable spot turn back? (true/false): ").strip().lower()
            # Properly maps 'y', 'yes', or 'true' cleanly to ROS 2 parameter requirements
            spot_turn = 'true' if spot_turn_input in ['true', 'y', 'yes'] else 'false'
        except ValueError:
            print("[Error] Invalid parameter inputs. Using defaults (fwd_time=2.0, spot_turn=false).")
            fwd_time = 2.0
            spot_turn = 'false'

        # 1. NEW: Launch Obstacle Avoidance Node in the background
        print("[Launching] obstacle_avoidance_node (aruco)")
        self.launch_node_background('aruco', 'obstacle_avoidance_node')

        # 2. Launch background follower node
        print(f"[Launching] aruco_follower (arucop) for ID {target_id}")
        self.launch_node_background('arucop', 'aruco_follower', 
            extra_args=['--ros-args', '-p', f'target_id:={target_id}'])

        # 3. Launch foreground search script
        print(f"[Launching] search_test | skew={skew} target={target_id}")
        self.launch_node_foreground('aruco', 'search_test',
            extra_args=[
                '--ros-args',
                '-p', f'target_id:={target_id}',
                '-p', f'search_skew:={skew}',
                '-p', f'search_forward_time:={fwd_time}',
                '-p', f'spot_turn_back:={spot_turn}'
            ])

        self.cleanup_active()

    def launch_node_background(self, package, executable, extra_args=None):
        command = ['ros2', 'run', package, executable]
        if extra_args:
            command.extend(extra_args)
        try:
            proc = subprocess.Popen(command)
            self.active_processes.append(proc)
            print(f"[BG] Started {executable} (pid={proc.pid})")
        except Exception as e:
            self.get_logger().error(f"Failed to launch {executable}: {e}")

    def launch_node_foreground(self, package, executable, extra_args=None):
        command = ['ros2', 'run', package, executable]
        if extra_args:
            command.extend(extra_args)
        try:
            subprocess.run(command, check=True)
        except KeyboardInterrupt:
            print(f"\n[Stopped] {executable} terminated by user.")
        except Exception as e:
            self.get_logger().error(f"Failed to launch {executable}: {e}")

    def cleanup_active(self):
        for proc in self.active_processes:
            try:
                proc.terminate()
                print(f"[Cleanup] Terminated pid={proc.pid}")
            except Exception:
                pass
        self.active_processes.clear()

    def cleanup(self):
        self.cleanup_active()
        if hasattr(self, 'zed_process'):
            self.zed_process.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ControlMenuNode()
    except (SystemExit, KeyboardInterrupt):
        if node:
            node.cleanup()
    finally:
        # Check context state to eliminate double-shutdown errors on lifecycle drops
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
