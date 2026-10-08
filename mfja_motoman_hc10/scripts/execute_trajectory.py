#!/usr/bin/env python3

import sys
import csv
import argparse
import time
import signal
from pathlib import Path
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Standard ROS 2 messages & actions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

# MotoROS2 specific interfaces
from industrial_msgs.msg import RobotStatus
from motoros2_interfaces.srv import StartTrajMode, ResetError
from std_srvs.srv import Trigger


class MotoROS2SingleTrajectoryExecutor(Node):
    def __init__(self, arm_selection: str):
        prefix = f"yaskawa_{arm_selection.upper()}"
        node_namespace = f"/{prefix}"

        super().__init__('motoros2_single_trajectory_executor', namespace=node_namespace)

        self.arm_selection = arm_selection.upper()
        self.joint_prefix = f"{prefix}_"

        # Initialize empty — auto-detect joint names from first /joint_states message
        self.joint_names: List[str] = []

        # Latest cached states
        self.latest_joint_state: Optional[JointState] = None
        self.latest_robot_status: Optional[RobotStatus] = None

        # Track active goal handle for clean cancellation on Ctrl+C
        self.current_goal_handle = None

        # ------------------------------------------------------------------
        # Step 1: Subscriptions, Service Clients & Action Client
        # ------------------------------------------------------------------

        # QoS profile matching sensor data (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.sub_joint_states = self.create_subscription(
            JointState, 'joint_states', self._joint_state_callback, qos_profile
        )
        self.sub_robot_status = self.create_subscription(
            RobotStatus, 'robot_status', self._robot_status_callback, qos_profile
        )

        # Service Clients
        self.client_start_traj_mode = self.create_client(StartTrajMode, 'start_traj_mode')
        self.client_reset_error = self.create_client(ResetError, 'reset_error')
        self.client_stop_traj_mode = self.create_client(Trigger, 'stop_traj_mode')

        # Action Client
        self.action_client = ActionClient(
            self, FollowJointTrajectory, 'follow_joint_trajectory'
        )

        self.get_logger().info(
            f"Initialized MotoROS2 Single Trajectory Executor for [{self.arm_selection}] in NS: '{self.get_namespace()}'"
        )

    def _joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def _robot_status_callback(self, msg: RobotStatus):
        self.latest_robot_status = msg

    def get_ordered_joint_positions(self) -> Optional[List[float]]:
        """Extract current joint positions ordered matching self.joint_names."""
        if not self.latest_joint_state:
            return None

        # Auto-detect joint names on first message if unpopulated
        if not self.joint_names:
            self.joint_names = list(self.latest_joint_state.name)
            self.get_logger().info(f"Auto-detected robot joint names from topic: {self.joint_names}")

        positions = []
        for name in self.joint_names:
            if name in self.latest_joint_state.name:
                idx = self.latest_joint_state.name.index(name)
                positions.append(self.latest_joint_state.position[idx])
            else:
                self.get_logger().error(f"Joint '{name}' not found in incoming joint_states message!")
                return None

        return positions

    # ------------------------------------------------------------------
    # Step 2: Retrieve Robot Status & Joint State
    # ------------------------------------------------------------------
    def wait_until_idle_and_get_state(self, timeout_sec: float = 10.0) -> List[float]:
        """
        Retrieve message from robot_status topic to verify robot is idle 
        (inspect in_motion) and store current JointState from joint_states.
        """
        self.get_logger().info("Checking robot_status and fetching current JointState...")
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if time.time() - start_time > timeout_sec:
                raise RuntimeError("Timeout waiting for robot_status or joint_states messages.")

            if self.latest_robot_status is None or self.latest_joint_state is None:
                continue

            # Verify in_motion == 0 (TriState.FALSE / Idle)
            if self.latest_robot_status.in_motion.val == 0:
                positions = self.get_ordered_joint_positions()
                if positions:
                    self.get_logger().info("Robot verified IDLE. Current joint state acquired.")
                    return positions
            else:
                # robot thinks it's moving
                self.get_logger().info(f"In motion : {str(self.latest_robot_status.in_motion.val)}")
                time.sleep(0.5)

            self.get_logger().info("Waiting for robot to become IDLE...", throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # Step 3: Read CSV Points & Construct Trajectories
    # ------------------------------------------------------------------
    def read_csv_points(self, filename: str) -> List[List[float]]:
        file_path = Path(filename)
        if not file_path.is_file():
            raise FileNotFoundError(f"Trajectory file not found: {file_path.resolve()}")

        csv_points = []
        with open(file_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            next(csv_reader, None)  # Skip header row
            for row in csv_reader:
                csv_points.append(list(map(float, row)))

        num_points = len(csv_points)
        if num_points == 0:
            raise ValueError(f"Trajectory CSV file '{file_path.name}' contains no data points.")
        if num_points > 200:
            raise ValueError(
                f"Trajectory CSV has {num_points} points, which exceeds the maximum allowed limit of 200 points."
            )

        return csv_points

    def construct_lead_in_trajectory(
        self, current_positions: List[float], target_positions: List[float], max_joint_speed: float = 0.25
    ) -> Optional[JointTrajectory]:
        max_delta = max([abs(c - t) for c, t in zip(current_positions, target_positions)])

        if max_delta <= 0.01:
            return None

        lead_in_duration = max(3.0, max_delta / max_joint_speed)
        if lead_in_duration >= 20:
            self.get_logger().info(f"Current position: {str(current_positions)}")
            self.get_logger().info(f"Target position: {str(target_positions)}")
            raise ValueError("Lead-in trajectory TOO LONG")

        self.get_logger().warn(
            f"Arm offset from CSV start position by {max_delta:.3f} rad. "
            f"Constructing SEPARATE lead-in trajectory ({lead_in_duration:.2f}s duration)..."
        )

        lead_in_traj = JointTrajectory()
        lead_in_traj.joint_names = self.joint_names
        lead_in_traj.header.stamp = self.get_clock().now().to_msg()
        lead_in_traj.header.frame_id = f"{self.joint_prefix}base_link"

        # Point 0: Current position at t = 0.0
        pt0 = JointTrajectoryPoint()
        pt0.positions = list(current_positions)
        pt0.velocities = [0.0] * len(self.joint_names)
        pt0.time_from_start = Duration(seconds=0.0).to_msg()
        lead_in_traj.points.append(pt0)

        # Point 1: Target start position of CSV at t = lead_in_duration
        pt1 = JointTrajectoryPoint()
        pt1.positions = list(target_positions)
        pt1.velocities = [0.0] * len(self.joint_names)
        pt1.time_from_start = Duration(seconds=lead_in_duration).to_msg()
        lead_in_traj.points.append(pt1)

        return lead_in_traj

    def construct_main_trajectory(self, current_positions: List[float], csv_points: List[List[float]], Ts: float = 0.1) -> JointTrajectory:
        main_traj = JointTrajectory()
        main_traj.joint_names = self.joint_names
        main_traj.header.stamp = self.get_clock().now().to_msg()
        main_traj.header.frame_id = f"{self.joint_prefix}base_link"

        # Mandatory Start Point (t=0.0) from current real state
        p0 = JointTrajectoryPoint()
        p0.positions = list(current_positions)
        p0.velocities = [0.0] * len(self.joint_names)
        p0.time_from_start = Duration(seconds=0.0).to_msg()
        main_traj.points.append(p0)

        # Append CSV points offset by Ts
        for idx, pos in enumerate(csv_points):
            pt = JointTrajectoryPoint()
            pt.positions = pos
            pt.velocities = [0.0] * len(self.joint_names)
            pt.time_from_start = Duration(seconds=(idx + 1) * Ts).to_msg()
            main_traj.points.append(pt)

        self.get_logger().info(f"Main CSV trajectory loaded with {len(main_traj.points)} points.")
        return main_traj

    # ------------------------------------------------------------------
    # Steps 4 & 5: Check Active Errors & Reset if needed
    # ------------------------------------------------------------------
    def check_and_clear_errors(self):
        """
        Retrieve robot_status to inspect in_error and error_code fields.
        If there are active errors, remedy by calling reset_error service.
        """
        self.get_logger().info("Checking robot_status for active errors...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_robot_status is None:
                continue

            if self.latest_robot_status.in_error.val != 0:
                err_code = self.latest_robot_status.error_codes
                self.get_logger().warn(f"Active robot ERROR detected (code: {str(err_code)}). Calling reset_error...")

                if not self.client_reset_error.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error("reset_error service unavailable. Retrying...")
                    time.sleep(1.0)
                    continue

                req = ResetError.Request()
                future = self.client_reset_error.call_async(req)
                
                while not future.done() and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.05)

                if not rclpy.ok():
                    return

                res = future.result()
                code_val = getattr(res.result_code, 'val', res.result_code) if res else None
                if code_val in (0, 1):
                    self.get_logger().info("Error reset successful.")
                else:
                    self.get_logger().error("Failed to reset error. Retrying check...")
                    time.sleep(1.0)
            else:
                self.get_logger().info("No active errors found. Proceeding.")
                break

    # ------------------------------------------------------------------
    # Step 6: Call start_traj_mode service
    # ------------------------------------------------------------------
    def start_trajectory_mode(self) -> bool:
        """Call start_traj_mode service to put MotoROS2 into trajectory mode."""
        self.get_logger().info("Calling start_traj_mode service...")
        if not self.client_start_traj_mode.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("start_traj_mode service unavailable.")
            return False

        req = StartTrajMode.Request()
        future = self.client_start_traj_mode.call_async(req)
        
        start_time = time.time()
        while not future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > 5.0:
                self.get_logger().error("Timeout waiting for start_traj_mode response!")
                return False

        if not rclpy.ok():
            return False

        res = future.result()
        if res is None:
            self.get_logger().error("start_traj_mode service call returned None.")
            return False

        # Support both .value and .val / primitive integer result checks
        code_val = getattr(res.result_code, 'val', getattr(res.result_code, 'value', res.result_code))
        if code_val in (0, 1):
            self.get_logger().info(f"MotoROS2 trajectory mode ENABLED (result_code: {code_val}).")
            return True
        else:
            self.get_logger().error(f"Failed to enable trajectory mode. Result code: {code_val}")
            return False

    # ------------------------------------------------------------------
    # Steps 7, 8, 9, 10: Submit Action Goal & Monitor Execution
    # ------------------------------------------------------------------
    def execute_trajectory_goal(self, trajectory: JointTrajectory, description: str = "Trajectory"):
        """Submits and monitors execution of a JointTrajectory goal."""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("follow_joint_trajectory action server unavailable.")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info(f"Submitting {description} goal ({len(trajectory.points)} points)...")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        
        while not send_goal_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        if not rclpy.ok():
            raise KeyboardInterrupt("Interrupted while sending goal.")

        self.current_goal_handle = send_goal_future.result()

        if not self.current_goal_handle.accepted:
            self.current_goal_handle = None
            raise RuntimeError(f"{description} goal was REJECTED by MotoROS2 action server.")

        self.get_logger().info(f"{description} goal ACCEPTED. Executing...")

        get_result_future = self.current_goal_handle.get_result_async()
        
        while not get_result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        if not rclpy.ok():
            raise KeyboardInterrupt("Interrupted while executing goal.")

        time.sleep(0.2)

        result = get_result_future.result()
        self.current_goal_handle = None
        error_code = result.result.error_code

        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"{description} execution SUCCESSFUL!")
        else:
            raise RuntimeError(f"{description} execution failed/aborted with error_code: {error_code}")

    def stop_trajectory_mode(self):
        """Call stop_traj_mode service to exit trajectory execution mode."""
        self.get_logger().info("Calling stop_traj_mode service...")
        if self.client_stop_traj_mode.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            future = self.client_stop_traj_mode.call_async(req)
            start_time = time.time()
            while not future.done() and (time.time() - start_time < 2.0):
                rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().info("MotoROS2 trajectory mode STOPPED.")


def main(args=None):
    parser = argparse.ArgumentParser(description="MotoROS2 Single Trajectory Executor")
    parser.add_argument("filename", type=str, help="Path to the CSV trajectory file")
    parser.add_argument("--robot", choices=["left", "right"], default="left", help="Target arm ('left' or 'right')")

    parsed_args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    executor = MotoROS2SingleTrajectoryExecutor(arm_selection=parsed_args.robot)

    # ------------------------------------------------------------------
    # Signal Handler for Ctrl+C (SIGINT / SIGTERM)
    # ------------------------------------------------------------------
    def signal_handler(sig, frame):
        executor.get_logger().warn("Ctrl+C detected! Stopping trajectory mode immediately...")

        # Cancel active goal if currently executing motion
        if executor.current_goal_handle is not None:
            try:
                executor.current_goal_handle.cancel_goal_async()
            except Exception:
                pass

        # Stop MotoROS2 Trajectory Mode
        executor.stop_trajectory_mode()
        executor.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # Register OS signal intercepts
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        stop_trying = False

        while not stop_trying and rclpy.ok():
            # 1. Read CSV upfront
            csv_points = executor.read_csv_points(parsed_args.filename)

            # 2. Get current position
            current_joint_positions = executor.wait_until_idle_and_get_state()

            # 3. Check and clear active errors
            executor.check_and_clear_errors()

            # 4. Enable trajectory mode
            if not executor.start_trajectory_mode():
                executor.get_logger().warn("Failed to enable trajectory mode. Restarting workflow...")
                time.sleep(1.0)
                continue

            try:
                # 5. Execute Lead-in Trajectory separately (if required)
                lead_in_traj = executor.construct_lead_in_trajectory(current_joint_positions, csv_points[0])

                if lead_in_traj is not None:
                    executor.get_logger().info("Executing LEAD-IN trajectory segment...")
                    executor.execute_trajectory_goal(lead_in_traj, description="Lead-In")
                    
                    # Pause to allow joints to settle before sampling fresh position for main trajectory
                    time.sleep(0.3)

                    # Re-sample actual physical joint position after lead-in finishes
                    current_joint_positions = executor.wait_until_idle_and_get_state()

                # 6. Construct Main Trajectory with fresh start position
                main_traj = executor.construct_main_trajectory(current_joint_positions, csv_points)

                # 7. Execute Main Trajectory from CSV
                executor.get_logger().info("Executing MAIN CSV trajectory...")
                executor.execute_trajectory_goal(main_traj, description="Main CSV Trajectory")

                # Execution success
                stop_trying = True

            except RuntimeError as err:
                executor.get_logger().error(f"Execution error: {err}. Cleaning up state and retrying...")
                executor.stop_trajectory_mode()
                # If there was an error, do not retry the same trajectory
                stop_trying = True

    except (KeyboardInterrupt, SystemExit):
        executor.get_logger().warn("Execution interrupted by user.")
    except Exception as err:
        executor.get_logger().error(f"Execution interrupted: {err}")

    finally:
        if rclpy.ok():
            executor.stop_trajectory_mode()
            executor.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()