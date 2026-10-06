#!/usr/bin/env python3

import sys
import csv
import argparse
import time
from pathlib import Path
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

# Standard ROS 2 messages & actions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

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
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        ]
        #self.joint_names = [f"{self.joint_prefix}{j}" for j in self.base_joint_names]

        # Latest cached states
        self.latest_joint_state: Optional[JointState] = None
        self.latest_robot_status: Optional[RobotStatus] = None

        # Subscriptions using BEST_EFFORT QoS profile to match MotoROS2 publisher settings
        self.sub_joint_states = self.create_subscription(
            JointState, 
            'joint_states', 
            self._joint_state_callback, 
            qos_profile_sensor_data
        )
        self.sub_robot_status = self.create_subscription(
            RobotStatus, 
            'robot_status', 
            self._robot_status_callback, 
            qos_profile_sensor_data
        )

        # Service Clients
        self.client_start_traj_mode = self.create_client(StartTrajMode, 'start_traj_mode')
        self.client_reset_error = self.create_client(ResetError, 'reset_error')
        self.client_stop_traj_mode = self.create_client(Trigger, 'stop_traj_mode')

        # STEP 1: Create FollowJointTrajectory action client pointing to MotoROS2 server
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
        
        positions = []
        for name in self.joint_names:
            if name in self.latest_joint_state.name:
                idx = self.latest_joint_state.name.index(name)
                positions.append(self.latest_joint_state.position[idx])
            else:
                return None
        return positions

    def wait_until_idle_and_get_state(self, timeout_sec: float = 10.0) -> List[float]:
        """
        STEP 2: Retrieve a message from robot_status topic to verify robot is idle 
        (inspect in_motion) and store current JointState from joint_states.
        """
        self.get_logger().info("Checking robot_status and fetching current JointState...")
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

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

            self.get_logger().info("Waiting for robot to become IDLE...", throttle_duration_sec=2.0)

    def read_and_construct_trajectory(
        self, 
        filename: str, 
        current_positions: List[float], 
        Ts: float = 0.1, 
        max_joint_speed: float = 0.5
    ) -> JointTrajectory:
        """
        STEP 3: Construct JointTrajectory using current state as start state (point 0 at t=0).
        If robot isn't at the first CSV position, prepend a smooth lead-in segment.
        """
        file_path = Path(filename)
        if not file_path.is_file():
            raise FileNotFoundError(f"Trajectory file not found: {file_path.resolve()}")

        # Read CSV raw points
        csv_points = []
        with open(file_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            next(csv_reader, None)  # Skip header row
            for row in csv_reader:
                csv_points.append(list(map(float, row)))

        if not csv_points:
            raise ValueError(f"Trajectory CSV file '{file_path.name}' contains no data points.")

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = f"{self.joint_prefix}base_link"

        # Point 0 MUST be current robot joint state at t = 0.0 (MotoROS2 requirement)
        p0 = JointTrajectoryPoint()
        p0.positions = list(current_positions)
        p0.velocities = [0.0] * len(self.joint_names)
        p0.time_from_start = Duration(seconds=0.0).to_msg()
        trajectory.points.append(p0)

        # Check distance between current state and CSV start point
        first_csv_point = csv_points[0]
        max_delta = max([abs(c - t) for c, t in zip(current_positions, first_csv_point)])

        # If arm is not already at first CSV point, calculate lead-in segment duration
        if max_delta > 0.01:
            lead_in_duration = max(2.0, max_delta / max_joint_speed)
            self.get_logger().warn(
                f"Arm offset from CSV start position by {max_delta:.3f} rad. "
                f"Prepending a {lead_in_duration:.2f}s smooth lead-in segment."
            )
        else:
            lead_in_duration = 0.0

        # Append CSV points with calculated time_from_start
        for idx, pos in enumerate(csv_points):
            pt = JointTrajectoryPoint()
            pt.positions = pos
            pt.velocities = [0.0] * len(self.joint_names)

            time_offset = lead_in_duration + (idx * Ts if lead_in_duration > 0 else (idx + 1) * Ts)
            pt.time_from_start = Duration(seconds=time_offset).to_msg()
            trajectory.points.append(pt)

        self.get_logger().info(f"Constructed trajectory with {len(trajectory.points)} total points.")
        return trajectory

    def check_and_clear_errors(self):
        """
        STEPS 4 & 5: Retrieve robot_status to inspect in_error and error_code fields.
        If there are active errors, remedy by calling reset_error service.
        """
        self.get_logger().info("Checking robot_status for active errors...")
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_robot_status is None:
                continue

            if self.latest_robot_status.in_error.val != 0:
                err_code = self.latest_robot_status.error_code
                self.get_logger().warn(f"Active robot ERROR detected (code: {err_code}). Calling reset_error...")

                if not self.client_reset_error.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error("reset_error service unavailable. Retrying...")
                    time.sleep(1.0)
                    continue

                req = ResetError.Request()
                future = self.client_reset_error.call_async(req)
                rclpy.spin_until_future_complete(self, future)

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
            
    def start_trajectory_mode(self) -> bool:
        """
        STEP 6: Call start_traj_mode service to put MotoROS2 into trajectory mode.
        """
        self.get_logger().info("Calling start_traj_mode service...")
        if not self.client_start_traj_mode.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("start_traj_mode service unavailable.")
            return False

        req = StartTrajMode.Request()
        future = self.client_start_traj_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        if res is None:
            self.get_logger().error("start_traj_mode service call returned None.")
            return False

        # Extract result code value safely across ROS 2 interface versions
        code_val = getattr(res.result_code, 'val', res.result_code)

        # 0 or 1 indicates SUCCESS / READY depending on interface definition
        if code_val in (0, 1):
            self.get_logger().info(f"MotoROS2 trajectory mode ENABLED (result_code: {code_val}).")
            return True
        else:
            self.get_logger().error(f"start_traj_mode failed with result_code: {code_val}")
            return False
        
    # def start_trajectory_mode(self) -> bool:
    #     """
    #     STEP 6: Call start_traj_mode service to put MotoROS2 into trajectory mode.
    #     """
    #     self.get_logger().info("Calling start_traj_mode service...")
    #     if not self.client_start_traj_mode.wait_for_service(timeout_sec=5.0):
    #         self.get_logger().error("start_traj_mode service unavailable.")
    #         return False

    #     req = StartTrajMode.Request()
    #     future = self.client_start_traj_mode.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)

    #     res = future.result()
    #     if res and res.result_code.val == 1:
    #         self.get_logger().info("MotoROS2 trajectory mode ENABLED.")
    #         return True
    #     else:
    #         code = res.result_code.val if res else 'None'
    #         self.get_logger().error(f"start_traj_mode failed with result_code: {code}")
    #         return False

    def execute_trajectory_goal(self, trajectory: JointTrajectory):
        """
        STEPS 7 - 10:
        - Construct FollowJointTrajectory goal and store created trajectory in it
        - Submit goal using client created in Step 1
        - Wait for acceptance and monitor goal execution
        - Inspect result error_code to confirm execution success
        """
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("follow_joint_trajectory action server unavailable.")

        # Step 7: Construct Goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        # Step 8: Submit goal
        self.get_logger().info("Submitting goal to follow_joint_trajectory action server...")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        # Step 9: Wait for goal acceptance
        if not goal_handle.accepted:
            raise RuntimeError("Goal was REJECTED by MotoROS2 action server.")

        self.get_logger().info("Goal ACCEPTED by MotoROS2. Executing trajectory...")

        # Step 9 (cont): Monitor goal state while executing
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        # Step 10: Final status check via error_code field
        result = get_result_future.result()
        error_code = result.result.error_code

        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory execution SUCCESSFUL!")
        else:
            raise RuntimeError(f"Trajectory execution failed/aborted with error_code: {error_code}")

    def stop_trajectory_mode(self):
        """STEP 12: Call stop_traj_mode service to exit trajectory execution mode."""
        self.get_logger().info("Calling stop_traj_mode service...")
        if self.client_stop_traj_mode.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            future = self.client_stop_traj_mode.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info("MotoROS2 trajectory mode STOPPED.")


def main(args=None):
    parser = argparse.ArgumentParser(description="MotoROS2 Single Trajectory Executor")
    parser.add_argument("filename", type=str, help="Path to the CSV trajectory file")
    parser.add_argument("--robot", choices=["left", "right"], default="left", help="Target arm ('left' or 'right')")

    parsed_args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    # Step 1: Create action client (inside Node __init__)
    executor = MotoROS2SingleTrajectoryExecutor(arm_selection=parsed_args.robot)

    try:
        execution_success = False

        while not execution_success and rclpy.ok():
            # Step 2: Verify robot is idle & store current JointState
            current_joint_positions = executor.wait_until_idle_and_get_state()

            # Step 3: Construct trajectory message with current state as Point 0
            trajectory = executor.read_and_construct_trajectory(
                parsed_args.filename, current_joint_positions
            )

            # Steps 4 & 5: Check in_error, remedy, call reset_error
            executor.check_and_clear_errors()

            # Step 6: Call start_traj_mode service
            if not executor.start_trajectory_mode():
                executor.get_logger().warn("Failed to enable trajectory mode. Restarting at step 4...")
                time.sleep(1.0)
                continue

            # Steps 7-10: Execute goal and verify error_code
            try:
                executor.execute_trajectory_goal(trajectory)
                execution_success = True
            except RuntimeError as err:
                executor.get_logger().error(f"Execution error: {err}. Retrying workflow...")
                time.sleep(1.0)

    except Exception as err:
        executor.get_logger().error(f"Execution interrupted: {err}")

    finally:
        # Step 12: Call stop_traj_mode service
        executor.stop_trajectory_mode()
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()