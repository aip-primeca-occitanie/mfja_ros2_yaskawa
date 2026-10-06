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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# MotoROS2 specific interfaces
from industrial_msgs.msg import RobotStatus
from motoros2_interfaces.srv import StartTrajMode, ResetError
from std_srvs.srv import Trigger

DEFAULT_TRAJECTORY_PATH = "trajectory.csv"


class MotoROS2TrajectoryExecutor(Node):
    def __init__(self, arm_selection: str):
        prefix = f"yaskawa_{arm_selection.upper()}"
        node_namespace = f"/{prefix}"

        super().__init__('motoros2_trajectory_executor', namespace=node_namespace)

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

        # ------------------------------------------------------------------
        # Step 1: Subscriptions, Service Clients & Action Client
        # ------------------------------------------------------------------
        # Configure QoS profile to match Best Effort sensor publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_joint_states = self.create_subscription(
            JointState, 
            'joint_states', 
            self._joint_state_callback, 
            qos_profile
        )
        
        
        self.sub_robot_status = self.create_subscription(
            RobotStatus,
            'robot_status',
            self._robot_status_callback, 
            qos_profile
        )

        # MotoROS2 Service Clients
        self.client_start_traj_mode = self.create_client(StartTrajMode, 'start_traj_mode')
        self.client_reset_error = self.create_client(ResetError, 'reset_error')
        self.client_stop_traj_mode = self.create_client(Trigger, 'stop_traj_mode')

        # FollowJointTrajectory Action Client
        self.action_client = ActionClient(
            self, FollowJointTrajectory, 'follow_joint_trajectory'
        )

        self.get_logger().info(f"Initialized MotoROS2 Executor for [{self.arm_selection}] in NS: '{self.get_namespace()}'")

    def _joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def _robot_status_callback(self, msg: RobotStatus):
        self.latest_robot_status = msg

    def get_ordered_joint_positions(self) -> Optional[List[float]]:
        """Extract current joint positions ordered according to self.joint_names."""
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

    # ------------------------------------------------------------------
    # Step 2: Retrieve Robot Status & Joint State
    # ------------------------------------------------------------------
    def wait_until_idle_and_get_state(self, timeout_sec: float = 10.0) -> List[float]:
        self.get_logger().info("Checking robot_status and fetching current joint state...")
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start_time > timeout_sec:
                raise RuntimeError("Timeout waiting for robot_status or joint_states messages.")

            if self.latest_robot_status is None or self.latest_joint_state is None:
                continue

            # Verify robot is idle (in_motion == 0 / TriState.FALSE)
            if self.latest_robot_status.in_motion.val == 0:
                positions = self.get_ordered_joint_positions()
                if positions:
                    self.get_logger().info(f"Robot is IDLE. Current joint state cached: {positions}")
                    return positions


            self.get_logger().info("Waiting for robot to come to a complete stop...", throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # Step 3: Construct Trajectory (with current state as point 0)
    # ------------------------------------------------------------------
    
    def read_and_construct_trajectory(
        self, 
        filename: str, 
        current_positions: List[float], 
        Ts: float = 0.1, 
        max_joint_speed: float = 0.2
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


    # ------------------------------------------------------------------
    # Steps 4 & 5: Check Active Errors & Reset if needed
    # ------------------------------------------------------------------
    def verify_and_clear_errors(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_robot_status is None:
                continue

            # Check if in_error state is active
            if self.latest_robot_status.in_error.val != 0:
                err_code = self.latest_robot_status.error_codes
                self.get_logger().warn(f"Robot in ERROR state (code: {err_code}). Requesting reset_error...")

                if not self.client_reset_error.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error("reset_error service unavailable.")
                    time.sleep(1.0)
                    continue

                req = ResetError.Request()
                future = self.client_reset_error.call_async(req)
                rclpy.spin_until_future_complete(self, future)

                res = future.result()
                if res and res.result_code.val == 1: # Success
                    self.get_logger().info("Error reset successful.")
                else:
                    self.get_logger().error("Failed to reset error. Retrying...")
                    time.sleep(1.0)
            else:
                self.get_logger().info("No active errors detected.")
                break

    # ------------------------------------------------------------------
    # Step 6: Call start_traj_mode service
    # ------------------------------------------------------------------
    def enable_trajectory_mode(self):
        self.get_logger().info("Calling start_traj_mode service...")
        if not self.client_start_traj_mode.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("start_traj_mode service not available.")

        req = StartTrajMode.Request()
        future = self.client_start_traj_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        if res and res.result_code.value == 1:  # Success
            self.get_logger().info("MotoROS2 trajectory mode ENABLED successfully.")
        else:
            code = res.result_code.value if res else 'None'
            raise RuntimeError(f"Failed to enable trajectory mode. Result code: {code}")

    # ------------------------------------------------------------------
    # Steps 7, 8, 9, 10: Submit Action Goal & Monitor Execution
    # ------------------------------------------------------------------
    def execute_action_goal(self, trajectory: JointTrajectory):
        self.get_logger().info("Waiting for follow_joint_trajectory action server...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("follow_joint_trajectory action server unavailable.")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info("Submitting goal to MotoROS2 action server...")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal rejected by MotoROS2 Action Server.")

        self.get_logger().info("Goal accepted by MotoROS2. Monitoring trajectory execution...")

        # Wait for completion
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result()
        error_code = result.result.error_code

        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory execution SUCCESSFUL!")
        else:
            self.get_logger().error(f"Trajectory execution failed with error_code: {error_code}")

    # ------------------------------------------------------------------
    # Step 11: Call stop_traj_mode service
    # ------------------------------------------------------------------
    def disable_trajectory_mode(self):
        self.get_logger().info("Calling stop_traj_mode service...")
        if self.client_stop_traj_mode.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            future = self.client_stop_traj_mode.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info("MotoROS2 trajectory mode STOPPED.")


def main(args=None):
    parser = argparse.ArgumentParser(description="MotoROS2 Trajectory Executor Protocol")
    parser.add_argument("filenames", nargs="*", default=[DEFAULT_TRAJECTORY_PATH], help="Paths to CSV trajectory files")
    parser.add_argument("--robot", choices=["left", "right"], default="left", help="Target arm ('left' or 'right')")

    parsed_args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    executor = MotoROS2TrajectoryExecutor(arm_selection=parsed_args.robot)

    try:
        # Loop over trajectories (Step 11 -> Step 2 recursion)
        for traj_file in parsed_args.filenames:
            # Step 2: Verify idle & get joint state
            current_state = executor.wait_until_idle_and_get_state()

            # Step 3: Construct trajectory using current state as point 0
            trajectory = executor.read_and_construct_trajectory(traj_file, current_state)

            # Step 4 & 5: Clear active errors
            executor.verify_and_clear_errors()

            # Step 6: Put MotoROS2 into trajectory mode
            executor.enable_trajectory_mode()

            # Steps 7-10: Execute via Action Client
            executor.execute_action_goal(trajectory)

    except Exception as err:
        executor.get_logger().error(f"Execution stopped due to error: {err}")

    finally:
        # Step 11: Exit trajectory execution mode on completion/shutdown
        executor.disable_trajectory_mode()
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()