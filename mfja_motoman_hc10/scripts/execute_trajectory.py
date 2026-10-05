#!/usr/bin/env python3

import sys
import csv
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory

# In ROS 2, moveit_commander is deprecated.
# If you use moveit_py, it is configured via the Node / MoveItPy API:
# from moveit.planning import MoveItPy


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('execute_trajectory')

        self.current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6"
        ]

        # ROS 2 Subscription
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.pub_trajectory = self.create_publisher(
            JointTrajectory,
            'joint_path_command',
            10
        )

        self.pub_display = self.create_publisher(
            DisplayTrajectory,
            '/move_group/display_planned_path',
            20
        )

        self.get_logger().info("TrajectoryExecutor Node Initialized")

    def joint_state_callback(self, msg: JointState):
        """Update current joint position based on subscribed joint names."""
        # Map values according to expected joint_names order if available
        if msg.position:
            self.current_joint_position = list(msg.position)
            self.get_logger().debug(f"Received joint positions: {self.current_joint_position}")

    def is_current_init(self, initial_position, threshold: float = 0.01) -> bool:
        """Check if robot is within threshold distance of initial position."""
        for curr, init in zip(self.current_joint_position, initial_position):
            if abs(init - curr) > threshold:
                return False
        return True

    def read_trajectory(self, filename: str, Ts: float = 0.1) -> JointTrajectory:
        """Read CSV trajectory file and output a JointTrajectory message."""
        self.get_logger().info(f"Reading trajectory from: {filename}")
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        with open(filename, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            # Skip header
            next(csv_reader, None)

            line_count = 1
            for row in csv_reader:
                point = JointTrajectoryPoint()
                point.positions = list(map(float, row))
                point.velocities = [0.0] * len(self.joint_names)
                
                # Convert time duration float to rclpy Duration -> msg
                time_offset = Duration(seconds=Ts * line_count)
                point.time_from_start = time_offset.to_msg()

                trajectory.points.append(point)
                line_count += 1

        return trajectory

    def execute_trajectory(self, filename: str):
        """Read trajectory, prepend current position, and publish."""
        trajectory = self.read_trajectory(filename)

        if not trajectory.points:
            self.get_logger().error("Trajectory file is empty or invalid!")
            return

        initial_position = trajectory.points[0].positions
        self.get_logger().info(f"Target initial position: {initial_position}")

        init_ok = self.is_current_init(initial_position)
        if not init_ok:
            self.get_logger().warn("Current state does not match trajectory start position.")
            # Note: For automated planning to start position in ROS 2, 
            # use a MoveGroupAction client or MoveItPy interface here.

        # Prepend current position at t = 0
        point_current = JointTrajectoryPoint()
        point_current.positions = list(self.current_joint_position)
        point_current.velocities = [0.0] * len(self.joint_names)
        point_current.time_from_start = Duration(seconds=0).to_msg()
        trajectory.points.insert(0, point_current)

        # Header setup (Note: seq was removed in ROS 2 Header)
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = "base_link"

        # Publish
        self.pub_trajectory.publish(trajectory)
        self.get_logger().info("Successfully published JointTrajectory command.")


def main(args=None):
    parser = argparse.ArgumentParser(description="ROS 2 Trajectory Executor")
    parser.add_argument("filename", help="Path to CSV trajectory file")
    known_args, remaining_args = parser.parse_known_args()

    rclpy.init(args=remaining_args)

    executor_node = TrajectoryExecutor()

    # Spin briefly to allow state callbacks to receive current joint positions
    start_time = time.time()
    while time.time() - start_time < 1.0:
        rclpy.spin_once(executor_node, timeout_sec=0.1)

    executor_node.execute_trajectory(filename=known_args.filename)

    # Clean shutdown
    executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()