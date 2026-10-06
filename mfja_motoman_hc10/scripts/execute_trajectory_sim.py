#!/usr/bin/env python3
import csv
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class CSVTrajectoryFollower(Node):
    def __init__(self, action_topic="/dn2p1_controller/follow_joint_trajectory"):
        super().__init__("csv_trajectory_follower")

        # Define joint names explicitly according to HC10 robot model
        self.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        self._action_client = ActionClient(self, FollowJointTrajectory, action_topic)
        self.get_logger().info(f"Connecting to action server on '{action_topic}'...")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"Action server '{action_topic}' not available! "
                "Check controller names with: ros2 control list_controllers"
            )
            sys.exit(1)

        self.get_logger().info("Action server connected successfully.")

    def parse_csv(self, csv_filepath, time_step=0.1):
        """
        Parses 6 joint values per line and calculates time_from_start.
        :param csv_filepath: Path to CSV file
        :param time_step: Time in seconds between consecutive trajectory points
        """
        points = []
        current_time = 0.0

        with open(csv_filepath, mode="r") as f:
            reader = csv.reader(f)

            for row_idx, row in enumerate(reader):
                if not row or row[0].startswith("#"):
                    continue

                # Clean whitespace and parse float joint values
                positions = [float(val.strip()) for val in row if val.strip()]

                if len(positions) != 6:
                    self.get_logger().error(
                        f"Line {row_idx + 1} has {len(positions)} values instead of 6!"
                    )
                    sys.exit(1)

                current_time += time_step

                point = JointTrajectoryPoint()
                point.positions = positions

                # Convert float time to ROS 2 Duration message
                sec = int(current_time)
                nanosec = int((current_time - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)

                points.append(point)

        return points

    def execute_csv_trajectory(self, csv_filepath, time_step=0.1):
        points = self.parse_csv(csv_filepath, time_step=time_step)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = points

        self.get_logger().info(
            f"Sending trajectory with {len(points)} points ({time_step}s interval per step)..."
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected by controller.")
            return False

        self.get_logger().info("Goal accepted! Executing trajectory...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("Trajectory execution completed successfully!")
            return True
        else:
            self.get_logger().error(f"Execution failed with status code: {result.status}")
            return False


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 send_simple_csv_trajectory.py <path_to_csv_file> [time_step_in_seconds]")
        sys.exit(1)

    csv_path = sys.argv[1]
    time_step = 0.1 #float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    rclpy.init()
    node = CSVTrajectoryFollower(action_topic="/dn2p1_controller/follow_joint_trajectory")

    try:
        node.execute_csv_trajectory(csv_path, time_step=time_step)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()