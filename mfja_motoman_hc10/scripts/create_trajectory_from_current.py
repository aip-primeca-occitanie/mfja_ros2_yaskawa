#!/usr/bin/env python3

import argparse
import csv
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def moveJ(qi, qf, tf, Te):
    """
    Calculate joint trajectory cubic polynomials between qi and qf.
    
    qi: initial joint positions [6]
    qf: final joint positions [6]
    tf: total time (seconds)
    Te: step time / sampling period (seconds)
    """
    l1 = np.array([1, 0, 0, 0])
    l2 = np.array([0, 1, 0, 0])
    l3 = np.array([1, tf, tf**2, tf**3])
    l4 = np.array([0, 1, 2 * tf, 3 * (tf**2)])

    A = np.vstack((l1, l2, l3, l4))
    A_inv = np.linalg.inv(A)

    steps = int(round(tf / Te))
    q = np.zeros((steps, 6))

    for j in range(steps):
        t = j * Te
        T_vec = np.array([1, t, t**2, t**3])
        for i in range(6):
            Q = np.array([[qi[i]], [0], [qf[i]], [0]])
            C = np.matmul(A_inv, Q)
            q[j, i] = np.dot(T_vec, C).item()

    return q



class JointStateFetcher(Node):
    def __init__(self, arm_selection: str):
        prefix = f"yaskawa_{arm_selection.upper()}"
        node_namespace = f"/{prefix}"
        super().__init__('joint_state_fetcher', namespace=node_namespace)

        self.target_joint_names = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        ]
        # self.target_joint_names = [f"{self.joint_prefix}{j}" for j in self.base_joint_names]

        self.latest_joint_state = None

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

    def _joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def get_ordered_current_positions(self, timeout_sec: float = 5.0):
        """Spins until joint states are received and returns ordered joint values."""
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_joint_state is not None:
                positions = []
                for name in self.target_joint_names:
                    if name in self.latest_joint_state.name:
                        idx = self.latest_joint_state.name.index(name)
                        positions.append(self.latest_joint_state.position[idx])
                
                if len(positions) == 6:
                    return positions

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                raise TimeoutError("Timed out waiting for target joint states.")
                

def write_traj_to_file(traj, filename: str):
    with open(filename, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        # Optional: Add joint names header if needed by your reader protocol
        for row in traj:
            csv_writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description="Generate trajectory relative to current robot state.")
    parser.add_argument("--filename", default="trajectory.csv", help="Output path for CSV trajectory file")
    parser.add_argument("--robot", choices=["left", "right"], default="left", help="Target arm ('left' or 'right')")
    parser.add_argument("--delta", nargs=6, type=float, default=[0.0, 0.0, 0.0, 0.0, -1.0, 0.0],
                        help="Joint displacement offset vector (radians), e.g. 0 0 0 0 -1 0")
    parser.add_argument("--tf", type=float, default=5.0, help="Total execution duration (seconds)")
    parser.add_argument("--te", type=float, default=0.1, help="Time step / sample time (seconds)")

    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    fetcher = JointStateFetcher(arm_selection=parsed_args.robot)

    try:
        fetcher.get_logger().info(f"Fetching current joint position for {parsed_args.robot} arm...")
        qi = fetcher.get_ordered_current_positions()
        fetcher.get_logger().info(f"Current Joint Positions (qi): {qi}")

        # Compute target position qf = qi + delta
        delta = np.array(parsed_args.delta)
        qf = (np.array(qi) + delta).tolist()
        fetcher.get_logger().info(f"Calculated Goal Positions (qf): {qf}")

        # Generate trajectory using moveJ
        traj = moveJ(qi, qf, tf=parsed_args.tf, Te=parsed_args.te)

        # Save to CSV
        write_traj_to_file(traj, parsed_args.filename)
        fetcher.get_logger().info(f"Trajectory successfully written to: {parsed_args.filename}")

    except Exception as err:
        fetcher.get_logger().error(f"Failed to generate trajectory: {err}")
    finally:
        fetcher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()