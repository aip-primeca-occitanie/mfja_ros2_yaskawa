#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest

class MoveGroupClient(Node):
    def __init__(self):
        super().__init__('move_group_python_client', namespace='yaskawa_LEFT')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_joint_goal(self, group_name, joint_goals):
        self._action_client.wait_for_server()

        req = MotionPlanRequest()
        req.group_name = group_name
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 10

        # Build joint constraints
        constraints = Constraints()
        for j_name, target_val in joint_goals.items():
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = target_val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False  # Set False to Execute on real robot

        self.get_logger().info('Sending motion goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = MoveGroupClient()

    targets = {
        "joint_1": 0.0,
        "joint_2": 0.0,
        "joint_3": 0.0,
        "joint_4": 0.0,
        "joint_5": 0.0,
        "joint_6": 0.0
    }
    
    node.send_joint_goal("DN2P1", targets)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()