#!/usr/bin/env python3
"""
Optimized: Move UR5 to pick and drop boxes quickly, removing intermediate steps.
"""
from time import sleep
from os import path
from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink, DetachLink

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self.attach_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_client = self.create_client(DetachLink, '/GripperMagnetOFF')

        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            pass
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            pass

    def attach_gripper(self, box_name):
        req = AttachLink.Request()
        req.model1_name = box_name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def detach_gripper(self, box_name):
        req = DetachLink.Request()
        req.model1_name = box_name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main():
    rclpy.init()
    node = Node("optimized_pose_goal")
    gripper_client = GripperClient()

    posarray = [[0.20, -0.47, 0.65], [0.73, 0.40, -0.05], [0.75, -0.23, -0.05]]
    quatarray = [[0.5, 0.5, -0.5, 0.5], [0.0, 1.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
    ids = [1, 3, 49]  # Box IDs
    
    joint_positions_drop = [
        [-0.03403341798766224, -1.74533, -1.39626, -3.14159, -1.5708, 3.14159],
        [-0.03403341798766224, -2.35619, -0.785398, -3.14159, -1.5708, 3.14159]
    ]

    initial_joint_positions = [
        0.0, -2.39, 2.4, -3.15, -1.58, 3.15
    ]

    callback_group = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    for i in range(3):
        # Move to the position (pick up box)
        position = posarray[i]
        quat_xyzw = quatarray[i]
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw)
        moveit2.wait_until_executed()

        # Attach the gripper to the box
        gripper_client.attach_gripper(f"box{ids[i]}")

        # Move to drop position
        moveit2.move_to_configuration(joint_positions_drop[i % 2])
        moveit2.wait_until_executed()

        # Detach the gripper
        gripper_client.detach_gripper(f"box{ids[i]}")

    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
    main()
