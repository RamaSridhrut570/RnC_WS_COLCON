#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink, DetachLink  # For gripper control

def main():
    rclpy.init()

    # Create node
    node = Node("ur5_pick_place")

    # Declare parameters for positions and orientation
    node.declare_parameter("position1", [0.20, -0.47, 0.65])  # P1
    node.declare_parameter("position2", [0.75, 0.49, -0.05])  # P2
    node.declare_parameter("position3", [0.75, -0.23, -0.05])  # P3
    node.declare_parameter("position_drop", [-0.69, 0.10, 0.44])  # D
    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])  # Default orientation
    node.declare_parameter("cartesian", False)

    callback_group = ReentrantCallbackGroup()

    # Create MoveIt2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Gripper control services
    attach_gripper = node.create_client(AttachLink, '/GripperMagnetON')
    detach_gripper = node.create_client(DetachLink, '/GripperMagnetOFF')

    # Wait for services
    while not attach_gripper.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for Attach service...')
    while not detach_gripper.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for Detach service...')

    # Spin the node in background
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Get parameters
    position1 = node.get_parameter("position1").get_parameter_value().double_array_value
    position2 = node.get_parameter("position2").get_parameter_value().double_array_value
    position3 = node.get_parameter("position3").get_parameter_value().double_array_value
    position_drop = node.get_parameter("position_drop").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to P1 and pick the box
    node.get_logger().info(f"Moving to P1: {list(position1)}")
    moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    # Attach the gripper
    req_attach = AttachLink.Request()
    req_attach.model1_name = "box1"  # Replace with actual box name
    req_attach.link1_name = "link"
    req_attach.model2_name = "ur5"
    req_attach.link2_name = "wrist_3_link"
    attach_gripper.call_async(req_attach)

    # Move to Drop position D
    node.get_logger().info(f"Moving to Drop: {list(position_drop)}")
    moveit2.move_to_pose(position=position_drop, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    # Detach the gripper
    req_detach = DetachLink.Request()
    req_detach.model1_name = "box1"  # Replace with actual box name
    req_detach.link1_name = "link"
    req_detach.model2_name = "ur5"
    req_detach.link2_name = "wrist_3_link"
    detach_gripper.call_async(req_detach)

    # Move to P2
    node.get_logger().info(f"Moving to P2: {list(position2)}")
    moveit2.move_to_pose(position=position2, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    # Attach the gripper again for next pick/drop cycle
    attach_gripper.call_async(req_attach)

    # Move to Drop position D again
    node.get_logger().info(f"Moving to Drop: {list(position_drop)}")
    moveit2.move_to_pose(position=position_drop, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    # Detach the gripper
    detach_gripper.call_async(req_detach)

    # Move to P3
    node.get_logger().info(f"Moving to P3: {list(position3)}")
    moveit2.move_to_pose(position=position3, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    # Final Drop
    node.get_logger().info(f"Moving to Drop: {list(position_drop)}")
    moveit2.move_to_pose(position=position_drop, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()

