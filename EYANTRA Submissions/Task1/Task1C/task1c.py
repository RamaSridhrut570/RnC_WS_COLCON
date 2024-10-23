#!/usr/bin/env python3
"""
Example of moving to a pose goal with an intermediate position before picking up the box, 
with ArUco detection running in the background.
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
from task1b_boiler_plate import *

BOX_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack_box.stl"
)
RACK_MESH= path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack.stl")

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        
        # Create service clients for AttachLink and DetachLink
        self.attach_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_client = self.create_client(DetachLink, '/GripperMagnetOFF')

        # Wait for the AttachLink service to be available
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper ON service not available, waiting again...')

        # Wait for the DetachLink service to be available
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper OFF service not available, waiting again...')

    def attach_gripper(self, box_name):
        req = AttachLink.Request()
        req.model1_name = box_name      
        req.link1_name = 'link'         
        req.model2_name = 'ur5'         
        req.link2_name = 'wrist_3_link' 

        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Gripper attached successfully')
        else:
            self.get_logger().error('Failed to attach gripper')

    def detach_gripper(self, box_name):
        req = DetachLink.Request()
        req.model1_name = box_name      
        req.link1_name = 'link'         
        req.model2_name = 'ur5'         
        req.link2_name = 'wrist_3_link' 

        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Gripper detached successfully')
        else:
            self.get_logger().error('Failed to detach gripper')

def aruco_detection():
    rclpy.init(args=None)
    aruco_tf_node = aruco_tf()
    aruco_tf_executor = rclpy.executors.SingleThreadedExecutor()

    aruco_tf_executor.add_node(aruco_tf_node)
    try:
        # ArUco detection runs indefinitely in this thread
        aruco_tf_executor.spin()
    finally:
        aruco_tf_executor.shutdown()
        aruco_tf_node.destroy_node()

def main():
    rclpy.init()  # Initialize rclpy once
    node = Node("ex_pose_goal")
    gripper_client = GripperClient()

    # Initialize and start the ArUco detection node in a separate thread
    aruco_tf_node = aruco_tf()  # Create the ArUco detection node
    aruco_tf_executor = rclpy.executors.SingleThreadedExecutor()
    aruco_tf_executor.add_node(aruco_tf_node)

    # Start the ArUco TF node in a separate thread
    aruco_tf_thread = Thread(target=aruco_tf_executor.spin, daemon=True)
    aruco_tf_thread.start()

    # Positions and orientations for the boxes
    posarray = [[0.20, -0.47, 0.65], [0.73, 0.40, -0.05], [0.75, -0.23, -0.05]]
    quatarray = [[0.5, 0.5, -0.5, 0.5], [0.0, 1.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
    ids = [1, 3, 49]  # Box IDs

    # Joint positions for dropping the boxes
    joint_positions_drop = [
        [-0.03403341798766224, -1.74533, -1.39626, -3.14159, -1.5708, 3.14159],   # First and third box
        [-0.03403341798766224, -2.35619, -0.785398, -3.14159, -1.5708, 3.14159]   # Second box (-135, -45 degrees for J1, J2)
    ]

    # Initial joint positions (same for all boxes)
    initial_joint_positions = [
        0.0,                # 0 degrees base
        -2.39,              # -137 degrees shoulder
        2.4,                # 137.5 degrees elbow
        -3.15,              # -180.5 degrees wrist 1
        -1.58,              # -90.5 degrees wrist 2
        3.15                # 180.5 degrees wrist 3
    ]

    # Intermediate joint position before picking up the box
    intermediate_joint_positions = [
        0.0,                # 0 degrees base
        -1.57,              # -90 degrees shoulder
        1.57,               # 90 degrees elbow
        -1.57,              # -90 degrees wrist 1
        -1.57,              # -90 degrees wrist 2
        3.14159             # 180 degrees wrist 3
    ]
    
    # Declare parameters once
    node.declare_parameter("quat_xyzw", [0.5, 0.5, -0.5, 0.5])
    node.declare_parameter("cartesian", True)
    node.declare_parameter("joint_positions", joint_positions_drop[0])  # Default to first box drop configuration
    node.declare_parameter("initial_joint_positions", initial_joint_positions)
    
    # Create callback group
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Adding collision mesh
    mesh_id1 = path.basename(BOX_MESH).split(".")[0]
    mesh_id2 = path.basename(RACK_MESH).split(".")[0]
    node.get_logger().info(f"Adding collision mesh '{RACK_MESH}' at position [0.3, -0.6, -0.6]")
    moveit2.add_collision_mesh(filepath=RACK_MESH, id=mesh_id2, position=[0.3, -0.6, -0.6], quat_xyzw=[0.0, 0.0, 0.707, 0.707], frame_id=ur5.base_link_name())
    moveit2.add_collision_mesh(filepath=RACK_MESH, id=mesh_id2, position=[0.3, -0.6, -0.6], quat_xyzw=[0.0, 0.0, 0.707, 0.707], frame_id=ur5.base_link_name())
    moveit2.add_collision_mesh(filepath=RACK_MESH, id=mesh_id2, position=[0.3, -0.6, -0.6], quat_xyzw=[0.0, 0.0, 0.707, 0.707], frame_id=ur5.base_link_name())

    # Add box collision mesh
    node.get_logger().info(f"Adding collision mesh '{BOX_MESH}' at position [-0.4, 0.1, -0.1]")
    moveit2.add_collision_mesh(filepath=BOX_MESH, id=mesh_id1, position=[-0.4, 0.1, -0.1], quat_xyzw=[0.0, 0.707, 0.0, 0.707], frame_id=ur5.base_link_name())
    sleep(2)  # Wait for meshes to settle

    # Iterate over boxes
    for i in range(3):
        # Set box-specific position and quaternion
        position = posarray[i]
        quat_xyzw = quatarray[i]
        
        # Move to the pose (pick up box)
        node.get_logger().info(f"Moving to position {position} with orientation {quat_xyzw}")
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw)
        moveit2.wait_until_executed()

        # Attach the gripper to the box
        gripper_client.attach_gripper(f"box{ids[i]}")
        
        # Drop the box (use appropriate joint positions for each drop)
        node.get_logger().info("Moving to drop position")
        moveit2.move_to_configuration(joint_positions_drop[i % 2])
        moveit2.wait_until_executed()

        # Detach the gripper
        gripper_client.detach_gripper(f"box{ids[i]}")

    # Stop ArUco detection
    aruco_tf_executor.shutdown()
    aruco_tf_thread.join()

    # Shutdown the node
    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
    main()
