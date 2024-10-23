import time  # Add this to use time.sleep()
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Create the nodes for publishing odom and tf
    odom_publisher = rclpy.create_node('odom_publisher')
    odom_pub = odom_publisher.create_publisher(PoseStamped, 'odom', 10)
    
    tf_publisher = rclpy.create_node('tf_publisher')
    tf_pub = tf_publisher.create_publisher(PoseStamped, 'tf', 10)

    # Initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0  # Correct neutral orientation
    initial_pose.pose.orientation.z = 0.0
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()

    # Positions and orientations (in Euler angles)
    positions = [
        [0.0, 0.0, 0.0],  # Initial position (not used in loop)
        [-0.12, -2.35, 3.14],  # Second position
        [1.86, 2.56, 0.97],    # Third position
        [-3.84, 2.64, 2.78]    # Fourth position
    ]

    # Loop over the positions
    for a in range(1, len(positions)):
        # Define the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = odom_publisher.get_clock().now().to_msg()

        goal_pose.pose.position.x = positions[a][0]
        goal_pose.pose.position.y = positions[a][1]

        # Convert yaw (z-axis rotation) to a quaternion
        quaternion = quaternion_from_euler(0.0, 0.0, positions[a][2])
        goal_pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # Navigate to the goal pose
        nav.goToPose(goal_pose)
        time.sleep(1)
        # Wait for task completion
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
                
            time.sleep(1)  # Optional delay to avoid printing too frequently
        
        # Check result
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            tf_pub.publish(goal_pose)
            odom_pub.publish(goal_pose)
            
            print('Goal succeeded!')
            feedback = nav.getFeedback()

            if feedback:
                # Extract and print the current position [x, y]
                current_position = feedback.current_pose.pose.position

                # Extract quaternion orientation and convert to Euler angles
                orientation_q = feedback.current_pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                _, _, yaw = euler_from_quaternion(orientation_list)  # Extract the yaw

                print(f"Current position: x={current_position.x}, y={current_position.y}, yaw={yaw}")

                
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

    # Add delay to allow the bot to settle on its final orientation
    time.sleep(2)  # Adjust time as needed for proper orientation

    # Properly shut down navigation
    nav.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
