#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    # --- Init
    rclpy.init()

    nav = BasicNavigator()

    # Robotics Lab Initial Pose
    r_spawn_x_val = 0.0104361
    r_spawn_y_val = -0.0174723
    r_spawn_yaw_val = 0.000796943
    r_initial_pose = create_pose_stamped(nav, r_spawn_x_val, r_spawn_y_val, r_spawn_yaw_val)

    # 4th Floor Initial Pose
    f_spawn_x_val = 3.39665 
    f_spawn_y_val = -8.20401
    f_spawn_yaw_val = -3.1408
    f_initial_pose = create_pose_stamped(nav, f_spawn_x_val, f_spawn_y_val, f_spawn_yaw_val)
    
    # --- Set initial pose
    # nav.setInitialPose(f_initial_pose)

    # --- Wait for Nav2
    nav.waitUntilNav2Active()

    # --- Send Nav2 goal

    # --- Nav2 Goal Poses ---
    # goal_pose1 = create_pose_stamped(nav, 3.5, 1.0, 1.57)
    # goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)
    # goal_pose3 = create_pose_stamped(nav, 0.5, 1.0, 0.0)

    # --- Go to one pose
    # nav.goToPose(goal_pose1)
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     # print(feedback)

    # --- Nav2 Waypoints ---
    # Robotics Waypoints
    r_waypoints = []
    r_waypoints.append(create_pose_stamped(nav, 2.38756, -2.92152, -1.54369))
    r_waypoints.append(create_pose_stamped(nav, 2.43836, -5.59837, 1.57))
    r_waypoints.append(create_pose_stamped(nav, 0.0612044, -0.00312877, 0.000796839))

    # 4th Floor Waypoints
    f_waypoints = []
    f_waypoints.append(create_pose_stamped(nav, 3.30424, -8.06772, -3.1408))
    f_waypoints.append(create_pose_stamped(nav, -5.49223, -8.00682, -3.1408))
    f_waypoints.append(create_pose_stamped(nav, -13.9684, -8.10684, -3.1408))
    f_waypoints.append(create_pose_stamped(nav, -16.7415, -6.89626, 0.00079735))
    f_waypoints.append(create_pose_stamped(nav, -5.49223, -8.00682, -3.1408))
    f_waypoints.append(create_pose_stamped(nav, 3.39665, -8.20401, -3.1408))

    # --- Follow waypoints
    nav_start = nav.get_clock().now()
    nav.followWaypoints(r_waypoints)
    i = 0
    while not nav.isTaskComplete():
        i = i + 1
        feedback = nav.getFeedback()
        if feedback and i % 5 == 0:
            print("Executing current waypoint: " + 
                  str(feedback.current_waypoint + 1) + "/" + str(len(r_waypoints)))
            now = nav.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                nav.cancelTask()

    # Do something depending on the return code
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # --- Shutdown
    # nav.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()