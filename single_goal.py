#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from pose_subscriber import PoseSubscriber
"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()
    pose_subscriber = PoseSubscriber()
    navigator = BasicNavigator()

    while rclpy.ok():
          rclpy.spin_once(pose_subscriber, timeout_sec=0.1)
          if pose_subscriber.initial_pose is not None:
              navigator.setInitialPose(pose_subscriber.initial_pose)
              break
            
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.298
    goal_pose.pose.position.y = -1.388
    goal_pose.pose.orientation.w = 0.78

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        #i = i + 1
        #feedback = navigator.getFeedback()
        #if feedback and i % 5 == 0:
         #   print('Estimated time of arrival: ' + '{0:.0f}'.format(
        #        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
        #          + ' seconds.')

            # Some navigation timeout to demo cancellation
        #    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
        #        navigator.cancelTask()

            # Some navigation request change to demo preemption
         #   if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
         #       goal_pose.pose.position.x = -3.0
        #        navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        rclpy.spin_once(pose_subscriber, timeout_sec=0.1)
        feedback = navigator.getFeedback()
        if feedback and feedback.estimated_time_remaining:
            eta_seconds = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f'Estimated time of arrival: {eta_seconds:.0f} seconds.')

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = -3.0  # Change goal position
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    pose_subscriber.destroy_node()
    rclpy.shutdown()
    #exit(0)


if __name__ == '__main__':
    main()


CMakelist.txt


cmake_minimum_required(VERSION 3.5)
project(smart_robots)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_simple_commander REQUIRED)

# Install Python modules and scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/pose_subscriber.py
  scripts/navigation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


Package.xml

<?xml version="1.0"?>
<package format="3">
  <name>smart_robots</name>
  <version>0.0.1</version>
  <description>A sample ROS 2 package for navigation with pose subscription and goal setting.</description>

  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_simple_commander</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
