#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by AutomaticAddison.com

import time # Time library

from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from nav2_simple_commander.robot_navigator import BasicNavigator# Helper module

'''
Navigates a robot through goal poses.
'''
def main():

  # Start the ROS 2 Python Client Library
  rclpy.init()


  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator("basic_navigator")

  # Set the robot's initial pose if necessary
  # initial_pose = PoseStamped()
  # initial_pose.header.frame_id = 'map'
  # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  # initial_pose.pose.position.x = -6.5
  # initial_pose.pose.position.y = -4.2
  # initial_pose.pose.position.z = 0.0
  # initial_pose.pose.orientation.x = 0.0
  # initial_pose.pose.orientation.y = 0.0
  # initial_pose.pose.orientation.z = 0.0
  # initial_pose.pose.orientation.w = 1.0
  # navigator.setInitialPose(initial_pose)

  # Activate navigation, if not autostarted. This should be called after setInitialPose()
  # or this will initialize at the origin of the map and update the costmap with bogus readings.
  # If autostart, you should `waitUntilNav2Active()` instead.
  # navigator.lifecycleStartup()

  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active(localizer='controller_server')

  # If desired, you can change or load the map as well
  # navigator.changeMap('/path/to/map.yaml')

  # You may use the navigator to clear or obtain costmaps
  # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
  # global_costmap = navigator.getGlobalCostmap()
  # local_costmap = navigator.getLocalCostmap()

  # Set the robot's goal poses
  goal_poses1 = []
  goal_poses2 = []
  follow_waypoints1 = []
  follow_waypoints2 = []
  follow_waypoints3 = []
  
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 9.0#20.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  follow_waypoints1.append(goal_pose)

  #goal_pose = PoseStamped()
  #goal_pose.header.frame_id = 'map'
  #goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  #goal_pose.pose.position.x = 10.0#20.0
  #oal_pose.pose.position.y = 0.0
  #oal_pose.pose.position.z = 0.0
  #goal_pose.pose.orientation.x = 0.0
  #goal_pose.pose.orientation.y = 0.0
  #goal_pose.pose.orientation.z = 0.0
  #goal_pose.pose.orientation.w = 1.0
  #follow_waypoints1.append(goal_pose)

  #goal_pose = PoseStamped()
  #goal_pose.header.frame_id = 'map'
  #goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  #goal_pose.pose.position.x = 10.5#22.0
  #oal_pose.pose.position.y = 0.5
  #oal_pose.pose.position.z = 0.0
  #goal_pose.pose.orientation.x = 0.0
  #goal_pose.pose.orientation.y = 0.0
  #goal_pose.pose.orientation.z = 0.38
  #goal_pose.pose.orientation.w = 0.92
  #goal_poses1.append(goal_pose)
  
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 10.5#22.0
  goal_pose.pose.position.y = 2.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.707
  goal_pose.pose.orientation.w = 0.707
  goal_poses1.append(goal_pose)

  #goal_pose = PoseStamped()
  #goal_pose.header.frame_id = 'map'
  #goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  #oal_pose.pose.position.x = 10.0#20.0
  #goal_pose.pose.position.y = 2.0
  #goal_pose.pose.position.z = 0.0
  #goal_pose.pose.orientation.x = 0.0
  #goal_pose.pose.orientation.y = 0.0
  #goal_pose.pose.orientation.z = 0.92
  #oal_pose.pose.orientation.w = 0.38
  #goal_poses1.append(goal_pose)
  
  #goal_pose = PoseStamped()
  #goal_pose.header.frame_id = 'map'
  #goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  #goal_pose.pose.position.x = 10.0#20.0
  #goal_pose.pose.position.y = 2.0
  #goal_pose.pose.position.z = 0.0
  #goal_pose.pose.orientation.x = 0.0
  #goal_pose.pose.orientation.y = 0.0
  #goal_pose.pose.orientation.z = 1.0
  #goal_pose.pose.orientation.w = 0.0
  #goal_poses1.append(goal_pose)

  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 9.0#20.0
  goal_pose.pose.position.y = 4.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 1.0
  goal_pose.pose.orientation.w = 0.0
  goal_poses1.append(goal_pose)
  
  #goal_pose = PoseStamped()
  #goal_pose.header.frame_id = 'map'
  #oal_pose.header.stamp = navigator.get_clock().now().to_msg()
  #oal_pose.pose.position.x = 0.0
  #goal_pose.pose.position.y = 2.0
  #goal_pose.pose.position.z = 0.0
  #goal_pose.pose.orientation.x = 0.0
  #goal_pose.pose.orientation.y = 0.0
  #goal_pose.pose.orientation.z = 1.0
  #goal_pose.pose.orientation.w = 0.0
  #follow_waypoints2.append(goal_pose)

  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -9.0
  goal_pose.pose.position.y = 4.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 1.0
  goal_pose.pose.orientation.w = 0.0
  follow_waypoints2.append(goal_pose)

  #goal_pose = PoseStamped()
  #goal_pose.header.frame_id = 'map'
  #goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  #goal_pose.pose.position.x = -10.0
  #goal_pose.pose.position.y = 2.0
  #goal_pose.pose.position.z = 0.0
  #goal_pose.pose.orientation.x = 0.0
  #goal_pose.pose.orientation.y = 0.0
  #goal_pose.pose.orientation.z = 1.0
  #goal_pose.pose.orientation.w = 0.0
  #follow_waypoints2.append(goal_pose)

  #goal_pose = PoseStamped()
  #goal_pose.header.frame_id = 'map'
  #goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  #goal_pose.pose.position.x = -10.5
  #goal_pose.pose.position.y = 1.5
  #goal_pose.pose.position.z = 0.0
  #goal_pose.pose.orientation.x = 0.0
  #goal_pose.pose.orientation.y = 0.0
  #goal_pose.pose.orientation.z = 0.92
  #goal_pose.pose.orientation.w = -0.38
  #goal_poses2.append(goal_pose)

  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -10.5
  goal_pose.pose.position.y = 2.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.707
  goal_pose.pose.orientation.w = -0.707
  goal_poses2.append(goal_pose)

  #goal_pose = PoseStamped()
  #goal_pose.header.frame_id = 'map'
  #goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  #goal_pose.pose.position.x = -10.0
  #goal_pose.pose.position.y = 0.0
  #goal_pose.pose.position.z = 0.0
  #goal_pose.pose.orientation.x = 0.0
  #goal_pose.pose.orientation.y = 0.0
  #goal_pose.pose.orientation.z = 0.38
  #goal_pose.pose.orientation.w = -0.92
  #goal_poses2.append(goal_pose)

  #goal_pose = PoseStamped()
  #goal_pose.header.frame_id = 'map'
  #oal_pose.header.stamp = navigator.get_clock().now().to_msg()
  #oal_pose.pose.position.x = -10.0
  #goal_pose.pose.position.y = 0.0
  #oal_pose.pose.position.z = 0.0
  #goal_pose.pose.orientation.x = 0.0
  #goal_pose.pose.orientation.y = 0.0
  #goal_pose.pose.orientation.z = 0.0
  #goal_pose.pose.orientation.w = 1.0
  #goal_poses2.append(goal_pose)

  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -9.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  goal_poses2.append(goal_pose)

  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 0.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  follow_waypoints3.append(goal_pose)

  
  

  # sanity check a valid path exists
  # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
  for i in range(2):

    # Go through the goal poses
    #navigator.goThroughPoses(goal_poses)
    #navigator.goToPose(goal_poses)
    navigator.followWaypoints(follow_waypoints1)
    while not navigator.isTaskComplete():
      pass

    navigator.goThroughPoses(goal_poses1)
    while not navigator.isTaskComplete():
      pass

    navigator.followWaypoints(follow_waypoints2)
    while not navigator.isTaskComplete():
      pass

    navigator.goThroughPoses(goal_poses2)
    while not navigator.isTaskComplete():
      pass

    navigator.followWaypoints(follow_waypoints3)
    while not navigator.isTaskComplete():
      pass



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

  # Close the ROS 2 Navigation Stack
  navigator.lifecycleShutdown()

  exit(0)

if __name__ == '__main__':
  main()
