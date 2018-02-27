#! /usr/bin/env python

import rospy
import numpy as np
import math
import core
import fetch_api

from perception_msgs.msg import TennisBallPoses
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped

def main():
  rospy.init_node("core")

  all_poses = []

  plan = core.Planner()
  mover = core.Mover()
  grabber = core.Grabber()
  rospy.sleep(5)
  #   all_poses.extend(return_poses)
  # while return_poses is None:
  #   return_poses = plan.get_pose()
  #   all_poses.extend(return_poses)

  all_poses = plan.get_pose()
  print(all_poses)
  print plan.goto_first_pose()

  # pose = Pose()

  # pose.position.x = 0.83608096838
  # pose.position.y = 0.0
  # pose.position.z = 0.385022521019

  # pose.orientation.w = 1.0

  # grabber.move(pose)
  # rospy.spin()

if __name__ == "__main__":
  main()
