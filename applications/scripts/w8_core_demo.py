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
  #   all_poses.extend(return_poses)
  # while return_poses is None:
  #   return_poses = plan.get_pose()
  #   all_poses.extend(return_poses)
  while True:
    all_poses = plan.get_pose()
    print(all_poses)
    rospy.sleep(1)
  rospy.spin()

if __name__ == "__main__":
  main()