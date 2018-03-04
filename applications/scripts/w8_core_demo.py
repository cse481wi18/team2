#! /usr/bin/env python

import rospy
import numpy as np
import math
import core
import fetch_api

from tf import TransformListener
from perception_msgs.msg import TennisBallPoses
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped

def main():
  rospy.init_node("core")

  all_poses = []

  planner = core.Planner()
  mover = core.Mover()
  grabber = core.Grabber(planner)
  while True:
    rospy.sleep(3)
    #   all_poses.extend(return_poses)
    # while return_poses is None:
    #   return_poses = plan.get_pose()
    #   all_poses.extend(return_poses)
    res = planner.get_pose()
    # print "Planner returns", res
    all_pick_up_poses = res["pickup_poses"]
    all_object_poses = res["object_poses"]

    poseStamped = mover.goto_pose(all_pick_up_poses[0])
    object_poseStamped = PoseStamped()
    object_poseStamped.header.frame_id = "map"
    object_poseStamped.pose = all_object_poses[0]

    listener = TransformListener()
    listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
    base_link_pose = listener.transformPose('/base_link', object_poseStamped).pose
    grabber.move(base_link_pose)
  # pose = Pose()

  # pose.position.x = 0.83608096838
  # pose.position.y = 0.0
  # pose.position.z = 0.385022521019

  # pose.orientation.w = 1.0

  # grabber.move(pose)
  # rospy.spin()

if __name__ == "__main__":
  main()
