#! /usr/bin/env python

import rospy
import numpy as np
import math
import core
import copy
import fetch_api

from tf import TransformListener
from perception_msgs.msg import TennisBallPoses
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped

# NOTE: increase force for turning!!!

def main():
  rospy.init_node("core")

  all_poses = []

  planner = core.Planner()
  mover = core.Mover()
  finder = core.Finder(planner)
  grabber = core.Grabber(planner, finder)
  while True:
    grabber.goto_pose_unload()
    print "Core: Getting pose from planner"
    planner.confidence_threshold = 10.0
    all_object_poses = planner.get_pose()
    first_pose = None
    if len(all_object_poses) > 0:
      first_pose = all_object_poses[0]
    else:
      finder.find()
      continue
    # print "Planner returns", res


    print "Core: calling graber to grab"
    mover.face_pose(first_pose)

    print "Core: Transforming object pose"
    object_poseStamped = PoseStamped()
    object_poseStamped.header.frame_id = "map"
    object_poseStamped.pose = first_pose
    listener = TransformListener()
    listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
    base_link_pose = listener.transformPose('/base_link', object_poseStamped).pose

    grabber_success = grabber.grab(base_link_pose)
    if grabber_success:
      pass
    else:
      print "Core: grabber failed, moving closer"
      mover.move_to_grab_pose(first_pose)

      listener = TransformListener()
      listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
      base_link_pose2 = listener.transformPose('/base_link', object_poseStamped).pose

      grabber_success2 = grabber.grab(base_link_pose2)
      print "Core: new grabber result:", grabber_success2

    planner.reduce_confidence(first_pose)

  # pose = Pose()

  # pose.position.x = 0.83608096838
  # pose.position.y = 0.0
  # pose.position.z = 0.385022521019

  # pose.orientation.w = 1.0

  # grabber.grab(pose)
  # rospy.spin()

if __name__ == "__main__":
  main()
