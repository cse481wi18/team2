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
    planner.confidence_drop_rate = 0.90
    planner.unpause()
    rospy.sleep(2)
    planner.pause()
    #   all_poses.extend(return_poses)
    # while return_poses is None:
    #   return_poses = plan.get_pose()
    #   all_poses.extend(return_poses)
    print "Core: Getting pose from planner"
    all_object_poses = planner.get_pose()
    first_pose = all_object_poses[0]
    # print "Planner returns", res
    # poseStamped = mover.goto_pose(all_pick_up_poses[0])

    print "Core: Transforming object pose"
    object_poseStamped = PoseStamped()
    object_poseStamped.header.frame_id = "map"
    object_poseStamped.pose = first_pose

    listener = TransformListener()
    listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
    base_link_pose = listener.transformPose('/base_link', object_poseStamped).pose

    print "Core: calling graber to grab"
    grabber_success = grabber.grab(base_link_pose)
    if grabber_success:
      planner.reduce_confidence(first_pose)
    else:
      print "Core: grabber failed, moving closer"
      mover.move_to_grab_pose(first_pose)

      listener = TransformListener()
      listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
      base_link_pose2 = listener.transformPose('/base_link', object_poseStamped).pose

      grabber_success2 = grabber.grab(base_link_pose2)
      print "Core: new grabber result:", grabber_success2

  # pose = Pose()

  # pose.position.x = 0.83608096838
  # pose.position.y = 0.0
  # pose.position.z = 0.385022521019

  # pose.orientation.w = 1.0

  # grabber.grab(pose)
  # rospy.spin()

if __name__ == "__main__":
  main()
