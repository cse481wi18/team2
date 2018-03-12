#! /usr/bin/env python

import rospy
import numpy as np
import math
import core
import copy
import fetch_api

from tf import TransformListener
from std_msgs.msg import String
from perception_msgs.msg import TennisBallPoses
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped

# NOTE: increase force for turning!!!

planner = None

def start_cb(msg):
  print msg.data
  if msg.data == 'start':
    start()

rospy.init_node("core")
messager = core.Messager()
planner = core.Planner(messager, "/map", "scan")
mover = core.Mover(messager)
finder = core.Finder(messager, planner)
grabber = core.Grabber(messager, planner, finder)
listener = TransformListener()


def grab_until_no_balls(first_ball_pose):
  # TEMP CODE - ENTER SEPARATE MODE
  print "Core: ENTER LOCAL MODE"
  tmp = copy.deepcopy(planner.all_points_confidences)
  # planner.message_type = ""
  planner.all_points_confidences = {}
  pose = copy.deepcopy(first_ball_pose)
  # base_link_pose = copy.deepcopy(first_ball_base_link_pose)
  b = True
  while True:
    messager.publish_status1("Local mode - moving arm to unload position...")
    grabber.goto_pose_unload()
    messager.publish_status1("Local mode - facing target...")
    mover.face_pose(pose)
    messager.publish_status1("Local mode - checking distancing and moving back...")
    if b:
      mover.move_back_if_close(pose)
    messager.publish_status1("Local mode - grabbing...")
    grab_res = grabber.grab(pose)
    if grab_res == 1:
      print "Core: local mode grab succeeded"
      planner.reduce_confidence(pose)
      # TODO: reduce confidence in tmp as well

      messager.publish_status1("Local mode - grab succeeded - observing to get next pose...")
      finder.observe_pose(pose, "map")
      all_object_poses = planner.get_pose()
      if all_object_poses is not None and len(all_object_poses) > 0:
        pose = all_object_poses[0]
        b = True
      else:
        break

    elif grab_res == -1:
      messager.publish_status1("Local mode - no balls visible - observing to get next pose...")
      print "Core: local mode grab failed, no balls visible"
      planner.reduce_confidence(pose)

      finder.observe_pose(pose, "map")
      all_object_poses = planner.get_pose()
      if all_object_poses is not None and len(all_object_poses) > 0:
        pose = all_object_poses[0]
        b = False
      else:
        break
    else:
      messager.publish_status1("Local mode - grab failed - moving closer...")
      print "Core: local mode grab failed, moving closer"
      messager.publish_status2("Facing pose...")
      mover.face_pose(pose)
      messager.publish_status2("Moving to grab pose...")
      mover.move_to_grab_pose(pose)
      messager.publish_status2("Facing pose...")
      mover.face_pose(pose)
      b = False



    # object_poseStamped = PoseStamped()
    # object_poseStamped.header.frame_id = "map"
    # object_poseStamped.pose = pose
    # listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
    # base_link_pose = listener.transformPose('/base_link', object_poseStamped).pose
    
  planner.all_points_confidences = tmp
  # planner.message_type = "scan"
  print "Core: EXIT LOCAL MODE"
    # END OF TEMP CODE

def start():
  all_poses = []

  messager.publish_status1("Starting...")

  while True:
    messager.publish_status1("Moving arm to unload position...")
    grabber.goto_pose_unload()

    messager.publish_status1("Gettin pose from Planner...")
    print "Core: Getting pose from planner"
    all_object_poses = planner.get_pose()
    # print "Core: Planner returned", len(all_object_poses), "poses"
    first_pose = None
    if all_object_poses is not None and len(all_object_poses) > 0:
      first_pose = all_object_poses[0]
    else:

      messager.publish_status1("Moving to next anchor...")
      # finder.move_to_next_map_point()

      messager.publish_status1("Looking around...")
      finder.find()
      continue
    # print "Planner returns", res

    print "Core: calling graber to grab"
    messager.publish_status1("Facing pose ...")
    mover.face_pose(first_pose)

    grab_until_no_balls(first_pose)
    planner.reduce_confidence(first_pose)

# if __name__ == "__main__":
#   main()

start_sub = rospy.Subscriber('robot_controller/start_msg', String, start_cb)
print "???"
rospy.spin()