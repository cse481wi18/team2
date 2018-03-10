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

rospy.init_node("core")
planner = core.Planner("/map")
mover = core.Mover()
finder = core.Finder(planner)
grabber = core.Grabber(planner, finder)
listener = TransformListener()

def grab_until_no_balls(first_ball_pose):
  # TEMP CODE - ENTER SEPARATE MODE
  print "Core: ENTER LOCAL MODE"
  tmp = copy.deepcopy(planner.all_points_confidences)
  planner.all_points_confidences = {}
  pose = copy.deepcopy(first_ball_pose)
  # base_link_pose = copy.deepcopy(first_ball_base_link_pose)
  while True:
    grabber.goto_pose_unload()
    # mover.face_pose(pose)
    grab_res = grabber.grab(pose)
    if grab_res == 1:
      print "Core: local mode grab succeeded"
      planner.reduce_confidence(pose)
      # TODO: reduce confidence in tmp as well

      all_object_poses = planner.get_pose()
      if len(all_object_poses) > 0:
        pose = all_object_poses[0]
      else:
        break

    elif grab_res == -1:
      print "Core: local mode grab failed, no balls visible"
      planner.reduce_confidence(pose)

      all_object_poses = planner.get_pose()
      if len(all_object_poses) > 0:
        pose = all_object_poses[0]
      else:
        break
    else:
      print "Core: local mode grab failed, moving closer"
      mover.face_pose(pose)
      mover.move_to_grab_pose(pose)
      mover.face_pose(pose)



    # object_poseStamped = PoseStamped()
    # object_poseStamped.header.frame_id = "map"
    # object_poseStamped.pose = pose
    # listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
    # base_link_pose = listener.transformPose('/base_link', object_poseStamped).pose
    
  planner.all_points_confidences = tmp
  print "Core: EXIT LOCAL MODE"
    # END OF TEMP CODE

def main():
  all_poses = []
  current_map_pose_found_ball = False

  while True:
    grabber.goto_pose_unload()
    print "Core: Getting pose from planner"
    all_object_poses = planner.get_pose()
    print "Core: Planner returned", len(all_object_poses), "poses"
    first_pose = None
    if len(all_object_poses) > 0:
      current_map_pose_found_ball = True # May need 
      first_pose = all_object_poses[0]
    else:
      if current_map_pose_found_ball is False:
        finder.move_to_next_map_point()
      else:
        finder.move_to_next_map_point()
      finder.find()
      current_map_pose_found_ball = False
      continue
    # print "Planner returns", res

    print "Core: calling graber to grab"
    mover.face_pose(first_pose)

    grab_until_no_balls(first_pose)
    planner.reduce_confidence(first_pose)


if __name__ == "__main__":
  main()
