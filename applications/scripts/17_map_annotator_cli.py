#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped

HELP_STR = """Welcome to the map annotator!
Commands:
  list: List saved poses.
  save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.
  delete <name>: Delete the pose given by <name>.
  goto <name>: Sends the robot to the pose given by <name>.
  help: Show this list of commands
"""

class Core(object):

  def __init__(self):
    self.poses = {}
    self.lastPose = Pose()
    self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.savePose_cb)
    self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)

  def listPoses(self):
    return self.poses
    # names = []
    # for key in self.poses:
    #     names.append(key)
    # return names

  def savePose(self, name):
    self.poses[name] = self.lastPose

  def savePose_cb(self, msg):
    self.lastPose = msg.pose.pose

  def deletePose(self, name):
    if name in self.poses:
      del self.poses[name] 

  def gotoPose(self, name):     
    pose = self.poses[name]

    # pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)

    poseStamped = PoseStamped()
    poseStamped.header.frame_id = "map"
    poseStamped.pose = pose
    self.goal_pub.publish(poseStamped)


def main():
  rospy.init_node("lab17")
  print HELP_STR

  core = Core()
  while True:  
    line = sys.stdin.readline()
    tokens = line.rstrip().split(' ', 1)
    cmd = tokens[0]
    name = ""
    if len(tokens) > 1:
      name = tokens[1]

    if cmd == "list":
      print core.listPoses()
    elif cmd == "save":
      core.savePose(name)
    elif cmd == "delete":
      core.deletePose(name)
    elif cmd == "goto":
      core.gotoPose(name)
    elif cmd == "exit":
      break
    else:
      print HELP_STR

  

if __name__ == '__main__':
  main()
