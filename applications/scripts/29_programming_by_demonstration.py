#! /usr/bin/env python

import rospy
import sys
import fetch_api
import copy
import tf.transformations as tft
import numpy as np
import pickle

from ar_track_alvar_msgs.msg import AlvarMarkers
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker, MenuEntry

def newMarker(name, x, y, z):
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = "base_link"
  int_marker.name = name
  int_marker.description = ""
  int_marker.pose.position.x = x
  int_marker.pose.position.y = y
  int_marker.pose.position.z = z
  int_marker.pose.orientation.w = 1

  box_marker = Marker()
  box_marker.type = Marker.ARROW
  box_marker.pose.orientation.y = 1
  box_marker.pose.orientation.w = 1
  box_marker.scale.x = 0.13
  box_marker.scale.y = 0.08
  box_marker.scale.z = 0.08
  box_marker.color.r = 1.0
  box_marker.color.g = 0.85
  box_marker.color.b = 0.0
  box_marker.color.a = 1.0

  button_control = InteractiveMarkerControl()
  button_control.interaction_mode = InteractiveMarkerControl.BUTTON
  button_control.always_visible = True
  button_control.markers.append(box_marker)
  int_marker.controls.append(button_control)
  return int_marker

def pose_to_matrix(pose):
  p = pose.position
  r = pose.orientation
  T = tft.translation_matrix([p.x, p.y, p.z])
  R = tft.quaternion_matrix([r.x, r.y, r.z, r.w])
  return R + T - np.identity(4)

def pose_to_pose_stamped(pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "base_link"
    pose_stamped.header.stamp = rospy.Time(0)
    return pose_stamped

def matrix_to_pose(matrix):
  pose = Pose()
  (rx, ry, rz, w) = tft.quaternion_from_matrix(matrix)
  (tx, ty, tz) = tft.translation_from_matrix(matrix)
  pose.orientation = Quaternion(rx, ry, rz, w)
  pose.position = Point(tx, ty, tz)
  return pose

class ArTagReader(object):
  def __init__(self):
    self.markers = {}
    self.pbd = None

  def callback(self, msg):
    self.markers = {}
    for m in msg.markers:
      self.markers[m.id] = m.pose.pose

    # self.markers = list(map(lambda m: m.pose.pose, msg.markers))
    if self.pbd is not None:
      self.pbd.change_marker(self.markers)

class PbD():
  GO_TO_POSE_COMMAND = 1
  OPEN_GRIPPER_COMMAND = 2
  CLOSE_GRIPPER_COMMAND = 3

  def __init__(self, server, ar_tag_reader, base, arm, gripper):
    try: 
        pickle_in = open("29_pbd.pickle", "rb")
        self.programs = pickle.load(pickle_in)
    except IOError:
        self.programs = {}

    self.curr_program_name = ""
    self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.savePose_cb)
    self.curr_pose = None
    self.curr_program = []

    self.server = server
    self.base = base
    self.arm = arm
    self.gripper = gripper
    self.ar_tag_reader = ar_tag_reader

    self.marker_menu_names = {}

  def create_program(self):
    print "what is the name of this program???? "
    # prompt user for the name
    name = sys.stdin.readline()
    self.curr_program_name = name.rstrip()
    self.curr_program = []
    # if rospy.get_param("use_sim_time") is None:
    self.arm.relax()
    print "RELAXED"
    # else:
    #   self.arm.unrelax()
    
  def save_program(self):
    self.programs[self.curr_program_name] = copy.deepcopy(self.curr_program)
    pickle_out = open("29_pbd.pickle", "wb")
    pickle.dump(self.programs, pickle_out)
    print "Program saved: \n", self.programs

  def save_pose_raw(self, num):
    #gripper_pos = self.curr_pose
    gripper_pos = self.arm.get_gripper_pose()
    if num == 0:
      self.save_pose(gripper_pos, num)
    else:
      marker_id = self.marker_menu_names[num - 1]
      # Tw2 = Tg2 * Tg1^-1 * Tw1
      print gripper_pos
      Tw1 = pose_to_matrix(gripper_pos)
      Tg1 = pose_to_matrix(self.ar_tag_reader.markers[marker_id])
      Tw_Tg = np.linalg.inv(Tg1).dot(Tw1)

      self.save_pose(matrix_to_pose(Tw_Tg), marker_id)
    print "Pose saved relative to #%d" % marker_id

  def save_pose(self, pos, marker_id):
    self.curr_program.append({
      'command': PbD.GO_TO_POSE_COMMAND,
      'pose': copy.deepcopy(pos),
      'relative_to': marker_id
    })

  def open_gripper(self):
    self.gripper.open()
    self.curr_program.append({
      'command': PbD.OPEN_GRIPPER_COMMAND
    })
    print "Open gripper action saved"

  def close_gripper(self):
    self.gripper.close()
    self.curr_program.append({
      'command': PbD.CLOSE_GRIPPER_COMMAND
    })
    print "Close gripper action saved"

  def print_programs(self):
    print self.programs

  def savePose_cb(self, msg):
    print "pose updated"
    self.curr_pose = msg.pose.pose

  def load_program_raw(self):
    print "what is the name of the program to load???? "
    name = sys.stdin.readline().rstrip()
    
    if name in self.programs.keys():
      print "loading %s..." % name
      self.load_program(self.programs[name])
    else:
      print self.programs[name]
      print "no such program!!1!"

  def load_program(self, program):
    self.arm.unrelax()
    i = 0
    for action in program:
      if action['command'] == PbD.GO_TO_POSE_COMMAND:
        marker_id = action['relative_to']
        print "Action", i, "-", "go to pose relative to", marker_id
        pose = action['pose']
        if marker_id == 0:
          print pose
          self.arm.move_to_pose(pose_to_pose_stamped(pose))
        else:
          Tw_Tg = pose_to_matrix(pose)
          Tg2 = pose_to_matrix(self.ar_tag_reader.markers[marker_id])
          Tw2 = Tg2.dot(Tw_Tg)
          print matrix_to_pose(Tw2)
          self.arm.move_to_pose(pose_to_pose_stamped(matrix_to_pose(Tw2)))
          pass

      elif action['command'] == PbD.OPEN_GRIPPER_COMMAND:
        print "Action", i, "-", "open"
        self.gripper.open()
      elif action['command'] == PbD.CLOSE_GRIPPER_COMMAND:
        print "Action", i, "-", "close"
        self.gripper.close()
      
      rospy.sleep(0.5)
      i += 1

  def change_marker(self, tagMarkers):
    forward_marker = newMarker("follow", 0, 0, 1.3)
    def handle_forward_input(feedback):
      if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        self.print_programs()
      if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        if feedback.menu_entry_id == 1:
          self.create_program()
        if feedback.menu_entry_id == 2:
          self.save_program()
        if feedback.menu_entry_id >= 100:
          self.save_pose_raw(feedback.menu_entry_id - 100)
        if feedback.menu_entry_id == 4:
          self.open_gripper()
        if feedback.menu_entry_id == 5:
          self.close_gripper()
        if feedback.menu_entry_id == 6:
          self.load_program_raw()
    self.server.insert(forward_marker, handle_forward_input)

    menu_create = MenuEntry()
    menu_create.id = 1
    menu_create.parent_id = 0
    menu_create.title = "Create program (input name in CLI)"
    menu_create.command_type = MenuEntry.FEEDBACK

    menu_save_program = MenuEntry()
    menu_save_program.id = 2
    menu_save_program.parent_id = 0
    menu_save_program.title = "Save program"
    menu_save_program.command_type = MenuEntry.FEEDBACK

    menu_save_pose = MenuEntry()
    menu_save_pose.id = 3
    menu_save_pose.parent_id = 0
    menu_save_pose.title = "Save pose..."
    menu_save_pose.command_type = MenuEntry.FEEDBACK

    menu_save_pose_robot = MenuEntry()
    menu_save_pose_robot.id = 100
    menu_save_pose_robot.parent_id = 3
    menu_save_pose_robot.title = "robot"
    menu_save_pose_robot.command_type = MenuEntry.FEEDBACK

    menu_save_pose_tags = [None] * len(tagMarkers)
    self.marker_menu_names = {}
    numTags = 0
    for id in tagMarkers:
      pos = tagMarkers[id]
      self.marker_menu_names[numTags] = id
      menu_save_pose_tags[numTags] = MenuEntry()
      menu_save_pose_tags[numTags].id = 100 + numTags + 1
      menu_save_pose_tags[numTags].parent_id = 3
      menu_save_pose_tags[numTags].title = str(id)
      menu_save_pose_tags[numTags].command_type = MenuEntry.FEEDBACK
      numTags += 1

    menu_open = MenuEntry()
    menu_open.id = 4
    menu_open.parent_id = 0
    menu_open.title = "Open gripper"
    menu_open.command_type = MenuEntry.FEEDBACK
    
    menu_close = MenuEntry()
    menu_close.id = 5
    menu_close.parent_id = 0
    menu_close.title = "Close gripper"
    menu_close.command_type = MenuEntry.FEEDBACK

    load_program = MenuEntry()
    load_program.id = 6
    load_program.parent_id = 0
    load_program.title = "Load program (input name in CLI)"
    load_program.command_type = MenuEntry.FEEDBACK

    forward_marker.menu_entries.append(menu_create)
    forward_marker.menu_entries.append(menu_save_program)
    forward_marker.menu_entries.append(menu_save_pose)
    forward_marker.menu_entries.append(menu_save_pose_robot)
    forward_marker.menu_entries.append(menu_open)
    forward_marker.menu_entries.append(menu_close)
    forward_marker.menu_entries.append(load_program)
    for i in range(numTags):
      forward_marker.menu_entries.append(menu_save_pose_tags[i])

    self.server.applyChanges()

def main():
  rospy.init_node('programming_by_demonstration')

  ar_tag_reader = ArTagReader()
  sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_tag_reader.callback) # Subscribe to AR tag poses, use reader.callback

  base = fetch_api.Base()
  arm = fetch_api.Arm()
  gripper = fetch_api.Gripper()
  
  server = InteractiveMarkerServer('programming_by_demonstration')
  pbd = PbD(server, ar_tag_reader, base, arm, gripper)
  pbd.change_marker({})
  ar_tag_reader.pbd = pbd

  rospy.spin()

if __name__ == '__main__':
  main()