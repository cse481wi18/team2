#! /usr/bin/env python

import rospy
import fetch_api

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

def newMarker(name, x, y):
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = "base_link"
  int_marker.name = name
  int_marker.description = ""
  int_marker.pose.position.x = x
  int_marker.pose.position.y = y
  int_marker.pose.orientation.w = 1

  box_marker = Marker()
  box_marker.type = Marker.CUBE
  box_marker.pose.orientation.w = 1
  box_marker.scale.x = 0.45
  box_marker.scale.y = 0.45
  box_marker.scale.z = 0.45
  box_marker.color.r = 0.0
  box_marker.color.g = 0.5
  box_marker.color.b = 0.5
  box_marker.color.a = 1.0

  button_control = InteractiveMarkerControl()
  button_control.interaction_mode = InteractiveMarkerControl.BUTTON
  button_control.always_visible = True
  button_control.markers.append(box_marker)
  int_marker.controls.append(button_control)
  return int_marker

def main():
  base = fetch_api.Base()
  #base.turn(value) 

  print "hello"
  
  rospy.init_node('interactive_node')

  server = InteractiveMarkerServer("simple_marker")

  forward_marker = newMarker("forward", 0.8, 0)
  def handle_forward_input(marker):
    if marker.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
      base.go_forward(0.8, 1)
  server.insert(forward_marker, handle_forward_input)

  left_marker = newMarker("left", 0, 0.8)
  def handle_left_input(marker):
    if marker.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
      base.turn(30, 1)
  server.insert(left_marker, handle_left_input)

  right_marker = newMarker("right", 0, -0.8)
  def handle_right_input(marker):
    if marker.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
      base.turn(-30, 1)
  server.insert(right_marker, handle_right_input)

  server.applyChanges()
  
  rospy.spin()

if __name__ == '__main__':
  main()
