#!/usr/bin/env python

import pickle
import rospy
import numpy as np
import sys
from fetch_api import base.py
from map_annotator.srv import UserActions, MapPoses, PoseNames

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback

class MapServer:

    def __init__(self):
        self.pub = rospy.Publisher("/map_annotator/pose_names", PoseNames, queue_size=10, latch=True)
        self.sub = rospy.Subscriber("/map_annotator/user_actions", UserActions, action_cb)
        self.int_server = InteractiveMarkerServer("/map_annotator/map_poses")
        self.poses = {}

    def get_keys(self):
        names = []
        for key, value in self.poses:
            names.append(key)
        return names

    def action_cb(data):
        if data.command == data.CREATE:
            self.create_pose(data.name)
            names = self.get_keys()
            msg = PoseNames()
            msg.names = names
            self.pub.publish(msg)
        else if data.command == data.DELETE:
            name = data.name
            del self.poses[name]
            self.int_server.erase(name)
            self.int_server.applyChanges()
            names = self.get_keys()
            msg = PoseNames()
            msg.names = names
            self.pub.publish(msg)
        else if data.command == data.GOTO:
            name = data.name
            self.goto(name)

    def goto(self, name):
        pose = self.poses[name]
        # pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)
        goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
        goal_pub.publish(pose)

    
    def add_pose(marker):
        if marker.event_type = InteractiveMarkerFeedback.POSE_UPDATE:
            name = marker.name
            pose = marker.pose
            self.poses[name] = pose 


    def create_pose(self, name):

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = name
        int_marker.pose.orientation.w = 1
        int_marker.scale = 0.45

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.scale.x = 0.45
        arrow_marker.scale.y = 0.45
        arrow_marker.scale.z = 0.45
        arrow_marker.color.r = 1.0
        arrow_marker.color.a = 1.0

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.orientation.w = 1
        text_marker.text = name

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.interaction_mode = MOVE_ROTATE
        control.markers.append(arrow_marker)
        control.always_visible = True

        control.markers.append(text_marker)

        int_marker.controls.append(control)

        self.int_server.insert(int_marker, add_pose)
        self.int_server.applyChanges()

def main():
    rospy.init_node('map_annotator_server')

    server = MapServer()
    rospy.spin()