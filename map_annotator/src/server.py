#!/usr/bin/env python

import pickle
import rospy
import numpy as np
import sys
import logging
from map_annotator.msg import UserActions, PoseNames

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


class MapServer(object):

    def __init__(self):
        self.cnt = 0
        self.pub = rospy.Publisher("/map_annotator/pose_names", PoseNames, queue_size=10, latch=True)
        self.sub = rospy.Subscriber("/map_annotator/user_actions", UserActions, self.action_cb)
        self.int_server = InteractiveMarkerServer("/map_annotator/map_poses")
        try: 
            pickle_in = open("poses.pickle", "rb")
            self.poses = pickle.load(pickle_in)
        except IOError:
            self.poses = {}
        msg = PoseNames()
        msg.pose_names = self.get_keys()
        self.pub.publish(msg)

    def get_keys(self):
        names = []
        for key in self.poses:
            names.append(key)
        return names

    def update_poses(self):
        msg = PoseNames()
        msg.pose_names = self.get_keys()
        self.pub.publish(msg)

        pickle_out = open("poses.pickle", "wb")
        pickle.dump(self.poses, pickle_out)

    def action_cb(self, data):
        if data.command == data.CREATE:
            self.create_pose(data.name)
            self.update_poses()
            
        elif data.command == data.DELETE:
            name = data.name
            del self.poses[name]
            self.int_server.erase(name)
            self.int_server.applyChanges()
            self.update_poses()
        elif data.command == data.GOTO:
            name = data.name
            self.goto(name)

    def goto(self, name):
        pose = self.poses[name]

        # pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)
        goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)

        poseStamped = PoseStamped()
        poseStamped.header.frame_id = "map"
        poseStamped.pose = pose
        goal_pub.publish(poseStamped)

        raise Exception(pose)

    
    def add_pose(self, marker):
        # raise Exception(marker)
        if marker.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            name = marker.marker_name
            pose = marker.pose
            self.poses[name] = pose 
            self.cnt += 1
            if (self.cnt > 100):
                raise Exception(pose)


    def create_pose(self, name):

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.pose.orientation.w = 1
        int_marker.pose.position.z = 0.1
        int_marker.scale = 1

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.scale.x = 0.55
        arrow_marker.scale.y = 0.15
        arrow_marker.scale.z = 0.15
        arrow_marker.color.r = 1.0
        arrow_marker.color.a = 1.0

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.orientation.w = 1
        text_marker.pose.position.z = 0.3
        text_marker.text = name
        text_marker.color.a = 1.0
        text_marker.scale.x = 0.3
        text_marker.scale.y = 0.3
        text_marker.scale.z = 0.3

        control = InteractiveMarkerControl()
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.orientation.w = 1.0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.markers.append(arrow_marker)
        control.markers.append(text_marker)
        control.always_visible = True

        control_ring = InteractiveMarkerControl()
        control_ring.orientation.w = 1
        control_ring.orientation.x = 0
        control_ring.orientation.y = 1
        control_ring.orientation.z = 0
        control_ring.name = 'rotate_z'
        control_ring.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        # control.markers.append(text_marker)

        int_marker.controls.append(control)
        int_marker.controls.append(control_ring)

        self.poses[name] = int_marker.pose

        self.int_server.insert(int_marker, self.add_pose)
        self.int_server.applyChanges()
  nav_path = NavPath(marker_publisher)

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('map_annotator_server')
    wait_for_time()

    server = MapServer()
    rospy.spin()

if __name__ == '__main__':
    main()
