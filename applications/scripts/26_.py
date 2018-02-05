#! /usr/bin/env python

import fetch_api
import rospy
import copy
import numpy as np
import tf.transformations as tft
from tf import TransformListener
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from geometry_msgs.msg import PoseStamped

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

def forward(p, dist):
    return move_pose(p, dist, 0, 0)

def up(p, dist):
    return move_pose(p, 0, 0, dist)

def move_pose(p, x, y, z):
    p = copy.deepcopy(p)
    listener = TransformListener(rospy.Duration(10))
    listener.waitForTransform('base_link', 'wrist_roll_link', rospy.Time(), rospy.Duration(4.0))
    mat = tft.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    offset = np.matmul(mat, np.array([x, y, z, 1]))
    p.position.x += offset[0]
    p.position.y += offset[1]
    p.position.z += offset[2]
    return p

def make_6dof_controls():
    controls = []

    control = InteractiveMarkerControl()
    # control.orientation_mode = InteractiveMarkerControl.FIXED

    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))

    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))

    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))

    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))

    return controls

def to_pose_stamped(pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "base_link"
    pose_stamped.header.stamp = rospy.Time(0)
    return pose_stamped

X_OFFSET = 0.166
class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._gripper_im = None
        self._open = True


    def start(self):
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "base_link"
        gripper_im.name = "gripper_teleop"
        gripper_im.description = "gripper_teleop"

        ps = PoseStamped()
        ps.header.frame_id = "wrist_roll_link"
        ps.pose.position.x = X_OFFSET
        ps.pose.position.y = 0
        ps.pose.position.z = 0
        ps.pose.orientation.w = 1

        listener = TransformListener(rospy.Duration(10))
        listener.waitForTransform('base_link', 'wrist_roll_link', rospy.Time(), rospy.Duration(4.0))
        ps_base_link = listener.transformPose("base_link", ps)
        gripper_im.pose = ps_base_link.pose

        menu_move = MenuEntry()
        menu_move.id = 1
        menu_move.parent_id = 0
        menu_move.title = "Move"
        menu_move.command_type = MenuEntry.FEEDBACK
        
        menu_open = MenuEntry()
        menu_open.id = 2
        menu_open.parent_id = 0
        menu_open.title = "Open"
        menu_open.command_type = MenuEntry.FEEDBACK
        
        menu_close = MenuEntry()
        menu_close.id = 3
        menu_close.parent_id = 0
        menu_close.title = "Close"
        menu_close.command_type = MenuEntry.FEEDBACK
    
        gripper_im.menu_entries.append(menu_move)
        gripper_im.menu_entries.append(menu_open)
        gripper_im.menu_entries.append(menu_close)

        gripper = Marker()
        gripper.type = Marker.MESH_RESOURCE
        gripper.mesh_resource = GRIPPER_MESH
        gripper.color.r = 1.0
        gripper.color.a = 1.0
        gripper.scale.x = 1.5
        gripper.scale.y = 1.5
        gripper.scale.z = 1.5

        l_finger = Marker()
        l_finger.type = Marker.MESH_RESOURCE
        l_finger.mesh_resource = L_FINGER_MESH
        l_finger.color.r = 1.0
        l_finger.color.a = 1.0
        l_finger.scale.x = 1.5
        l_finger.scale.z = 1.5

        r_finger = Marker()
        r_finger.type = Marker.MESH_RESOURCE
        r_finger.mesh_resource = R_FINGER_MESH
        r_finger.color.r = 1.0
        r_finger.color.a = 1.0
        r_finger.scale.x = 1.5
        r_finger.scale.z = 1.5

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

        gripper_control = InteractiveMarkerControl()
        gripper_control.markers.append(gripper)
        gripper_control.markers.append(l_finger)
        gripper_control.markers.append(r_finger)
        gripper_control.always_visible = True
        gripper_control.interaction_mode = InteractiveMarkerControl.BUTTON
        gripper_im.controls.append(gripper_control)

        six_dof_controls = make_6dof_controls()
        gripper_im.controls.extend(six_dof_controls) 

        self._gripper_im = gripper_im
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            if self._open:
                self._gripper.open()
            else:
                self._gripper.close()
            self._open = not self._open
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                self.handle_move()
            elif feedback.menu_entry_id == 2:
                self.handle_open()
            elif feedback.menu_entry_id == 3:
                self.handle_close()
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.check_pose(to_pose_stamped(feedback.pose))

    def check_pose(self, pose_stamped):
        if self._arm.compute_ik(pose_stamped):
            for m in self._gripper_im.controls[0].markers:
                m.color.r = 0.0
                m.color.g = 1.0
            print "good"
        else:
            for m in self._gripper_im.controls[0].markers:
                m.color.r = 1.0
                m.color.g = 0.0
            print "bad"
        self._gripper_im.pose = pose_stamped.pose
        self._im_server.insert(self._gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_open(self):
        self._open = False
        self._gripper.open()

    def handle_close(self):
        self._open = True
        self._gripper.close()

    def handle_move(self):
        print 'moving'
        p = forward(self._gripper_im.pose, -X_OFFSET)
        self._arm.move_to_pose(to_pose_stamped(p))
        pass


class FakeGripperTeleop(object):
    def __init__(self, arm, im_server, name):
        self._arm = arm
        self._im_server = im_server
        self._gripper_im = None
        self.name = name

    def start(self):
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "base_link"
        gripper_im.name = "fake_gripper_teleop_" + self.name
        gripper_im.description = self.name

        ps = PoseStamped()
        ps.header.frame_id = "wrist_roll_link"
        ps.pose.position.x = X_OFFSET
        ps.pose.position.y = 0
        ps.pose.position.z = 0
        ps.pose.orientation.w = 1

        listener = TransformListener(rospy.Duration(10))
        listener.waitForTransform('base_link', 'wrist_roll_link', rospy.Time(), rospy.Duration(4.0))
        ps_base_link = listener.transformPose("base_link", ps)
        gripper_im.pose = ps_base_link.pose

        gripper = Marker()
        gripper.type = Marker.MESH_RESOURCE
        gripper.mesh_resource = GRIPPER_MESH
        gripper.color.r = 1.0
        gripper.color.a = 1.0
        gripper.scale.x = 1.5
        gripper.scale.y = 1.5
        gripper.scale.z = 1.5

        l_finger = Marker()
        l_finger.type = Marker.MESH_RESOURCE
        l_finger.mesh_resource = L_FINGER_MESH
        l_finger.color.r = 1.0
        l_finger.color.a = 1.0
        l_finger.scale.x = 1.5
        l_finger.scale.z = 1.5

        r_finger = Marker()
        r_finger.type = Marker.MESH_RESOURCE
        r_finger.mesh_resource = R_FINGER_MESH
        r_finger.color.r = 1.0
        r_finger.color.a = 1.0
        r_finger.scale.x = 1.5
        r_finger.scale.z = 1.5

        gripper_control = InteractiveMarkerControl()
        gripper_control.markers.append(gripper)
        gripper_control.markers.append(l_finger)
        gripper_control.markers.append(r_finger)
        gripper_control.always_visible = True
        gripper_im.controls.append(gripper_control)

        self._gripper_im = gripper_im
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.check_pose(to_pose_stamped(feedback.pose))

    def check_pose(self, pose_stamped):
        pose_stamped.pose = forward(pose_stamped.pose, -X_OFFSET)
        if self._arm.compute_ik(pose_stamped):
            for m in self._gripper_im.controls[0].markers:
                m.color.r = 0.0
                m.color.g = 1.0
        else:
            for m in self._gripper_im.controls[0].markers:
                m.color.r = 1.0
                m.color.g = 0.0
        pose_stamped.pose = forward(pose_stamped.pose, X_OFFSET)
        self._gripper_im.pose = pose_stamped.pose
        self._im_server.insert(self._gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._obj_pose = None
        self._fakeGripperTeleop_pre = FakeGripperTeleop(arm, im_server, "pre")
        self._fakeGripperTeleop_pre.start()
        self._fakeGripperTeleop_grasp = FakeGripperTeleop(arm, im_server, "grasp")
        self._fakeGripperTeleop_grasp.start()
        self._fakeGripperTeleop_lift = FakeGripperTeleop(arm, im_server, "lift")
        self._fakeGripperTeleop_lift.start()

    def start(self):
        obj_im = InteractiveMarker()
        obj_im.header.frame_id = "base_link"
        obj_im.name = "object"
        obj_im.description = "object"

        menu_pick = MenuEntry()
        menu_pick.id = 1
        menu_pick.parent_id = 0
        menu_pick.title = "Pick"
        menu_pick.command_type = MenuEntry.FEEDBACK
        obj_im.menu_entries.append(menu_pick)

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.pose.orientation.w = 1
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 0.1
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        obj_control = InteractiveMarkerControl()
        obj_control.markers.append(box_marker)
        obj_control.always_visible = True
        obj_control.interaction_mode = InteractiveMarkerControl.BUTTON
        obj_im.controls.append(obj_control)

        six_dof_controls = make_6dof_controls()
        obj_im.controls.extend(six_dof_controls) 

        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)
        self._obj_pose = obj_im.pose

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                self.handle_pick()
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.handle_update_pose(feedback.pose)
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.handle_click()

    def get_pose_pre(self):
        return forward(self._obj_pose, -0.3)

    def get_pose_grasp(self):
        return forward(self._obj_pose, -0.2)

    def get_pose_lift(self):
        p = copy.deepcopy(self._obj_pose)
        p.position.z += 0.2
        return p

    def handle_update_pose(self, pose):
        self._obj_pose = pose

    def handle_click(self):
        self._fakeGripperTeleop_pre.check_pose(to_pose_stamped(forward(self.get_pose_pre(), X_OFFSET)))
        self._fakeGripperTeleop_grasp.check_pose(to_pose_stamped(forward(self.get_pose_grasp(), X_OFFSET)))
        self._fakeGripperTeleop_lift.check_pose(to_pose_stamped(forward(self.get_pose_lift(),X_OFFSET)))

    def handle_pick(self):
        pos_pre = self.get_pose_pre()
        pos_grasp = self.get_pose_grasp()
        pos_lift = self.get_pose_lift()

        self._gripper.open()

        rospy.sleep(1)
        if self._arm.compute_ik(to_pose_stamped(pos_pre)):
            err = self._arm.move_to_pose(to_pose_stamped(pos_pre))
            if err is None:
                pass
            else:
                raise "Failed to reach pre-grasp-pose"
        else:
            raise "Can't reach pre-grasp-pose" 

        rospy.sleep(1)
        if self._arm.compute_ik(to_pose_stamped(pos_grasp)):
            err = self._arm.move_to_pose(to_pose_stamped(pos_grasp))
            if err is None:
                pass
            else:
                raise "Failed to reach grasp-pose"
        else:
            raise "Can't reach grasp-pose" 
            
        rospy.sleep(1)
        self._gripper.close()

        rospy.sleep(1)
        if self._arm.compute_ik(to_pose_stamped(pos_lift)):
            err = self._arm.move_to_pose(to_pose_stamped(pos_lift))
            if err is None:
                pass
            else:
                raise "Failed to reach lift-pose"
        else:
            raise "Can't reach lift-pose" 


def main():
    rospy.init_node("advanced")

    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    im_server.applyChanges()
    auto_pick_im_server.applyChanges()
    rospy.spin()

if __name__ == '__main__':
  main()