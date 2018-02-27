import fetch_api
import rospy
import copy
import numpy as np
import tf.transformations as tft
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker

def to_pose_stamped(pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "base_link"
    pose_stamped.header.stamp = rospy.Time(0)
    return pose_stamped

def forward(p, dist):
    return move_pose(p, dist, 0, 0)

def move_pose(p, x, y, z):
    p = copy.deepcopy(p)
    listener = TransformListener(rospy.Duration(10))
    listener.waitForTransform('base_link', 'wrist_roll_link', rospy.Time(), rospy.Duration(4.0))
    mat = tft.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    offset = mat.dot(np.array([x, y, z, 1]))
    p.position.x += offset[0]
    p.position.y += offset[1]
    p.position.z += offset[2]
    return p

class Grabber:
    def __init__(self):
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)

    def move(self, pose):
        pos_pre = to_pose_stamped(self.get_pose_pre(pose))
        pos_grasp = to_pose_stamped(self.get_pose_grasp(pose))
        pos_lift = to_pose_stamped(self.get_pose_lift(pose))
        pos_unload = to_pose_stamped(self.get_pose_unload(pose))

        # Visualize
        i = 12000
        for pose in [pos_pre, pos_grasp, pos_lift, pos_unload]:
            object_marker = Marker()
            object_marker.ns = "objects"
            object_marker.id = i
            object_marker.header.frame_id = "base_link"
            object_marker.type = Marker.CUBE
            object_marker.pose = pose.pose
            object_marker.scale.x = 0.2
            object_marker.scale.y = 0.05
            object_marker.scale.z = 0.05
            object_marker.color.r = 0.3 + 0.2 * i
            object_marker.color.g = 0
            object_marker.color.b = 0.3 + 0.2 * i
            object_marker.color.a = 0.3 + 0.2 * i
            self.marker_pub.publish(object_marker)
            i += 1
        
        self._gripper.open()

        if self.check_pose(pos_pre) is True:
            err = self._arm.move_to_pose(pos_pre)
            if err is None:
                print "move to pre-grasp-pose"
            else:
                print "Failed to reach pre-grasp-pose"
        else:
            print "Can't reach pre-grasp-pose"

        rospy.sleep(1)
        
        if self.check_pose(pos_grasp) is True:
            err = self._arm.move_to_pose(pos_grasp)
            if err is None:
                print "move to grasp-pose"
            else:
                print "Failed to reach grasp-pose"
        else:
            print "Can't reach grasp-pose"

        rospy.sleep(1)
        self._gripper.close()

        rospy.sleep(1)
        if self.check_pose(pos_lift) is True:
            err = self._arm.move_to_pose(pos_lift)
            if err is None:
                print "move to lift-pose"
            else:
                print "Failed to reach lift-pose"
        else:
            print "Can't reach lift-pose"

        rospy.sleep(1)
        if self.check_pose(pos_unload) is True:
            err = self._arm.move_to_pose(pos_unload)
            if err is None:
                print "move to unload-pose"
                pass
            else:
                print "Failed to reach unload-pose"
        else:
            print "Can't reach unload-pose"
        self._gripper.open()
        return True

    def check_pose(self, pose_stamped):
        return self._arm.compute_ik(pose_stamped)

    def get_pose_pre(self, pose):
        p = copy.deepcopy(pose)
        p = forward(p, 0.02)
        p.orientation.x = 0.0
        p.orientation.y = 1.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        return forward(p, -0.4)

    def get_pose_grasp(self, pose):
        p = copy.deepcopy(pose)
        p = forward(p, 0.02)
        p.orientation.x = 0.0
        p.orientation.y = 1.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        return forward(p, -0.19)

    def get_pose_lift(self, pose):
        return self.get_pose_pre(pose)
        # p = copy.deepcopy(pose)
        # p.position.z += 0.2
        # return p

    def get_pose_unload(self, pose):
        p = Pose()
        p.position.x = -0.34511
        p.position.y = 0.22313
        p.position.z = 0.98066
        p.orientation.x = -0.36003
        p.orientation.y = 0.72444
        p.orientation.z = 0.46414
        p.orientation.w = -0.36075
        return p