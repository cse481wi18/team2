import fetch_api
import rospy
import copy
import numpy as np
import tf.transformations as tft
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker
from perception_msgs.msg import TennisBallPoses

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
    # All poses in Grabber are of frame "/base_link"

    def __init__(self, planner, finder):
        self._arm = fetch_api.Arm()
        self.head = fetch_api.Head()
        self._gripper = fetch_api.Gripper()
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.listener = TransformListener(rospy.Duration(10))
        self.planner = planner
        self.finder = finder
        self.grasp_pose_offset = 0.00

    def grab(self, pose):
        # (Planner, Pose) -> bool

        # TEMP CODE
        tmp = copy.deepcopy(self.planner.all_points_confidences)
        self.planner.all_points_confidences = {}

        # print "Planned pose:", pose
        self.finder.observe_pose(pose, "base_link")
        
        res = self.planner.get_pose()
        if len(res) == 0:
            print "Grabber: planner did not return any pose"
            self.planner.all_points_confidences = tmp
            return
        actual_pose = res[0]

        self.planner.all_points_confidences = tmp
        # END OF TEMP CODE

        # print "Adjusted pose:", pose

        # Change frame
        object_poseStamped = PoseStamped()
        object_poseStamped.header.frame_id = "map"
        object_poseStamped.pose = actual_pose
        listener = TransformListener()
        listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
        base_link_pose = listener.transformPose('/base_link', object_poseStamped).pose

        self.grasp_pose_offset = 0.00
        while self.check_pose(to_pose_stamped(self.get_pose_grasp(base_link_pose))) is False and self.grasp_pose_offset < 0.02:
            self.grasp_pose_offset += 0.01

        print "Grabber: grasp pose offset =", self.grasp_pose_offset

        pos_pre = to_pose_stamped(self.get_pose_pre(base_link_pose))
        pos_grasp = to_pose_stamped(self.get_pose_grasp(base_link_pose))
        pos_lift = to_pose_stamped(self.get_pose_lift(base_link_pose))
        pos_pre_unload = to_pose_stamped(self.get_pose_pre_unload(base_link_pose))
        pos_unload = to_pose_stamped(self.get_pose_unload(base_link_pose))

        # Visualize
        i = 12000
        for ps in [pos_pre, pos_grasp, pos_pre_unload, pos_unload]:
            object_marker = Marker()
            object_marker.ns = "objects"
            object_marker.id = i
            object_marker.header.frame_id = "base_link"
            object_marker.type = Marker.CUBE
            object_marker.pose = ps.pose
            object_marker.scale.x = 0.2
            object_marker.scale.y = 0.05
            object_marker.scale.z = 0.05
            object_marker.color.r = 0.3 + 0.2 * i
            object_marker.color.g = 0
            object_marker.color.b = 0.3 + 0.2 * i
            object_marker.color.a = 0.5
            self.marker_pub.publish(object_marker)
            i += 1

        if self.check_pose(pos_pre) is False:
            print "Grabber: can't reach pre-pose"
            return False
        elif self.check_pose(pos_grasp) is False:
            print "Grabber: can't reach grasp-pose"
            return False
        elif self.check_pose(pos_pre_unload) is False:
            print "Grabber: can't reach pre-unload-pose"
            return False
        elif self.check_pose(pos_unload) is False:
            print "Grabber: can't reach unload-pose"
            return False
        else:
            if self.check_pose(pos_pre_unload) is True:
                err = self._arm.move_to_pose(pos_pre_unload)
                if err is None:
                    print "move to pre-unload-pose(before grasp)"
                    pass
                else:
                    print "Failed to reach pre-unload-pose(before grasp)"
                    return False
            else:
                print "Can't reach pre-unload-pose(before grasp)"
                return False

            self._gripper.open()
            rospy.sleep(0.1)

            if self.check_pose(pos_pre) is True:
                err = self._arm.move_to_pose(pos_pre)
                if err is None:
                    print "move to pre-grasp-pose"
                else:
                    print "Failed to reach pre-grasp-pose"
                    return False
            else:
                print "Can't reach pre-grasp-pose"
                return False

            rospy.sleep(0.1)

            if self.check_pose(pos_grasp) is True:
                err = self._arm.move_to_pose(pos_grasp)
                if err is None:
                    print "move to grasp-pose"
                else:
                    print "Failed to reach grasp-pose"
                    return False
            else:
                print "Can't reach grasp-pose"
                return False

            rospy.sleep(0.1)
            self._gripper.close()

            rospy.sleep(0.1)
            if self.check_pose(pos_lift) is True:
                err = self._arm.move_to_pose(pos_lift)
                if err is None:
                    print "move to lift-pose"
                else:
                    print "Failed to reach lift-pose"
                    return False
            else:
                print "Can't reach lift-pose"
                return False

            rospy.sleep(0.1)
            if self.check_pose(pos_pre_unload) is True:
                err = self._arm.move_to_pose(pos_pre_unload)
                if err is None:
                    print "move to pre-unload-pose"
                    pass
                else:
                    print "Failed to reach pre-unload-pose"
            else:
                print "Can't reach pre-unload-pose"
                return False

            rospy.sleep(0.1)
            if self.check_pose(pos_unload) is True:
                err = self._arm.move_to_pose(pos_unload)
                if err is None:
                    print "move to unload-pose"
                    pass
                else:
                    print "Failed to reach unload-pose"
                    return False
            else:
                print "Can't reach unload-pose"
                return False
            self._gripper.open()
            return True

    def check_pose(self, pose_stamped):
        return self._arm.compute_ik(pose_stamped)

    def get_pose_pre(self, pose):
        p = copy.deepcopy(pose)
        # p = forward(p, 0.20)
        p.orientation.x = 0.0
        p.orientation.y = 1.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        return forward(p, -0.40)

    def get_pose_grasp(self, pose):
        p = copy.deepcopy(pose)
        # p = forward(p, 0.20)
        p.orientation.x = 0.0
        p.orientation.y = 1.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        return forward(p, -0.19 - self.grasp_pose_offset)

    def get_pose_lift(self, pose):
        return self.get_pose_pre(pose)
        # p = copy.deepcopy(pose)
        # p.position.z += 0.2
        # return p

    def get_pose_pre_unload(self, pose):
        p = Pose()
        p.position.x = 0.37702
        p.position.y = 0.22687
        p.position.z = 1.14154
        p.orientation.x = 0.24932
        p.orientation.y = 0.46137
        p.orientation.z = 0.84951
        p.orientation.w = 0.05758
        return p

    def get_pose_unload(self, pose):
        p = Pose()
        p.position.x = -0.29742
        p.position.y = 0.18528
        p.position.z = 0.90081
        p.orientation.x = -0.27022
        p.orientation.y = 0.30241
        p.orientation.z = 0.83477
        p.orientation.w = -0.37242
        return p    

    def goto_pose_unload(self):
        pos_unload = to_pose_stamped(self.get_pose_unload(None))
        self._arm.move_to_pose(pos_unload)