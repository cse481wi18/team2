import rospy
import actionlib
import math
import tf.transformations as tft
import numpy as np
import copy
import fetch_api
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker

def quaternion_between(p1, p2):
    # (Point, Point) -> Quaternion
    angle = math.atan2(p2.y - p1.y, p2.x - p1.x)
    arr = tft.quaternion_from_euler(0, 0, angle)
    return ndarray_to_quaternion(arr)

def ndarray_to_quaternion(arr):
    q = Quaternion()
    q.x = arr[0]
    q.y = arr[1]
    q.z = arr[2]
    q.w = arr[3]
    return q

def dist_between(x, y):
    # (Point, Point) -> double
    dx = (x.x - y.x) ** 2
    dy = (x.y - y.y) ** 2
    return np.sqrt(dx + dy)

class Mover:
    def __init__(self):
        self.robot_point = None # Point
        self.robot_quaternion = None # Quaternion
        self.base = fetch_api.Base()
        # self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.move_base_client = rospy.Publisher("/move_base_simple/goal", PoseStamped)
        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.save_robot_pose_cb)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.grab_distance_offset = 0.55 # for future measurement

    def save_robot_pose_cb(self, robot_pose_msg):
        # (Planner, PoseWithCovarianceStamped) -> None
        self.robot_point = robot_pose_msg.pose.pose.position
        self.robot_quaternion = robot_pose_msg.pose.pose.orientation

    # def goto_pose(self, pose):
    #     if pose is None:
    #         rospy.logerr('No pose')
    #     else:
    #         goal = MoveBaseGoal()
    #         goal.target_pose.header.frame_id = 'map'
    #         goal.target_pose.header.stamp = rospy.Time().now()
    #         goal.target_pose.pose = pose
    #         self._move_base_client.send_goal(goal)

    #         return self._move_base_client.wait_for_result(rospy.Duration(20.0))

    def move_to_grab_pose(self, ball_pose):     
        while self.robot_point is None:
            print "Mover: Robot pose not received, sleeping"
            rospy.sleep(1)

        pickup_pose = self.get_pickup_pose(ball_pose.position)
        print "Mover: Going to grasp pose"
        pickup_pose.orientation = quaternion_between(self.robot_point, ball_pose.position)

        poseStamped = PoseStamped()
        poseStamped.header.frame_id = "map"
        poseStamped.pose = pickup_pose

        goal = MoveBaseGoal()
        goal.target_pose = poseStamped

        object_marker = Marker()
        object_marker.ns = "objects"
        object_marker.id = 83456213
        object_marker.header.frame_id = "map"
        object_marker.type = Marker.SPHERE
        object_marker.pose = pickup_pose
        object_marker.scale.x = 0.3
        object_marker.scale.y = 0.3
        object_marker.scale.z = 0.05
        object_marker.color.r = 1
        object_marker.color.g = 1
        object_marker.color.b = 0
        object_marker.color.a = 0.3
        self.marker_pub.publish(object_marker)

        d = dist_between(self.robot_point, pickup_pose.position)
        self.base.go_forward(d)

        # self.move_base_client.publish(poseStamped)
        # self.move_base_client.wait_for_result(rospy.Duration(20.0))
        # self.move_base_client.send_goal(goal)
        # res = self.move_base_client.wait_for_result(rospy.Duration(20.0))
        # self.move_base_client.cancel_all_goals()
        # print "Mover: returns", res

        # return poseStamped
        #  = copy.deepcopy(poseStamped)
        # object_poseStamped.pose.position = self.ordered_object_points[0]

        # self.listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
        # base_link_pose = self.listener.transformPose('/base_link', object_poseStamped).pose
        # self.grabber.move(base_link_pose)
        # print "base_link_pose:", base_link_pose

    def face_pose(self, ball_pose):
        while self.robot_point is None:
            print "Mover: Robot pose not received, sleeping"
            rospy.sleep(1)

        pickup_pose = Pose()
        pickup_pose.position = copy.deepcopy(self.robot_point)
        print "Mover: facing towards ball"
        pickup_pose.orientation = quaternion_between(self.robot_point, ball_pose.position)

        poseStamped = PoseStamped()
        poseStamped.header.frame_id = "map"
        poseStamped.pose = pickup_pose

        goal = MoveBaseGoal()
        goal.target_pose = poseStamped

        object_marker = Marker()
        object_marker.ns = "objects"
        object_marker.id = 83456214
        object_marker.header.frame_id = "map"
        object_marker.type = Marker.SPHERE
        object_marker.pose = pickup_pose
        object_marker.scale.x = 0.3
        object_marker.scale.y = 0.3
        object_marker.scale.z = 0.05
        object_marker.color.r = 1
        object_marker.color.g = 1
        object_marker.color.b = 0
        object_marker.color.a = 0.3
        self.marker_pub.publish(object_marker)

        object_marker2 = copy.deepcopy(object_marker)
        object_marker2.pose.position = ball_pose.position
        self.marker_pub.publish(object_marker2)

        print "Mover: face_pose - debug 1"
        q1 = self.robot_quaternion
        q2 = pickup_pose.orientation
        print "Mover: face_pose - debug 2"
        qm1 = tft.quaternion_matrix([q1.x, q1.y, q1.z, q1.w])
        qm2 = tft.quaternion_matrix([q2.x, q2.y, q2.z, q2.w])
        print "Mover: face_pose - debug 3"
        theta1 = math.atan2(qm1[1,0], qm1[0,0])
        theta2 = math.atan2(qm2[1,0], qm2[0,0])
        print "Mover: face_pose - debug 4"
        difference = (theta1 - theta2) % (2 * math.pi)
        difference_deg = difference / math.pi * 180
        print "Mover: face_pose - debug 5"
        self.base.turn(-difference_deg)

        # self.move_base_client.send_goal(goal)
        # res = self.move_base_client.wait_for_result(rospy.Duration(20.0))
        # self.move_base_client.cancel_all_goals()

    def get_pickup_pose(self, ball_point):
        # (Planner, Point) -> Pose
        dist = dist_between(self.robot_point, ball_point)
        rate = self.grab_distance_offset / dist
        dx = rate * (ball_point.x - self.robot_point.x)
        dy = rate * (ball_point.y - self.robot_point.y)
        pose = Pose()
        pose.position.x = ball_point.x - dx
        pose.position.y = ball_point.y - dy
        return pose
