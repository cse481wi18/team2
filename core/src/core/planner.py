import rospy
import numpy as np
import math
import actionlib
import tf.transformations as tft
import core
import copy

from tf import TransformListener
from perception_msgs.msg import TennisBallPoses
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def is_too_close(p1, p2): 
    # (Point, Point) -> bool
    difference = np.array([p1.x - p2.x, p1.y - p2.y, p1.z - p2.z])
    dist = np.sqrt(difference.dot(difference))
    return dist < 0.001

class Planner:
    def __init__(self):
        # self.header_frame_id = None
        # self.header_init = False
        # self.grabber = core.Grabber()

        self.listener = TransformListener(rospy.Duration(10))

         # Point
        #self.last_poses = []
        self.robot_point = None # Point
        self.all_points = [] # [Point]
        self.ordered_object_points = [] # [Point]
        self.ordered_pickup_poses = [] #
        self.distance_offset = 0.5 # for future measurement
        self.object_pose_sub = rospy.Subscriber('recognizer/object_positions', TennisBallPoses, self.save_ball_poses_cb)
        self.ordered_pickup_pub = rospy.Publisher('/ordered_pickup', TennisBallPoses, queue_size=1)
        self.ordered_ball_pub = rospy.Publisher('/ordered_ball', TennisBallPoses, queue_size=1)
        self.unordered_pub = rospy.Publisher('/unordered_balls', TennisBallPoses, queue_size=1)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.save_robot_pose_cb)
        
        # self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
        # self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def get_pose(self):
        # (Planner) -> [?]

        print "", self.robot_point
        while self.all_points is [] or self.robot_point is None:
            return None
        points = self.all_points
        distances = [self.dist(p, self.robot_point) for p in points]
        sorted_idx = np.argsort(distances)
        self.ordered_object_points = [points[i] for i in sorted_idx]
        self.ordered_pickup_poses = [self.get_real_pose(p) for p in self.ordered_object_points]

        distances = [self.dist(p.position, self.robot_point) for p in self.ordered_pickup_poses]
        print distances

        self.ordered_pickup_pub.publish(self.ordered_pickup_poses)

        ordered_object_poses = []
        for pt in self.ordered_object_points:
            p = Pose()
            p.position = pt
            p.orientation.w = 1
            ordered_object_poses.append(p)
        self.ordered_ball_pub.publish(ordered_object_poses)
        
        # Visualize
        i = 100003
        for pose in self.ordered_pickup_poses:
            object_marker = Marker()
            object_marker.ns = "objects"
            object_marker.id = i
            object_marker.header.frame_id = "map"
            object_marker.type = Marker.CUBE
            object_marker.pose = pose
            object_marker.scale.x = 0.1
            object_marker.scale.y = 0.1
            object_marker.scale.z = 0.1
            object_marker.color.r = 1
            object_marker.color.g = 0
            object_marker.color.a = 0.5
            self.marker_pub.publish(object_marker)
            i += 1

        return {"pickup_poses": self.ordered_pickup_poses, "object_poses": ordered_object_poses}
    
    def get_real_pose(self, ball_point):
        # (Planner, Point) -> Pose
        dist = self.dist(self.robot_point, ball_point)
        rate = self.distance_offset / dist
        dx = rate * (ball_point.x - self.robot_point.x)
        dy = rate * (ball_point.y - self.robot_point.y)
        pose = Pose()
        pose.position.x = ball_point.x - dx
        pose.position.y = ball_point.y - dy
        return pose

    def save_ball_poses_cb(self, msg):
        # (Planner, TennisBallPoses) -> None
        if len(msg.poses) > 0:
            self.header_frame_id = msg.poses[0].header.frame_id
            self.listener.waitForTransform('/map', self.header_frame_id, rospy.Time(), rospy.Duration(4.0))

        new_points = map(lambda p: self.listener.transformPose("map", p).pose.position, msg.poses)

        # Merge poses
        old_points = self.all_points
        valid_old_points = []
        for p in old_points:
            too_close = False
            for q in new_points:
                if is_too_close(p, q):
                    too_close = True
                    break
            if not too_close:
                valid_old_points.append(p)
        self.all_points = valid_old_points + new_points

    def save_robot_pose_cb(self, robot_pose_msg):
        # (Planner, PoseWithCovarianceStamped) -> None
        self.robot_point = robot_pose_msg.pose.pose.position
        print "Planner: robot pose received, HAHAHAA"
    
    def dist(self, x, y):
        # (Planner, Point, Point) -> double
        dx = (x.x - y.x) ** 2
        dy = (x.y - y.y) ** 2
        return np.sqrt(dx + dy)

