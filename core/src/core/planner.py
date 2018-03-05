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
    return dist < 0.1


def dist_between(x, y):
    # (Planner, Point, Point) -> double
    dx = (x.x - y.x) ** 2
    dy = (x.y - y.y) ** 2
    return np.sqrt(dx + dy)

class Planner:
    def __init__(self):
        # self.header_frame_id = None
        # self.header_init = False
        # self.grabber = core.Grabber()
        self.confidence_drop_rate = 0.9
        self.paused = True

        self.listener = TransformListener(rospy.Duration(10))

         # Point
        #self.last_poses = []
        self.robot_point = None # Point
        # self.all_points = [] # [Point]
        self.all_points_confidences = {} # {Point: double}
        # self.ordered_object_points = [] # [Point]
        # self.ordered_pickup_poses = [] #
        self.object_pose_sub = rospy.Subscriber('recognizer/object_positions', TennisBallPoses, self.save_ball_poses_cb)
        # self.ordered_pickup_pub = rospy.Publisher('/ordered_pickup', TennisBallPoses, queue_size=1)
        self.ordered_ball_pub = rospy.Publisher('/ordered_ball', TennisBallPoses, queue_size=1)
        self.unordered_pub = rospy.Publisher('/unordered_balls', TennisBallPoses, queue_size=1)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.save_robot_pose_cb)
        
        # self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
        # self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def pause(self):
        self.paused = True

    def unpause(self):
        self.paused = False

    def get_high_confidence_points(self):
        return list(filter(lambda pt: self.all_points_confidences[pt] > 1.0, self.all_points_confidences.keys()))

    def get_pose(self):
        # return []

        # (Planner) -> [?]
        while self.all_points_confidences.keys() is [] or self.robot_point is None:
            return None
        points = self.get_high_confidence_points()
        print "Planner:", len(points), "high-confidence points"

        distances = [dist_between(p, self.robot_point) for p in points]
        sorted_idx = np.argsort(distances)
        ordered_object_points = [points[i] for i in sorted_idx]
        # self.ordered_pickup_poses = [self.get_pickup_pose(p) for p in self.ordered_object_points]
        # distances = [self.dist(p.position, self.robot_point) for p in self.ordered_pickup_poses]

        # self.ordered_pickup_pub.publish(self.ordered_pickup_poses)

        ordered_object_poses = []
        for pt in ordered_object_points:
            p = Pose()
            p.position = pt
            p.orientation.w = 1
            ordered_object_poses.append(p)
        self.ordered_ball_pub.publish(ordered_object_poses)
        
        # Visualize
        i = 100003
        j = 12346
        for pose in ordered_object_poses:
            object_marker = Marker()
            object_marker.ns = "objects"
            object_marker.id = i
            object_marker.header.frame_id = "map"
            object_marker.type = Marker.SPHERE
            object_marker.pose = copy.deepcopy(pose)
            object_marker.scale.x = 0.08
            object_marker.scale.y = 0.08
            object_marker.scale.z = 0.08
            object_marker.color.r = 1
            object_marker.color.a = 0.5
            self.marker_pub.publish(object_marker)

            name_marker = Marker()
            name_marker.ns = "confidence"
            name_marker.id = i + j
            name_marker.header.frame_id = "map"
            name_marker.type = Marker.TEXT_VIEW_FACING
            name_marker.pose = copy.deepcopy(pose)
            name_marker.pose.position.z += 0.1
            name_marker.pose.orientation.w = 1
            name_marker.scale.x = 0.025
            name_marker.scale.y = 0.025
            name_marker.scale.z = 0.025
            name_marker.color.r = 0
            name_marker.color.g = 0
            name_marker.color.b = 1.0
            name_marker.color.a = 1.0
            name_marker.text = "" + str(self.all_points_confidences[pose.position])
            self.marker_pub.publish(name_marker)
            i += 1

        return ordered_object_poses

    def reduce_confidence(self, pose):
        print "Planner: reducing confidence"
        pts = []
        for pt in self.get_high_confidence_points():
            if is_too_close(pt, pose.position):
                pts.append(pt)
        for pt in pts:
            del self.all_points_confidences[pt]
        print "Planner: reduce confidence result -", len(pts), "out of", len(self.get_high_confidence_points()), "points removed"

    def save_ball_poses_cb(self, msg):
        # (Planner, TennisBallPoses) -> None

        if self.paused:
            return

        if len(msg.poses) > 0:
            self.header_frame_id = msg.poses[0].header.frame_id
            # self.last_time_stamp = msg.poses[0].header.stamp
            self.listener.waitForTransform('/map', self.header_frame_id, rospy.Time(), rospy.Duration(4.0))

        new_points = map(lambda p: self.listener.transformPose("map", p).pose.position, msg.poses)

        # Merge poses
        old_points = self.all_points_confidences.keys()
        valid_old_points = []
        new_confidences = {}
        for p in old_points + new_points:
            new_confidences[p] = 0.0
        for p in old_points:
            too_close = False
            for q in new_points:
                if is_too_close(p, q):
                    too_close = True
                    new_confidences[q] += self.all_points_confidences[p]
                    break
            if not too_close:
                new_confidences[p] += self.all_points_confidences[p] * self.confidence_drop_rate
                # valid_old_points.append(p)
        # self.all_points = valid_old_points + new_points
        for p in new_points: 
            new_confidences[p] += 1.0
        self.all_points_confidences = new_confidences

    def save_robot_pose_cb(self, robot_pose_msg):
        # (Planner, PoseWithCovarianceStamped) -> None
        self.robot_point = robot_pose_msg.pose.pose.position
        self.get_pose()
    

