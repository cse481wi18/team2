import rospy
import numpy as np
import math
from perception_msgs.msg import TennisBallPoses
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped, Point
from visualization_msgs.msg import Marker

def is_too_close(p1, p2): 
    # (Point, Point) -> bool
    difference = np.array([p1.x - p2.x, p1.y - p2.y, p1.z - p2.z])
    dist = np.sqrt(difference.dot(difference))
    return dist < 0.001

class Planner:
    def __init__(self):
        self.robot_pose = None # Point
        #self.last_poses = []
        self.all_poses = [] # [Point]
        self.distance_offset = 0.1 # for future measurement
        self.object_pose_sub = rospy.Subscriber('recognizer/object_positions', TennisBallPoses, self.save_ball_poses_cb)
        self.ordered_pub = rospy.Publisher('/ordered_balls', TennisBallPoses, queue_size=1)
        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.save_robot_pose_cb)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        
    
    def get_pose(self):
        # (Planner) -> [?]
        print "", self.robot_pose
        while self.all_poses is [] or self.robot_pose is None:
            return None
        poses = self.all_poses
        distances = [self.dist(p, self.robot_pose) for p in poses]
        sorted_idx = np.argsort(distances)
        ordered_poses = [poses[i] for i in sorted_idx]
        ordered_poses = [self.get_real_pose(p) for p in ordered_poses]

        distances = [self.dist(p.position, self.robot_pose) for p in ordered_poses]
        print distances

        self.ordered_pub.publish(ordered_poses)

        # Visualize
        i = 100003
        for pose in ordered_poses:
            object_marker = Marker()
            object_marker.ns = "objects"
            object_marker.id = i
            object_marker.header.frame_id = "base_link"
            object_marker.type = Marker.CUBE
            object_marker.pose = pose
            object_marker.scale.x = 0.1
            object_marker.scale.y = 0.1
            object_marker.scale.z = 0.1
            object_marker.color.r = 1
            object_marker.color.g = 1
            object_marker.color.a = 0.3
            self.marker_pub.publish(object_marker)
            i += 1

        return ordered_poses
    
    def get_real_pose(self, ball_pose):
        # (Planner, Point) -> Pose
        dist = self.dist(self.robot_pose, ball_pose)
        rate = self.distance_offset / dist
        dx = rate * (ball_pose.x - self.robot_pose.x)
        dy = rate * (ball_pose.y - self.robot_pose.y)
        pose = Pose()
        pose.position.x = ball_pose.x - dx
        pose.position.y = ball_pose.y - dy
        return pose

    def save_ball_poses_cb(self, msg):
        # (Planner, TennisBallPoses) -> None
        new_poses = map(lambda p: p.position, msg.poses)
        # Merge poses
        old_poses = self.all_poses
        valid_old_poses = []
        for p in old_poses:
            too_close = False
            for q in new_poses:
                if is_too_close(p, q):
                    too_close = True
                    break
            if not too_close:
                valid_old_poses.append(p)
        self.all_poses = valid_old_poses + new_poses

    def save_robot_pose_cb(self, robot_pose_msg):
        # (Planner, PoseWithCovarianceStamped) -> None
        self.robot_pose = robot_pose_msg.pose.pose.position
    
    def dist(self, x, y):
        # (Planner, Point, Point) -> double
        dx = (x.x - y.x) ** 2
        dy = (x.y - y.y) ** 2
        return np.sqrt(dx + dy)
