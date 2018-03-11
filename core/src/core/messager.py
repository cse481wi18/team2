import rospy
import actionlib
import math
import tf.transformations as tft
import numpy as np
import copy
import fetch_api
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from perception_msgs.msg import Pose2d, Poses2d
from std_msgs.msg import String, Float64

class Messager:
  def __init__(self):
    self.status1_message_pub = rospy.Publisher('core/messages/status1', String, queue_size=1)
    self.status2_message_pub = rospy.Publisher('core/messages/status2', String, queue_size=1)

    # Ball poses below
    self.scan_poses_message_pub = rospy.Publisher('/core/messages/scan_poses', Poses2d, queue_size=1)
    self.confirm_poses_message_pub = rospy.Publisher('/core/messages/confirm_poses', Poses2d, queue_size=1)
    self.observe_poses_message_pub = rospy.Publisher('/core/messages/observe_poses', Poses2d, queue_size=1)

    self.anchor_poses_message_pub = rospy.Publisher('/core/messages/anchor_poses', Poses2d, queue_size=1)
    self.look_at_poses_message_pub = rospy.Publisher('/core/messages/look_at_poses', Poses2d, queue_size=1)
    self.robot_pose_message_pub = rospy.Publisher('/core/messages/robot_pose', Pose2d, queue_size=1)

    self.robot_orientation_message_pub = rospy.Publisher('/core/messages/robot_orientation', Float64, queue_size=1)

    self.listener = TransformListener()

  def publish_status1(self, status):
    s = String()
    s.data = status
    self.status1_message_pub.publish(s)

  def publish_status2(self, status):
    s = String()
    s.data = status
    self.status2_message_pub.publish(s)

  def publish_scan_poses(self, points):
    # ([Pose]) => None
    # frame_id = map
    ps2d = point_arr_to_poses2d(points)
    self.scan_poses_message_pub.publish(ps2d)

  def publish_confirm_poses(self, points):
    # ([Pose]) => None
    # frame_id = map
    ps2d = point_arr_to_poses2d(points)
    self.confirm_poses_message_pub.publish(ps2d)

  def publish_observe_poses(self, points):
    # ([Pose]) => None
    # frame_id = map
    self.listener.waitForTransform('/map', '/odom', rospy.Time(), rospy.Duration(4.0))
    res = []
    for pt in points:
      object_poseStamped = PoseStamped()
      object_poseStamped.header.frame_id = "odom"
      object_poseStamped.pose.position = pt
      object_poseStamped.pose.orientation.w = 1
      map_point = self.listener.transformPose('/map', object_poseStamped).pose.position
      res.append(map_point)

    ps2d = point_arr_to_poses2d(res)
    self.observe_poses_message_pub.publish(ps2d)

  def publish_anchor_poses(self, points):
    # ([Pose]) => None
    # frame_id = map
    ps2d = point_arr_to_poses2d(points)
    self.anchor_poses_message_pub.publish(ps2d)

  def publish_look_at_pose(self, point):
    ps2d = []
    if point is not None:
      ps2d = point_arr_to_poses2d([point])
    self.look_at_poses_message_pub.publish(ps2d)

  def publish_robot_pose(self, point):
    p2d = point_to_pose2d(point)
    self.robot_pose_message_pub.publish(p2d)

  def publish_robot_orientation(self, yaw):
    self.robot_orientation_message_pub.publish(yaw)

def point_arr_to_poses2d(points):
  ps2d = Poses2d()
  ps2d.poses = map(lambda p: point_to_pose2d(p), points)
  return ps2d

def point_to_pose2d(point):
  p2d = Pose2d()
  p2d.x = point.x
  p2d.y = point.y
  return p2d