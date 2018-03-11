import rospy
import fetch_api
import actionlib
import copy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Finder:
    # All poses in Finder are of frame "/map", unless otherwise specified
    OBSERVE_TIME_SECS = 3.0
    FIND_TIME_SECS = 3.0
    FIND_TURN_STEP_DEGREES = 60.0
    FIND_HEAD_PANS = [0.5, 0.0, -0.5]
    FIND_HEAD_TILTS = [0.5, 1.0, 1.5]
    FIND_CONFIDENCE_DROP_RATE = 1.0
    OBSERVE_POSE_CONFIDENCE_DROP_RATE = 0.8

    MAP_MOVE_DISTANCE = 1.6

    def __init__(self, messager, planner):
      self.head = fetch_api.Head()
      self.base = fetch_api.Base()
      self.planner = planner
      self.messager = messager
      self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)

      self.robot_point = None # Point
      self.robot_quarternion = None # Quaternion
      self.all_map_points = []
      self.cur_map_point_index = 0

      # Temporary
      self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
      self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.save_robot_pose_cb)
      self.collect_map_points()

    # collect all balls along negative y axis first, 
    # then collect all balls along positive y axis
    # scan from left-> right, right->left in a Z-letter pattern
    def collect_map_points(self):
      while self.robot_point is None:
        rospy.sleep(0.3)
      print "Finder: collect_map_points - robot point received"
      pos1 = copy.deepcopy(self.robot_point)
      z = self.robot_point.z
      pos2 = Point(pos1.x, pos1.y + Finder.MAP_MOVE_DISTANCE, z)
      pos3 = Point(pos2.x - Finder.MAP_MOVE_DISTANCE, pos2.y, z)
      pos4 = Point(pos3.x, pos3.y - Finder.MAP_MOVE_DISTANCE, z)
      pos5 = Point(pos4.x - Finder.MAP_MOVE_DISTANCE, pos4.y, z)
      pos6 = Point(pos5.x, pos5.y + Finder.MAP_MOVE_DISTANCE, z)
      # That's all for negative y axis, collect them.
      self.all_map_points.append(pos1)
      self.all_map_points.append(pos2)
      # self.all_map_points.append(pos3)
      # self.all_map_points.append(pos4)
      # self.all_map_points.append(pos5)
      # self.all_map_points.append(pos6)

      pos7 = Point(pos1.x + Finder.MAP_MOVE_DISTANCE, pos1.y, z)
      pos8 = Point(pos7.x, pos7.y + Finder.MAP_MOVE_DISTANCE, z)
      pos9 = Point(pos8.x + Finder.MAP_MOVE_DISTANCE, pos8.y, z)
      pos10 = Point(pos9.x, pos9.y - Finder.MAP_MOVE_DISTANCE, z)
      pos11 = Point(pos10.x + Finder.MAP_MOVE_DISTANCE, pos10.y, z)
      pos12 = Point(pos11.x, pos11.y + Finder.MAP_MOVE_DISTANCE, z)
      # That's all for positive y axis, collect them.
      self.all_map_points.append(pos7)
      self.all_map_points.append(pos8)
      self.all_map_points.append(pos9)
      self.all_map_points.append(pos10)
      self.all_map_points.append(pos11)
      self.all_map_points.append(pos12)
      
      self.messager.publish_anchor_poses(self.all_map_points)

    def move_to_current_map_point(self):
      # go along the negative y axis first
      print "Finder: marking"

      for i in range(len(self.all_map_points)):
        pt = self.all_map_points[i]
        if i < self.cur_map_point_index:
          self.mark_map_point(pt, 1.0, 0.6, 0.0, 0.2, 83456315 + i * 100, i) # orange
        elif i > self.cur_map_point_index:
          self.mark_map_point(pt, 1.0, 0.5, 0.0, 0.2, 83456316 + i * 100, i) # darkorange
        elif i == self.cur_map_point_index:
          self.mark_map_point(pt, 1.0, 0.8, 0.0, 0.5, 83456317 + i * 100, i) # gold

          # Go to p
          goal = MoveBaseGoal()
          goal.target_pose.header.frame_id = 'map'
          goal.target_pose.header.stamp = rospy.Time().now()
          goal.target_pose.pose.position = pt
          goal.target_pose.pose.orientation = self.robot_quarternion
          print "Finder: publishing goal"
          self.move_base_client.send_goal(goal)
          self.move_base_client.wait_for_result(rospy.Duration(20.0))
      
    def move_to_next_map_point(self):
      self.cur_map_point_index = (self.cur_map_point_index + 1) % len(self.all_map_points)
      self.move_to_current_map_point()


    def find(self):
      # Look around
      print "Finder:", "looking around..."
      def script():
        rospy.sleep(Finder.FIND_TIME_SECS)

      degree_sum = 0
      # while degree_sum >= 0 and degree_sum < 360:
      for pan in Finder.FIND_HEAD_PANS:
        for tilt in Finder.FIND_HEAD_TILTS:
          self.head.pan_tilt(pan, tilt)
          rospy.sleep(0.5)
          self.planner.session(script, Finder.FIND_CONFIDENCE_DROP_RATE, 2.0)

    def move_and_find(self):
      # Move robot to a nearest anchor point
      pass

    def observe_pose(self, pose, frame_id):
      # Observe a pose
      # (Finder, Pose, string) -> None
      # Takes OBSERVE_TIME_SECS(3) seconds
      print "Finder:", "observing..."

      # MESSAGER USAGE
      if frame_id == "map":
        self.messager.publish_look_at_pose(pose.position)
      else:
        print "Finder: observing non-map pose, not displaying"

      def script():
        rospy.sleep(Finder.OBSERVE_TIME_SECS)

      object_marker = Marker()
      object_marker.ns = "objects"
      object_marker.id = 83456215
      object_marker.header.frame_id = frame_id
      object_marker.type = Marker.SPHERE
      object_marker.pose = pose
      object_marker.scale.x = 0.1
      object_marker.scale.y = 0.1
      object_marker.scale.z = 0.1
      object_marker.color.r = 0
      object_marker.color.g = 1
      object_marker.color.b = 0
      object_marker.color.a = 0.3
      self.marker_pub.publish(object_marker)

      # self.head.look_at(frame_id, pose.position.x, pose.position.y, pose.position.z)
      # self.planner.session(script, Finder.OBSERVE_POSE_CONFIDENCE_DROP_RATE, 1.0)
      # self.head.look_at(frame_id, pose.position.x, pose.position.y, pose.position.z + 0.01)
      # self.planner.session(script, Finder.OBSERVE_POSE_CONFIDENCE_DROP_RATE, 1.0)
      self.head.look_at(frame_id, pose.position.x, pose.position.y, pose.position.z)
      rospy.sleep(1)
      self.planner.session(script, Finder.OBSERVE_POSE_CONFIDENCE_DROP_RATE, 1.0)

      # MESSAGER USAGE
      self.messager.publish_look_at_pose(None)

    def save_robot_pose_cb(self, robot_pose_msg):
        # (Planner, PoseWithCovarianceStamped) -> None
        self.robot_point = robot_pose_msg.pose.pose.position
        self.robot_quarternion = robot_pose_msg.pose.pose.orientation
    
    def mark_map_point(self, point, r, g, b, a, id, text):     
      object_marker = Marker()
      object_marker.ns = "objects"
      object_marker.id = id
      object_marker.header.frame_id = "map"
      object_marker.type = Marker.SPHERE
      object_marker.pose.position = point
      object_marker.scale.x = 1
      object_marker.scale.y = 1
      object_marker.scale.z = 1
      object_marker.color.r = r
      object_marker.color.g = g
      object_marker.color.b = b 
      object_marker.color.a = a
      self.marker_pub.publish(object_marker)

      name_marker = Marker()
      name_marker.id = id + 1762523
      name_marker.header.frame_id = "map"
      name_marker.type = Marker.TEXT_VIEW_FACING
      name_marker.pose.position = copy.deepcopy(point)
      name_marker.pose.position.z += 1.0
      name_marker.scale.x = 0.25
      name_marker.scale.y = 0.25
      name_marker.scale.z = 0.25
      name_marker.color.b = 1
      name_marker.color.a = 1
      name_marker.text = "ball" + str(text)
      self.marker_pub.publish(name_marker)
