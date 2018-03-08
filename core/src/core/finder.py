import rospy
import fetch_api
from visualization_msgs.msg import Marker

class Finder:
    # All poses in Finder are of frame "/map", unless otherwise specified
    OBSERVE_TIME_SECS = 3.0
    FIND_TIME_SECS = 3.0
    FIND_TURN_STEP_DEGREES = 60.0
    FIND_HEAD_PANS = [0.5, 0.0, -0.5]
    FIND_HEAD_TILTS = [0.5, 1.0, 1.5]
    ANCHOR_DISTANCE = 5
    FIND_CONFIDENCE_DROP_RATE = 1.0
    OBSERVE_POSE_CONFIDENCE_DROP_RATE = 0.8

    def __init__(self, planner):
      self.head = fetch_api.Head()
      self.base = fetch_api.Base()
      self.planner = planner
      self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
      
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
      def script():
        rospy.sleep(Finder.OBSERVE_TIME_SECS)
      def script2():
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
      self.planner.session(script, Finder.OBSERVE_POSE_CONFIDENCE_DROP_RATE, 1.0)