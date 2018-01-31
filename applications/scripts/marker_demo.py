#!/usr/bin/env python

import rospy

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA



class NavPath(object):
    def __init__(self, marker_publisher):
        self._path = []
        self._marker_publisher = marker_publisher

    def show_point(self, marker_publisher, points):
      marker = Marker(
                  type=Marker.SPHERE_LIST,
                  id=0,
                  lifetime=rospy.Duration(1.5),
                  points=points,
                  #pose=Pose(points, Quaternion(0, 0, 0, 1)),
                  scale=Vector3(0.06, 0.06, 0.06),
                  header=Header(frame_id='odom'),
                  color=ColorRGBA(1.0, 1.0, 1.0, 0.8))
      marker_publisher.publish(marker)
            
    def callback(self, msg):
        rospy.loginfo(msg)
        if True:
            self._path.append(msg.pose.pose.position)
            self.show_point(self._marker_publisher, self._path)

def show_text_in_rviz(marker_publisher, text):
  marker = Marker(
              type=Marker.TEXT_VIEW_FACING,
              id=0,
              lifetime=rospy.Duration(1.5),
              pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
              scale=Vector3(0.06, 0.06, 0.06),
              header=Header(frame_id='base_link'),
              color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
              text=text)
  marker_publisher.publish(marker)



def wait_for_time():                                              
  """Wait for simulated time to begin.                          
  """                                                           
  while rospy.Time().now().to_sec() == 0:                       
      pass

def main():
  rospy.init_node('my_node')
  wait_for_time()

  marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
  nav_path = NavPath(marker_publisher)
  rospy.Subscriber('odom', Odometry, nav_path.callback)
  
  '''
  rospy.init_node('my_node')
  wait_for_time()
  marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
  rospy.sleep(0.5)                                                             
  show_text_in_rviz(marker_publisher, 'Hello world!')
  '''

  rospy.spin()

if __name__ == '__main__':
  main()
