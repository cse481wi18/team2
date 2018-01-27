import rospy

from nav_msgs.msg import Odometry

class NavPath(object):
    def __init__(self):
        self._path = []
            
    def callback(self, msg):
        rospy.loginfo(msg)
        if True:
            self._path.append(msg.position)

def main():
    # ...setup stuff...
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()