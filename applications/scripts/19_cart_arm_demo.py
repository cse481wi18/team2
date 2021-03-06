#! /usr/bin/env python

import math
import fetch_api
import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def wait_for_time():                                                                          
    """Wait for simulated time to begin.
    """                                                                                       
    while rospy.Time().now().to_sec() == 0:                                                   
        pass


def print_usage():                                                                            
    print 'Usage: ???'
        
def main():
    rospy.init_node('car_arm_demo')
    wait_for_time()

    pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))
    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    ps1.pose = pose1
    ps2 = PoseStamped()
    ps2.header.frame_id = 'base_link'
    ps2.pose = pose2
    gripper_poses = [ps1, ps2]

    arm = fetch_api.Arm()

    error = None
    i = 0
    print "hello"
    while error is None:
      error = arm.move_to_pose(gripper_poses[i % len(gripper_poses)])
      i += 1
      rospy.sleep(1)

    rospy.logerr(error)
    print error

    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)


if __name__ == '__main__':
    main()