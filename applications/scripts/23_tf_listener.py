#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    rospy.sleep(0.1)

    while not rospy.is_shutdown():
        rospy.sleep(1)
        try:
            (trans,rot) = listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0))
            print trans
            print rot

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "error"
            continue
