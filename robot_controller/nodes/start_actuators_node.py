#!/usr/bin/env python

import fetch_api
import rospy
from robot_controller.srv import SetStart, SetStartResponse
from std_msgs.msg import String

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class ActuatorServer(object):
    def __init__(self):
        self._start_msg_pub = rospy.Publisher('robot_controller/start_msg', \
                                                String, queue_size=5)

    def handle_set_start(self, request):
        start_msg = request.start_msg
        if start_msg == 'start':
            self._start_msg_pub.publish(start_msg)
        # TODO: move the torso to the requested height
        return SetStartResponse()

def main():
    rospy.init_node('robot_controller_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('robot_controller/set_start', SetStart,
                                  server.handle_set_start)
    rospy.spin()


if __name__ == '__main__':
    main()