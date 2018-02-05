#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetHead, SetHeadResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._head = fetch_api.Head()

    def handle_set_head(self, request):
        # TODO: move the Head to the requested height
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadResponse()

def main():
    rospy.init_node('web_teleop_head_actuators')
    wait_for_time()
    server = ActuatorServer()
    head_service = rospy.Service('web_teleop/set_head', SetHead,
                                  server.handle_set_head)
    rospy.spin()


if __name__ == '__main__':
    main()
