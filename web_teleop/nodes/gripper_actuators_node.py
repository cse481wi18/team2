#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetGripper, SetGripperResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._gripper = fetch_api.Gripper()

    def handle_set_gripper(self, request):
        # TODO: move the Gripper to the requested height
        if request.isClose:
            self._gripper.close(request.force)
        else:
            self._gripper.open()

        return SetGripperResponse()


def main():
    rospy.init_node('web_teleop_gripper_actuators')
    wait_for_time()
    server = ActuatorServer()
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper,
                                  server.handle_set_gripper)
    rospy.spin()


if __name__ == '__main__':
    main()