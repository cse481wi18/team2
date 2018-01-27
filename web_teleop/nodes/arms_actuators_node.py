#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetArm, SetArmResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._arm = fetch_api.Arm()
        self._arm_joints = fetch_api.ArmJoints()

    def handle_set_arm(self, request):
        # TODO: move the arm to the requested height
        value = request.value
        nodename = request.node_name
        setValueFn = getattr(self._arm_joints, nodename)
        setValueFn(value)

        self._arm.move_to_joints(self._arm_joints)
        return SetArmResponse()


def main():
    rospy.init_node('web_teleop_arms_actuators')
    wait_for_time()
    server = ActuatorServer()
    arm_service = rospy.Service('web_teleop/set_arm', SetArm,
                                  server.handle_set_arm)
    rospy.spin()

if __name__ == '__main__':
    main()