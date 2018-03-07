#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
from control_msgs.msg import PointHeadAction
from control_msgs.msg import PointHeadActionGoal
from control_msgs.msg import PointHeadGoal
from geometry_msgs.msg import Point

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal

import math
import rospy
import actionlib

LOOK_AT_ACTION_NAME = 'head_controller/point_head'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  # TODO: Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi / 2  # TODO: Minimum pan angle, in radians.
    MAX_PAN = math.pi / 2  # TODO: Maximum pan angle, in radians.
    MIN_TILT = -math.pi / 2  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = math.pi / 4  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        self.traj_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        self.point_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, PointHeadAction)
        while not self.traj_client.wait_for_server(timeout=rospy.Duration(1)):
            rospy.logwarn('Waiting for head trajectory server...')
        while not self.point_client.wait_for_server(timeout=rospy.Duration(1)):
            rospy.logwarn('Waiting for head pointing server...')

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        # TODO: Send the goal
        # TODO: Wait for result
        goal = PointHeadGoal()
        goal.target.point = Point(x, y, z)
        goal.target.header.frame_id = frame_id

        self.point_client.send_goal(goal)
        self.point_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        # TODO: Create a trajectory point
        # TODO: Set positions of the two joints in the trajectory point
        # TODO: Set time of the trajectory point

        pan = min(pan, Head.MAX_PAN)
        pan = max(pan, Head.MIN_PAN)
        tilt = min(tilt, Head.MAX_TILT)
        tilt = max(tilt, Head.MIN_TILT)

        trajectory = JointTrajectory()
        trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]
        trajectory.points = [point]

        # TODO: Create goal
        # TODO: Add joint names to the list
        # TODO: Add trajectory point created above to trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        
        # TODO: Send the goal
        # TODO: Wait for result
        self.traj_client.send_goal(goal)
        self.traj_client.wait_for_result(rospy.Duration.from_sec(5.0))
