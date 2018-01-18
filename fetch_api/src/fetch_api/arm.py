from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal

import rospy
import actionlib

from .arm_joints import ArmJoints

ACTION_NAME = "arm_controller/follow_joint_trajectory"

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point

        trajectory = JointTrajectory()
        trajectory.joint_names = ArmJoints.names()
        point = JointTrajectoryPoint()
        point.positions = arm_joints.values()
        point.velocities = [0.0] * len(ArmJoints.names())
        trajectory.points = [point]

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory

        # TODO: Send goal
        # TODO: Wait for result
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
