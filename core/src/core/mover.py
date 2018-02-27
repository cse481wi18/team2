import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Mover:
    def __init__(self):
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def goto_pose(self, pose):
        if pose is None:
            rospy.logerr('No pose')
        else:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time().now()
            goal.target_pose.pose = pose
            self._move_base_client.send_goal(goal)

            return self._move_base_client.wait_for_result(rospy.Duration(20.0))