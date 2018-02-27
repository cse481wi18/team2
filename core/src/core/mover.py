import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Mover:
    def __init__(self):
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.save_robot_pose_cb)
        self.robot_point = None

    def save_robot_pose_cb(self, robot_pose_msg):
        # (Planner, PoseWithCovarianceStamped) -> None
        self.robot_point = robot_pose_msg.pose.pose.position
    

    # def goto_pose(self, pose):
    #     if pose is None:
    #         rospy.logerr('No pose')
    #     else:
    #         goal = MoveBaseGoal()
    #         goal.target_pose.header.frame_id = 'map'
    #         goal.target_pose.header.stamp = rospy.Time().now()
    #         goal.target_pose.pose = pose
    #         self._move_base_client.send_goal(goal)

    #         return self._move_base_client.wait_for_result(rospy.Duration(20.0))

    def goto_pose(self, pose):     
        print "Going to first pose"
        pose.orientation = quaternion_between(self.robot_point, pose.position)
        print pose

        poseStamped = PoseStamped()
        poseStamped.header.frame_id = "map"
        poseStamped.pose = pose
        print "pose:", pose

        goal = MoveBaseGoal()
        goal.target_pose = poseStamped

        print "robot point:, robot_point

        self.move_base_client.send_goal(goal)
        print self.move_base_client.wait_for_result(rospy.Duration(20.0))

        return poseStamped
        #  = copy.deepcopy(poseStamped)
        # object_poseStamped.pose.position = self.ordered_object_points[0]

        # self.listener.waitForTransform('/base_link', '/map', rospy.Time(), rospy.Duration(4.0))
        # base_link_pose = self.listener.transformPose('/base_link', object_poseStamped).pose
        # self.grabber.move(base_link_pose)
        # print "base_link_pose:", base_link_pose