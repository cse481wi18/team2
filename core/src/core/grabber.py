import fetch_api
import rospy
import copy
import tf.transformations as tft
from tf import TransformListener
from geometry_msgs.msg import PoseStamped

def to_pose_stamped(pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "base_link"
    pose_stamped.header.stamp = rospy.Time(0)
    return pose_stamped

def forward(p, dist):
    return move_pose(p, dist, 0, 0)

def move_pose(p, x, y, z):
    p = copy.deepcopy(p)
    listener = TransformListener(rospy.Duration(10))
    listener.waitForTransform('base_link', 'wrist_roll_link', rospy.Time(), rospy.Duration(4.0))
    mat = tft.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    offset = mat.dot(np.array([x, y, z, 1]))
    p.position.x += offset[0]
    p.position.y += offset[1]
    p.position.z += offset[2]
    return p

class Grabber:
    def __init__(self):
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()

    def move(self, post_stamped):
        
        self._gripper.open()
        if self.check_pose(post_stamped):
            self._arm.move_to_pose(to_pose_stamped(p))

    def check_pose(self, pose_stamped):
        return self._arm.compute_ik(pose_stamped)
       
