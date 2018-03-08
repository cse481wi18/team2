#! /usr/bin/env python

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import rospy
import copy
import math
import tf.transformations as tft


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()

        base.go_forward(0.1)
        base.turn(30 * math.pi / 180)

        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        self.received = False
        self.start_position = None
        self.latest_position = None
        self.start_quaternion = None
        self.latest_quaternion = None

    def _odom_callback(self, msg):
        self.received = True
        self.latest_position = msg.pose.pose.position
        if (self.start_position is None):
            self.start_position = self.latest_position
        self.latest_quaternion = msg.pose.pose.orientation
        if (self.start_quaternion is None):
            self.start_quaternion = self.start_quaternion
        # print self.latest_quaternion
        # TODO: do something

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # print distance
        self.start_odom = None
        self.start_position = None
        self.received = False
        while self.received is False:
            rospy.sleep(0.1)
        
        # TODO: rospy.sleep until the base has received at least one message on /odom
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.start_position)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        moved_distance = 0
        while moved_distance <= distance:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            pos = self.latest_position
            moved_distance = math.sqrt((start.x - pos.x) ** 2 + (start.y - pos.y) ** 2 + (start.z - pos.z) ** 2)

            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        angular_distance %= 360
        if (angular_distance >= 180):
            angular_distance -= 360

        # TODO: ????????????????????????????????????
        # print "base-turn: wtf"
        if angular_distance < 20 and angular_distance > -20:
            return

        # TODO: rospy.sleep until the base has received at least one message on /odom
        self.start_odom = None
        self.received = False
        while self.received is False:
            rospy.sleep(0.1)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.latest_quaternion)
        m_start = tft.quaternion_matrix([start.x, start.y, start.z, start.w])
        theta_rads_start = math.atan2(m_start[1,0], m_start[0,0])
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        difference_deg = 0

        i = 0
        while abs(difference_deg) < abs(angular_distance) and i < 100:
            # print "base-turn: wtf"
            cur = self.latest_quaternion
            m_cur = tft.quaternion_matrix([cur.x, cur.y, cur.z, cur.w])
            theta_rads_cur = math.atan2(m_cur[1,0], m_cur[0,0])

            difference = (theta_rads_cur - theta_rads_start) % (2 * math.pi)
            difference_deg = difference / math.pi * 180
            if (difference_deg >= 180):
                difference_deg -= 360 

            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()
            i += 1

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        msg = Twist()
        msg.linear = Vector3(linear_speed,0,0)
        msg.angular = Vector3(0,0,angular_speed)

        self.pub.publish(msg)
        # rospy.logerr('Not implemented.')

    # def go_forward(self, distance, speed=0.1):
    #     """Moves the robot a certain distance.

    #     It's recommended that the robot move slowly. If the robot moves too
    #     quickly, it may overshoot the target. Note also that this method does
    #     not know if the robot's path is perturbed (e.g., by teleop). It stops
    #     once the distance traveled is equal to the given distance.

    #     You cannot use this method to move less than 1 cm.

    #     Args:
    #         distance: The distance, in meters, to rotate. A positive value
    #             means forward, negative means backward.
    #         max_speed: The maximum speed to travel, in meters/second.
    #     """
    #     while self.odom is None:
    #         rospy.sleep(0.1)
    #     start = copy.deepcopy(self.odom)
    #     rate = rospy.Rate(25)
    #     distance_from_start = self._linear_distance(start, self.odom)
    #     while distance_from_start < math.fabs(distance):
    #         distance_from_start = self._linear_distance(start, self.odom)
    #         if distance_from_start >= math.fabs(distance):
    #             return
    #         direction = -1 if distance < 0 else 1
    #         self.move(direction * speed, 0)
    #         rate.sleep()

    # def turn(self, angular_distance, speed=0.5):
    #     """Rotates the robot a certain angle.

    #     This illustrates how to turn the robot by checking that the X-axis of
    #     the robot matches that of the goal.

    #     Args:
    #         angular_distance: The angle, in radians, to rotate. A positive
    #             value rotates counter-clockwise.
    #         speed: The maximum angular speed to rotate, in radians/second.
    #     """
    #     while self.odom is None:
    #         rospy.sleep(0.1)
    #     direction = -1 if angular_distance < 0 else 1

    #     current_yaw = self._yaw_from_quaternion(self.odom.orientation)
    #     goal_yaw = current_yaw + angular_distance
    #     goal_x_axis = np.array([math.cos(goal_yaw), math.sin(goal_yaw), 0])

    #     rate = rospy.Rate(100)
    #     x_axis = self._x_axis_from_quaternion(self.odom.orientation)
    #     while not np.allclose(x_axis, goal_x_axis, atol=0.01):
    #         self.move(0, direction * speed)
    #         x_axis = self._x_axis_from_quaternion(self.odom.orientation)
    #         rate.sleep()

    # def turn_alternate(self, angular_distance, speed=0.5):
    #     """Rotates the robot a certain angle.

    #     This illustrates how to turn the robot using yaw angles and careful
    #     accounting.

    #     Args:
    #         angular_distance: The angle, in radians, to rotate. A positive
    #             value rotates counter-clockwise.
    #         speed: The maximum angular speed to rotate, in radians/second.
    #     """
    #     while self.odom is None:
    #         rospy.sleep(0.1)
    #     direction = -1 if angular_distance < 0 else 1

    #     current_coord = self._yaw_from_quaternion(self.odom.orientation) % (
    #         2 * math.pi)
    #     end_coord = (current_coord + angular_distance) % (2 * math.pi)
    #     rate = rospy.Rate(25)

    #     while True:
    #         current_coord = self._yaw_from_quaternion(
    #             self.odom.orientation) % (2 * math.pi)
    #         remaining = (direction *
    #                      (end_coord - current_coord)) % (2 * math.pi)
    #         if remaining < 0.01:
    #             return
    #         speed = max(0.25, min(1, remaining))
    #         self.move(0, direction * speed)
    #         rate.sleep()

    def stop(self):
        """Stops the mobile base from moving.
        """

        self.move(0, 0)
        #rospy.logerr('Not implemented.')
