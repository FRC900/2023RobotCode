#!/usr/bin/env python3

# Helpful template things:
import rospy
import actionlib
# This is the action that we'll be using, similar to the FibonacciAction from the tutorial:
from behavior_actions.msg import TurnToFrameAction, TurnToFrameFeedback, TurnToFrameResult, TurnToFrameGoal
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

import math
import tf2_ros
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2

import tf2_geometry_msgs

class TurnToFrameServer:
    def __init__(self):
        self.imu_sub = rospy.Subscriber("/imu/zeroed_imu", Imu, self.imu_callback)
        self.angle_pub = rospy.Publisher("/teleop/orientation_command", Float64, queue_size=1)

        self.yaw = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.server = actionlib.SimpleActionServer('turn_to_frame', TurnToFrameAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        self.yaw = euler[2]

    def execute_cb(self, goal):
        # This callback is called when the action server is called
        # The goal is the input to the action server
        # The result is what the action server returns
        # The feedback is what the action server sends back to the client during execution

        # This is the feedback message that we'll be sending back to the client
        feedback = TurnToFrameFeedback()

        # Find transform from the frame named in goal.point.header.frame_id to base_link
        try:
            transform = self.tf_buffer.lookup_transform("base_link", goal.point.header.frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            result = TurnToFrameResult()
            result.success = False
            self.server.set_aborted(result)
            return
        
        # Transform the point into base_link
        base_link_relative_point = tf2_geometry_msgs.do_transform_point(goal.point, transform)

        # Calculate the angle to turn to
        robot_relative_angle = math.atan2(base_link_relative_point.point.y, base_link_relative_point.point.x)
        field_relative_angle = self.yaw + robot_relative_angle

        # Publish angle to turn to
        angle_msg = Float64()
        angle_msg.data = field_relative_angle
        self.angle_pub.publish(angle_msg)

if __name__ == '__main__':
    rospy.init_node('turn_to_frame_server', anonymous=True)
    server = TurnToFrameServer()
    rospy.spin()