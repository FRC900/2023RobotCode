#! /usr/bin/env python3

import actionlib
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import math
import behavior_actions.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
import std_srvs.srv

class Aligner:
    # create messages that are used to publish feedback/result
    _feedback = behavior_actions.msg.AlignToSpeaker2024Feedback()
    _result = behavior_actions.msg.AlignToSpeaker2024Result()

    def __init__(self, name):   
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.AlignToSpeaker2024Action, execute_cb=self.aligner_callback, auto_start = False)
        self._as.start()
        self.current_yaw = 0
        self.current_orient_effort = 0
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.object_publish = rospy.Publisher("/teleop/orientation_command", std_msgs.msg.Float64, queue_size =1)
        self.object_subscribe = rospy.Subscriber("/imu/zeroed_imu", sensor_msgs.msg.Imu ,self.imu_callback)

        self.sub_effort = rospy.Subscriber("/teleop/orient_strafing/control_effort", std_msgs.msg.Float64, self.robot_orientation_effort_callback)
        self.pub_cmd_vel = rospy.Publisher("/speaker_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        yaw = euler[2]
        self.current_yaw = yaw
    def robot_orientation_effort_callback(self, msg):
        self.current_orient_effort = msg.data
    def aligner_callback(self, goal):
        success = True
        rospy.loginfo(f"Auto Aligning Actionlib called with goal {goal}")
        rate = rospy.Rate(50.0)

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            try:
                trans = self.tfBuffer.lookup_transform('base_link', 'speaker', rospy.Time())#gets 
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                rate.sleep()
                continue

            msg = std_msgs.msg.Float64()
            msg.data = self.current_yaw + math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            self.object_publish.publish(msg) 
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0 # todo, tune me
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)
            
            rate.sleep()
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('aligner')
    server = Aligner(rospy.get_name())
    rospy.spin()