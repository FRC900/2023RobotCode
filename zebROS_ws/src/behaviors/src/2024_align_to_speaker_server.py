#! /usr/bin/env python3

import actionlib
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import math
import behavior_actions.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
from frc_msgs.msg import MatchSpecificData
import std_srvs.srv
import angles

class Aligner:
    # create messages that are used to publish feedback/result
    _feedback = behavior_actions.msg.AlignToSpeaker2024Feedback()
    _result = behavior_actions.msg.AlignToSpeaker2024Result()

    def __init__(self, name):   
        self._action_name = name
        self.tolerance = rospy.get_param("tolerance")
        self.velocity_tolerance = rospy.get_param("velocity_tolerance")
        self.min_samples = rospy.get_param("min_samples")
        self.color = 0
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.AlignToSpeaker2024Action, execute_cb=self.aligner_callback, auto_start = False)
        self._as.start()
        self.current_yaw = 0
        self.current_orient_effort = 0
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.object_publish = rospy.Publisher("/teleop/orientation_command", std_msgs.msg.Float64, queue_size =1)
        self.object_subscribe = rospy.Subscriber("/imu/zeroed_imu", sensor_msgs.msg.Imu ,self.imu_callback)

        self.team_subscribe = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.data_callback)

        self.sub_effort = rospy.Subscriber("/teleop/orient_strafing/control_effort", std_msgs.msg.Float64, self.robot_orientation_effort_callback)
        self.pub_cmd_vel = rospy.Publisher("/speaker_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

        self.pub_dist_and_ang_vel = rospy.Publisher("/speaker_align/dist_and_ang", behavior_actions.msg.AutoAlignSpeaker, queue_size=1) #distance and angle

        self._feedback.aligned = False

        self.valid_samples = 0

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        yaw = euler[2]
        self.current_yaw = yaw

        offset_point = tf2_geometry_msgs.PointStamped()
        offset_point.header.frame_id = 'bluespeaker' if self.color == 1 else 'redspeaker'
        offset_point.header.stamp = rospy.get_rostime()

        offset_point.point.y = 0

        try:
            trans = self.tfBuffer.lookup_transform('base_link', offset_point.header.frame_id, rospy.Time())
            destination = tf2_geometry_msgs.do_transform_point(offset_point, trans)

            msg = std_msgs.msg.Float64()
            dist_ang_msg = behavior_actions.msg.AutoAlignSpeaker()

            dist_ang_msg.distance = math.sqrt(destination.point.x ** 2 + destination.point.y ** 2)
            msg.data = math.pi + self.current_yaw + math.atan2(destination.point.y, destination.point.x)
            dist_ang_msg.angle = math.atan2(destination.point.y, destination.point.x)

            self.pub_dist_and_ang_vel.publish(dist_ang_msg)
        except Exception as e:
            rospy.logwarn_throttle(1, f"align_to_speaker: can't publish distance {e}")
        
    def data_callback(self, data_msg):
        self.color = data_msg.allianceColor
        
    def robot_orientation_effort_callback(self, msg):
        self.current_orient_effort = msg.data
    def aligner_callback(self, goal: behavior_actions.msg.AlignToSpeaker2024Goal):
        success = True
        rospy.loginfo(f"Auto Aligning Actionlib called with goal {goal}")
        rate = rospy.Rate(60.0)

        self.valid_samples = 0

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            offset_point = tf2_geometry_msgs.PointStamped()
            offset_point.header.frame_id = 'bluespeaker' if self.color == 1 else 'redspeaker'
            offset_point.header.stamp = rospy.get_rostime()

            if goal.offsetting == True:
                offset_point.point.y = 2
            else:
                offset_point.point.y = 0

            try:
                trans = self.tfBuffer.lookup_transform('base_link', offset_point.header.frame_id, rospy.Time())
                destination = tf2_geometry_msgs.do_transform_point(offset_point, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                rate.sleep()
                continue

            msg = std_msgs.msg.Float64()
            dist_ang_msg = behavior_actions.msg.AutoAlignSpeaker()

            dist_ang_msg.distance = math.sqrt(destination.point.x ** 2 + destination.point.y ** 2)
            msg.data = math.pi + self.current_yaw + math.atan2(destination.point.y, destination.point.x)
            dist_ang_msg.angle = math.atan2(destination.point.y, destination.point.x)

            self._feedback.error = abs(angles.shortest_angular_distance(msg.data, self.current_yaw))
            self._feedback.aligned = False

            self.object_publish.publish(msg) 
            self.pub_dist_and_ang_vel.publish(dist_ang_msg)
            
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)

            rospy.loginfo(f"Align to speaker {abs(angles.shortest_angular_distance(msg.data, self.current_yaw))}")

            if self._feedback.error < self.tolerance and abs(self.current_orient_effort) < self.velocity_tolerance:
                self.valid_samples += 1
            else:
                self.valid_samples = 0

            if self.valid_samples >= self.min_samples:
                self._feedback.aligned = True
                rospy.loginfo_throttle(0.5, "2024_align_to_speaker: aligned")
                success = True
                if not goal.align_forever:
                    cmd_vel_msg = geometry_msgs.msg.Twist()
                    cmd_vel_msg.angular.x = 0
                    cmd_vel_msg.angular.y = 0
                    cmd_vel_msg.angular.z = 0.0
                    cmd_vel_msg.linear.x = 0
                    cmd_vel_msg.linear.y = 0
                    cmd_vel_msg.linear.z = 0
                    self.pub_cmd_vel.publish(cmd_vel_msg)
                    self._as.publish_feedback(self._feedback)
                    break
            
            self._as.publish_feedback(self._feedback)
            rate.sleep()
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('aligner')
    server = Aligner(rospy.get_name())
    rospy.spin()