#!/usr/bin/env python3

import rospy
import actionlib
import time
import math
import std_msgs.msg
import behavior_actions.msg
import numpy
import geometry_msgs.msg
import sensor_msgs.msg


from behavior_actions.msg import Shooting2024Goal, Shooting2024Feedback, Shooting2024Result, Shooting2024Action
from behavior_actions.msg import MoveWhileShooting2024Goal, MoveWhileShooting2024Feedback, MoveWhileShooting2024Result, MoveWhileShooting2024Action
from behavior_actions.msg import AlignToSpeaker2024Goal, AlignToSpeaker2024Feedback, AlignToSpeaker2024Result, AlignToSpeaker2024Action
from behavior_actions.msg import AutoAlignSpeaker
from std_msgs.msg import Float64
from interpolating_map import InterpolatingMap
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped



class MovingWhileShootingServer(object):
    def __init__(self, name):
        self.action_name = name
        self.result = MoveWhileShooting2024Result()
        self.feedback = MoveWhileShooting2024Feedback()

        self.dynamic_move_time = rospy.get_param("dynamic_move_time")
        #self.cmd_vel_pub_ = rospy.Publisher("/auto_note_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)


        self.negative = None

        #self.object_publish = rospy.Publisher("/teleop/orientation_command", std_msgs.msg.Float64, queue_size =1)
        self.object_subscribe = rospy.Subscriber("/teleop/orientation_command", std_msgs.msg.Float64, self.orientation_command_cb, tcp_nodelay=True, queue_size=1)
        self.current_orient_effort_cb_real = 0.0

        self.sub_effort = rospy.Subscriber("/teleop/orient_strafing/control_effort", std_msgs.msg.Float64, self.robot_orientation_effort_callback, tcp_nodelay=True)
        self.current_orient_effort_cb = 0

        self.object_subscribe = rospy.Subscriber("/imu/zeroed_imu", sensor_msgs.msg.Imu, self.imu_callback, tcp_nodelay=True)
        self.current_yaw = 0
        
        #self.cmd_vel_pub = rospy.Publisher("/speaker_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/auto_note_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1, tcp_nodelay=True)



        self.angle_holder = std_msgs.msg.Float64()

        self.cmd_vel_sub = rospy.Subscriber("/teleop/swerve_drive_controller/cmd_vel", geometry_msgs.msg.Twist, self.cmd_vel_sub_magnitude_convert_callback, tcp_nodelay=True, queue_size=1)
        self.scaled_x_val = 0.0
        self.scaled_y_val = 0.0
        self.scaled_x_transform = 0.0
        self.scaled_y_transform = 0.0
        #creating cmd_vel_sub, to read in the values of the velocity values whem moving in sim, then taking callback and scaling magnitude to 1 m/s

        self.angle_puller = rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, self.dist_and_ang_cb, tcp_nodelay=True ,queue_size=1)
        self.angle_cb = 0.0
        self.distance_cb = 0.0
        self.feedback_error_value = 0.0
        #creating angle_puller to subscribe nad read twist values, this should align the robot accorindlgy

        #        self.pub_cmd_vel = rospy.Publisher("/speaker_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        

        



        self.shooting_client = actionlib.SimpleActionClient('/shooting/shooting_server_2024', Shooting2024Action)
        rospy.loginfo("Waiting for shooting server")
        self.shooting_client.wait_for_server()

        self.align_to_speaker_client = actionlib.SimpleActionClient('/align_to_speaker/align_to_speaker_2024', AlignToSpeaker2024Action)
        rospy.loginfo("Waiting for AlignToSpeaker")
        self.align_to_speaker_client.wait_for_server()

        self.server = actionlib.SimpleActionServer(self.action_name, MoveWhileShooting2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()
        rospy.loginfo("2024_move_while_shooting_server: initialized")



        #self.x_field_relative_vel_align = self.current_robot_cmd_vel.linear.x * math.cos(-(self.current_yaw)) - self.current_robot_cmd_vel.linear.y * math.sin(-(self.current_yaw))
        #self.y_field_relative_vel_align = self.current_robot_cmd_vel.linear.x * math.sin(-(self.current_yaw)) + self.current_robot_cmd_vel.linear.y * math.cos(-(self.current_yaw))

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        yaw = euler[2]
        self.current_yaw = yaw


    def orientation_command_cb(self, msg):
        self.current_orient_effort_cb_real = msg.data


    def align_to_speaker_feedback_cb(self, feedback: AlignToSpeaker2024Feedback):
        self.align_to_speaker_done = feedback.aligned

    def shooting_feedback_cb(self, feedback: Shooting2024Feedback):
        self.shooting_done = (feedback.current_stage == feedback.SHOOTING)
        rospy.logwarn("2024 Align and shoot: Shooting done!")

    def robot_orientation_effort_callback(self, msg):
        self.current_orient_effort_cb = msg.data

    def dist_and_ang_cb(self, msg):
        self.angle_cb = msg.angle
        self.distane_cb = msg.distance

        self.angle_holder = self.angle_cb #might have to use this one?
        self.feedback_error_value = msg.feedback_error_value
        #rospy.loginfo("self.angle_twist_z has the new angle")
        
    def cmd_vel_sub_magnitude_convert_callback(self, msg: geometry_msgs.msg.Twist):
        #just find unit vector
        #is it even msg.linear.x?
        #the subscriber callback takes in objects of geometry_msgs.Twiststamped
        rospy.loginfo("2024 moving while shooting server: received cmd_vel_sub_magnitude convert callback")

        x_val = msg.linear.x
        y_val = msg.linear.y
        if (x_val == 0) or (y_val == 0):
            self.scaled_x_val = 0
            self.scaled_y_val = 0
            rospy.loginfo("2024 moving while shooting server: x_val and y_val is zero")
        if ((x_val > 0) or (y_val > 0)):
            rospy.loginfo("2024 moving while shooting server: x_val or y_val is greater than zero")
            self.scaled_x_transform = x_val * math.cos((-self.current_yaw)) - y_val * math.sin((-self.current_yaw))
            self.scaled_y_transform = x_val * math.sin((-self.current_yaw)) + y_val * math.cos((-self.current_yaw))
            vector_magnitude = math.hypot(self.scaled_x_transform, self.scaled_y_transform)
            rospy.loginfo("2024 moving while shooting server: take magnitude of the x_val and y_val transform")
            #self.x_field_relative_vel_align = self.current_robot_cmd_vel.linear.x * math.cos(-(self.current_yaw)) - self.current_robot_cmd_vel.linear.y * math.sin(-(self.current_yaw))
            #self.y_field_relative_vel_align = self.current_robot_cmd_vel.linear.x * math.sin(-(self.current_yaw)) + self.current_robot_cmd_vel.linear.y * math.cos(-(self.current_yaw))

            if vector_magnitude > 0:
                self.scaled_x_val = self.scaled_x_transform / vector_magnitude
                self.scaled_y_val = self.scaled_y_transform / vector_magnitude
                rospy.loginfo("2024_shooting_server, convert callback")

               

    def execute_cb(self, goal: MoveWhileShooting2024Goal):
        rospy.loginfo("move while shooting server 2024: has received execute cb")
        self.result.success = False
        shooting_goal = Shooting2024Goal()
        align_to_speaker_goal = AlignToSpeaker2024Goal()

        shooting_goal.distance = self.distance_cb
        shooting_goal.mode = shooting_goal.SPEAKER
        shooting_goal.setup_only = False
        shooting_goal.leave_spinning = False
        shooting_goal.request_time = rospy.get_time()
        shooting_done = False
        def shooting_done_cb(state, result):
            nonlocal shooting_done
            shooting_done = True
        self.shooting_client.send_goal(shooting_goal, done_cb=shooting_done_cb)
        rospy.loginfo("got here send shooting client cb")

        align_to_speaker_goal.align_forever = True
        align_to_speaker_goal.mode = 0
        align_to_speaker_goal.offsetting = False
        rospy.loginfo("got here, got status for the align to speaker goal")
        rospy.loginfo("2024 move while shooting server requesting align angle values")
        self.align_to_speaker_client.send_goal(align_to_speaker_goal, feedback_cb=self.align_to_speaker_feedback_cb)
        rospy.loginfo("align to speaker goal sent")
        #while conditoin with preempt and what not
        r = rospy.Rate(60.0)
        request_time = rospy.get_time()
        current_time = rospy.get_time()





        #while ((current_time - request_time) < self.dynamic_move_time):
        while True:
            #rospy.loginfo("move while shooting server 2024: is in execute cb while loop")
            current_time = rospy.get_time()
            #while not enough time has passed, go through this entire loop
           


            if self.server.is_preempt_requested():
                #if preempted, cancel the goals that we send and make sure the stuff that we are doing is preempted
                cmd_vel_msg_move_and_shoot = geometry_msgs.msg.Twist()

                rospy.loginfo("move while shooting server 2024: preempted")
                #self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.align_to_speaker_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                cmd_vel_msg_move_and_shoot.linear.x = 0
                cmd_vel_msg_move_and_shoot.linear.y = 0
                cmd_vel_msg_move_and_shoot.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel_msg_move_and_shoot)
                return
            if goal.move_align == True:
                #if we do intend on moving, aligning and shooting then do the following
                #rospy.loginfo("move while shooting server 2024: move_align is true, setting velocity and angular conditions")
                #rospy.loginfo(f"this is the angle we are using to align: {self.angle_cb}")
                #self.object_publish.publish(self.angle_cb) #main align command


                angle = math.atan2(self.scaled_y_val, self.scaled_x_val) - self.current_yaw

                # Calculate the magnitude of the field-relative velocity command
                magnitude = math.sqrt(self.scaled_x_val**2 + self.scaled_y_val**2)

                # Transform the field-relative velocity command to robot-relative
                robot_relative_x = magnitude * math.cos(angle)
                robot_relative_y = magnitude * math.sin(angle)


            #self.x_field_relative_vel_align = self.current_robot_cmd_vel.linear.x * math.cos(-(self.current_yaw)) - self.current_robot_cmd_vel.linear.y * math.sin(-(self.current_yaw))
            #self.y_field_relative_vel_align = self.current_robot_cmd_vel.linear.x * math.sin(-(self.current_yaw)) + self.current_robot_cmd_vel.linear.y * math.cos(-(self.current_yaw))

            
                cmd_vel_msg_move_and_shoot = geometry_msgs.msg.Twist()
                cmd_vel_msg_move_and_shoot.linear.x = self.scaled_x_val * math.cos((-self.current_yaw)) - self.scaled_y_val * math.sin((-self.current_yaw))
                cmd_vel_msg_move_and_shoot.linear.y = self.scaled_y_val * math.sin((-self.current_yaw)) + self.scaled_y_val * math.cos((-self.current_yaw))
                cmd_vel_msg_move_and_shoot.linear.z = 0.0
                cmd_vel_msg_move_and_shoot.angular.z = self.current_orient_effort_cb 
                cmd_vel_msg_move_and_shoot.angular.y = 0.0
                cmd_vel_msg_move_and_shoot.angular.x = 0.0

                if abs(self.feedback_error_value) > 0.1: 
                    cmd_vel_msg_move_and_shoot.angular.z += 1.0 * numpy.sign(self.current_orient_effort_cb) 

                self.cmd_vel_pub.publish(cmd_vel_msg_move_and_shoot)

            r.sleep() 

        self.result.success = True
        self.server.set_succeeded(self.result)
        return

if __name__ == '__main__':
    rospy.init_node('move_while_shooting_server_2024')
    server = MovingWhileShootingServer(rospy.get_name())
    rospy.spin()