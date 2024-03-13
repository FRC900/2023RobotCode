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

from behavior_actions.msg import DriveAndScore2024Action, DriveAndScore2024Goal, DriveAndScore2024Feedback, DriveAndScore2024Result 
from behavior_actions.msg import Clawster2024Action, Clawster2024Feedback, Clawster2024Result, Clawster2024Goal
from behavior_actions.msg import Arm2024Action, Arm2024Feedback, Arm2024Result, Arm2024Goal
import behavior_actions.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
from frc_msgs.msg import MatchSpecificData
import std_srvs.srv
import angles
import numpy

# Used for anytime we need to drive to a position and then score, (amp/trap) as opposed to shooting where we just need to align (2024_align_to_speaker_server.py and other)

class DriveAndScore:
    # create messages that are used to publish feedback/result
    _feedback = DriveAndScore2024Feedback()
    _result = DriveAndScore2024Result()

    def __init__(self, name):   
        self.valid_samples = 0
        self._action_name = name
        self.tolerance = rospy.get_param("tolerance")
        self.velocity_tolerance = rospy.get_param("velocity_tolerance")
        self.min_samples = rospy.get_param("min_samples")
        self.color = 0

        # get us to right spot
        self.align_client = actionlib.SimpleActionClient('/align_to_trap/align_to_trap_2024', behavior_actions.msg.AlignToTrap2024Action)
        rospy.loginfo("2024_intaking_server: waiting for shooter pivot server")
        self.align_client.wait_for_server()
        # shoot trap
        self.shooting_client = actionlib.SimpleActionClient('shooting/shooting_server_2024', behavior_actions.msg.Shooting2024Action)
        self.shooting_client.wait_for_server()
        
        
        self.arm_client = actionlib.SimpleActionClient('/arm/move_arm_server_2024', Arm2024Action)
        rospy.loginfo("2024_intaking_server: waiting for arm server")
        self.arm_client.wait_for_server()

        self.clawster_client = actionlib.SimpleActionClient('/clawster/clawster_server_2024', Clawster2024Action)
        rospy.loginfo("2024_intaking_server: waiting for clawster server")
        self.clawster_client.wait_for_server()

        self._as = actionlib.SimpleActionServer(self._action_name, DriveAndScore2024Action, execute_cb=self.score_cb, auto_start = False)
        self._as.start()
    
    def align_done_cb(self, status, result):
        rospy.loginfo("2024 drive and score, align server done")

    def score_cb(self, goal: DriveAndScore2024Goal):
        self.feed_forward = True
        success = True
        rospy.loginfo(f"Drive and score 2024 - called with goal {goal}")
        r = rospy.Rate(60.0)

        if goal.destination == goal.AMP:
            rospy.loginfo(f"Drive and score 2024 - called with goal {goal}")
            align_goal = behavior_actions.msg.AlignToTrap2024Goal()
            align_goal.destination = align_goal.AMP
            self.align_client.send_goal(align_goal )
        

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            
            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            r.sleep()
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('drive_and_score')
    server = DriveAndScore(rospy.get_name())
    rospy.spin()