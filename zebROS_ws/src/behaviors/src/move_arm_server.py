#! /usr/bin/env/python3

import rospy
import actionlib
from behavior_actions import Arm, ArmGoal, ArmFeedback, ArmResult

class ArmAction(object):
    _feedback = ArmFeedback()
    _result = ArmResult()
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Arm.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
   
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True

        self._feedback.percent_complete = 0
        rospy.loginfo(f"{self._action_name}: Executing. Moving arm ")



    rospy.init_node('arm_control', anonymous=True)
