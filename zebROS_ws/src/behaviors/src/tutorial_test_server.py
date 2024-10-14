#!/usr/bin/env python3

# Write an actionlib server to wait until test_limit_switch is toggled, then run test_motor at a given output provided in the goal

# Some useful hints: see lines 109-111 of zebROS_ws/src/ros_control_boilerplate/config/2024_compbot_base_jetson.yaml
# and run `rostopic echo /frcrobot_jetson/joint_states` to see the limit switch state
# You can toggle the test_limit_switch via the simulated driver station (`driversim`) under the "Simulated Inputs" tab
# You can see the motor's state under the "Talon Values" tab

# Helpful template things:
import rospy
import actionlib
# This is the action that we'll be using, similar to the FibonacciAction from the tutorial:
from behavior_actions.msg import TutorialTestGoal, TutorialTestFeedback, TutorialTestResult, TutorialTestAction
from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from sensor_msgs.msg import JointState

# Add code here :)