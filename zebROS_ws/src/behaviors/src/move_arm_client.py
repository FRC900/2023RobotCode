#! /usr/bin/env/python3

import rospy
import actionlib
from behavior_actions import Arm, ArmGoal, ArmFeedback, ArmResult


def arm_control_client():
    client = actionlib.SimpleActionClient('armcontrol', Arm)

    client.wait_for_server()
    goal = ArmGoal(order=20)

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result() 

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = arm_control_client()
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")