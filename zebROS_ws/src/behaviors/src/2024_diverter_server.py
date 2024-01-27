#!/usr/bin/env python3

import rospy

import actionlib

from behavior_actions.msg import NoteDiverterFeedback, NoteDiverterResult, NoteDiverterGoal, NoteDiverterAction
from std_msgs.msg import Float64

class NoteDiverterActionServer(object):
    # create messages that are used to publish feedback/result
    _result = NoteDiverterResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, NoteDiverterAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal: NoteDiverterGoal):
        # Conveyer topic
        conveyor_pub = rospy.Publisher("/frcrobot_jetson/note_conveyor_controller/command", Float64, queue_size=1)
        conveyor_pct = Float64()
        if goal.mode == goal.OFF:
            conveyor_pct.data = 0
        else:
            conveyor_pct.data = rospy.get_param("note_conveyor_speed")

        conveyor_pub.publish(conveyor_pct)

        # Diverter topic
        diverter_pub = rospy.Publisher("/frcrobot_jetson/note_diverter_controller/command", Float64, queue_size=1)
        diverter_pct = Float64()
        diverter_speed = rospy.get_param("note_diverter_speed")
        if goal.mode == goal.OFF:
            diverter_pct.data = 0
        elif goal.mode == goal.TO_CLAW:
            diverter_pct.data = diverter_speed
        else:
            diverter_pct.data = -1 * diverter_speed # Reverses(?)
        diverter_pub.publish(diverter_pct)

        self._result.success = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    # rospy.set_param('note_conveyor_speed', 1) # cheating
    # rospy.set_param('note_diverter_speed', 1)

    rospy.init_node('diverter_server_2024')
    server = NoteDiverterActionServer(rospy.get_name())
    rospy.spin()