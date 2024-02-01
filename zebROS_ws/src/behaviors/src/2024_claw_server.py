#!/usr/bin/env python3

import rospy

import actionlib

from behavior_actions.msg import Claw2024Feedback, Claw2024Result, Claw2024Goal, Claw2024Action
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from time import sleep

switch = 0
r = rospy.rate(10)
class Claw2024ActionServer(object):
    # create messages that are used to publish feedback/result
    _result = Claw2024Result()

    def __init__(self, name):
        self.claw_pub = rospy.Publisher("/frcrobot_jetson/claw_controller/command", Float64, queue_size=1)
        self.switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, callback)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Claw2024Action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal: Claw2024Goal):
        pct_out = Float64()
        success = True
        if goal.mode == goal.INTAKE_CLAW:
            pct_out.data = intake_speed
            self.claw_pub.publish(pct_out)
            while switch == 0 and (not rospy.is_shutdown()):
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    success = False
                    break
                r.sleep()

            pct_out.data = 0
            self.claw_pub.publish(pct_out)
            if success:
                self._result.has_game_piece = True
            else:
                self._result.has_game_piece = False

        else:
            pct_out.data = outtake_speed
            self.claw_pub.publish(pct_out)
            while switch != 0 and (not rospy.is_shutdown()):
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    success = False
                    break
                r.sleep()

            sleep(delay)
            pct_out.data = 0
            self.claw_pub.publish(pct_out)
            if success:
                self._result.has_game_piece = False
            else:
                self._result.has_game_piece = True
        
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

def callback(data):
    global switch
    switch = data.position[data.name.index("claw_limit_switch")]  # Or whatever actually says when it's pressed

if __name__ == '__main__':
    rospy.init_node('claw_server_2024')
    intake_speed = rospy.get_param("intake_speed")
    outtake_speed = rospy.get_param("outtake_speed")
    delay = rospy.get_param("outtake_stop_delay")
    #intake_speed = 1
    #outtake_speed = 1
    #delay = 0.5
    server = Claw2024ActionServer(rospy.get_name())
    rospy.spin()