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

def find_joint_state(msg: JointState, name: str) -> float:
    # Returns the state of the joint with name `name` in the JointState message `msg`
    for i in range(len(msg.name)):
        if msg.name[i] == name:
            return msg.position[i]
    return None

# Add code here :)
class TutorialTestServer(object):
    def __init__(self, name):
        self._command_service = "/frcrobot_jetson/test_motor_controller/command"
        self._joint_state_sub = rospy.Subscriber("frcrobot_jetson/joint_states", JointState, self.joint_state_cb)
        self.limit_switch = 0
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, TutorialTestAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def joint_state_cb(self, msg):
        self.limit_switch = find_joint_state(msg, "test_limit_switch")

    def execute_cb(self, goal):
        self._feedback = TutorialTestFeedback()
        self._feedback.current_state = self._feedback.WAITING_FOR_TOGGLE
        self._as.publish_feedback(self._feedback)

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo("2024_diverter_server: preempted")
                self._as.set_preempted()
                return
            if self.limit_switch != 0:
                self._feedback.current_state = self._feedback.RUNNING_MOTOR
                self._as.publish_feedback(self._feedback)
                rospy.wait_for_service(self._command_service)
                try:
                    test_motor_command = rospy.ServiceProxy(self._command_service, Command)
                    test_motor_command(CommandRequest(command = goal.output))
                    self._result = TutorialTestResult(success=True)
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
                    self._result = TutorialTestResult(success=False)
                self._as.set_succeeded(self._result)
                return
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("tutorial_test")
    server = TutorialTestServer(rospy.get_name())
    rospy.spin()