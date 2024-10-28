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

class TestServer:
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber('/frcrobot_jetson/joint_states', JointState, self.joint_state_callback)
        self.test_motor_client = rospy.ServiceProxy('/frcrobot_jetson/test_motor_controller/command', Command)
        self.limit_switch_state = False

        self.server = actionlib.SimpleActionServer('tutorial_test', TutorialTestAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

    def joint_state_callback(self, msg):
        self.limit_switch_state = find_joint_state(msg, 'diverter_limit_switch') > 0.0

    def turn_off_motor(self):
        command = CommandRequest()
        command.command = 0.0
        response = self.test_motor_client(command)

        # Check if the motor ran successfully
        return response.success

    def execute_cb(self, goal):
        # This callback is called when the action server is called
        # The goal is the input to the action server
        # The result is what the action server returns
        # The feedback is what the action server sends back to the client during execution

        # This is the feedback message that we'll be sending back to the client
        feedback = TutorialTestFeedback()
        feedback.current_state = feedback.WAITING_FOR_TOGGLE

        self.server.publish_feedback(feedback)

        # This is the result message that we'll be sending back to the client
        result = TutorialTestResult()

        # Check if the limit switch is toggled
        r = rospy.Rate(100)
        initial_state = self.limit_switch_state
        while initial_state == self.limit_switch_state:
            r.sleep()
            if self.server.is_preempt_requested():
                result.success = False
                self.server.set_preempted(result)
                return

        # Update the feedback message
        feedback.current_state = feedback.RUNNING_MOTOR

        self.server.publish_feedback(feedback)

        # Run the motor at the given output
        command = CommandRequest()
        command.command = goal.output
        response = self.test_motor_client(command)

        # Check if the motor ran successfully
        if response.success:
            result.success = True
        else:
            result.success = False

        # Wait ten seconds while checking for preempt, then turn off the motor
        start = rospy.Time.now()
        while (rospy.Time.now() - start) < rospy.Duration(10):
            if self.server.is_preempt_requested():
                self.turn_off_motor()
                result.success = False
                self.server.set_preempted(result)
                return

        result.success = self.turn_off_motor()

        # Send the result back to the client
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('tutorial_test_server', anonymous=True)
    server = TestServer()
    rospy.spin()