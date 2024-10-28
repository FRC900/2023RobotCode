import rospy
import actionlib
from action import Action
from datetime import datetime
from typing import List
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result

class WaitIntakeAction(Action):
    """Turns on the intake forever"""
    def __init__(self):        
        self.__intake_client = actionlib.SimpleActionClient("/intaking/intaking_server_2024", Intaking2024Action)

    def intake_feedback(self, msg: Intaking2024Feedback):
        pass

    def start(self):
        rospy.loginfo("Turning on intake in auto mode!")
        intake_action = Intaking2024Goal()
        intake_action.run_until_preempt = True
        intake_action.destination = intake_action.SHOOTER
        self.__intake_client.send_goal(intake_action, feedback_cb=self.intake_feedback)

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__finished

    def preempt(self):
        rospy.logwarn("Preempt called for intake action, stopping intake")
        self.__intake_client.cancel_goals_at_and_before_time(rospy.Time.now())
        self.__finished = True
