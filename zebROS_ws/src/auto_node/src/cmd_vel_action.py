import rospy
import actionlib
from action import Action
from datetime import datetime
from typing import List
from subsystem import Subsystem
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result
from geometry_msgs.msg import Twist

class CmdVelAction(Action):
    """Publishes the given Twist to the swerve topic for the specified time"""
    def __init__(self, twist: Twist, time: float):        
        self.__cmd_vel_pub_ = rospy.Publisher("/auto/cmd_vel", Twist, tcp_nodelay=False)
        self.__finished = False
        self.__run_time: float = time
        self.__twist: Twist = twist
        self.start_time: rospy.Time = rospy.Time(0)

    def start(self):
        rospy.loginfo(f"Starting Cmd Vel action with twist {self.__twist}")
        self.start_time = rospy.Time.now()
        self.__cmd_vel_pub_.publish(self.__twist)


    def update(self):
        if rospy.Time.now() - self.start_time > rospy.Duration(self.__run_time):
            rospy.loginfo_throttle(1, "Cmd Vel action finished")
            self.__finished = True
        else:
            self.__cmd_vel_pub_.publish(self.__twist)
            rospy.loginfo_throttle(1, "Still running cmd_vel action!")

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__finished

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.INTAKE ]
