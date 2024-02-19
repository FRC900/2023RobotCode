#!/usr/bin/env python3

import rospy
import actionlib

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

from behavior_actions.msg import Intaking2024Goal, Intaking2024Feedback, Intaking2024Result, Intaking2024Action


from behavior_actions.msg import NoteDiverterGoal, NoteDiverterFeedback, NoteDiverterResult, NoteDiverterAction
from behavior_actions.msg import Claw2024Goal, Claw2024Feedback, Claw2024Result, Claw2024Action
#^ will be claswster
from behavior_actions.msg import Arm2024Goal, Arm2024Feedback, Arm2024Result, Arm2024Action
from behavior_actions.msg import Claw2024Goal, Claw2024Feedback, Claw2024Result, Claw2024Action
#^ preshooter idk???


from std_msgs.msg import Float64

class IntakingServer(object):
    def __init__(self, name):
        self.action_name = name
        self.result = Intaking2024Result()
        self.feedback = Intaking2024Feedback()
        self.intaker_client = actionlib.SimpleActionClient('/intaker/intaker_server_2024', Intaking2024Action)
        # The preshooter and claw are very similar (drive motor until limit switch pressed). They'll probably be the same server.
        self.preshooter_client = actionlib.SimpleActionClient('/preshooter/preshooter_server_2024', Claw2024Action)
        self.diverter_client = actionlib.SimpleActionClient('/diverter/diverter_server_2024', NoteDiverterAction)
        self.arm_client = actionlib.SimpleActionClient('/arm/arm_server_2024', Arm2024Action)


        # speeds_map: [[distance: [left_speed, right_speed]], ...]
        speeds_map_param = rospy.get_param("speeds_map")

        self.server = actionlib.SimpleActionServer(self.action_name, Intaking2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

    def execute_cb(self, goal: Intaking2024Goal):

        self.feedback.current_stage = self.feedback.CLAW 

        
        if not goal.setup_only:
            self.feedback.current_stage = self.feedback.SHOOTING
            self.server.publish_feedback(self.feedback)

            preshooter_goal = Claw2024Goal()
            preshooter_goal.mode = preshooter_goal.OUTTAKE_CLAW

            self.preshooter_client.send_goal(preshooter_goal)

            self.preshooter_client.wait_for_result()

            rospy.loginfo("2024_shooting_server: +5 points hopefully")

        self.result.success = True
        rospy.loginfo("2024_shooting_server: succeeded")
        self.server.set_succeeded(self.result)

def dyn_rec_callback(config, level):
    rospy.loginfo("Received reconf call: " + str(config))
    global conveyor_speed
    global diverter_speed
    conveyor_speed = config["conveyor_speed"]
    diverter_speed = config["diverter_speed"]
    return config

if __name__ == '__main__':
    rospy.init_node('diverter_server_2024')

    # ddynrec = DDynamicReconfigure("diverter_dyn_rec")
    # ddynrec.add_variable("conveyor_speed", "float/double variable", rospy.get_param("note_conveyor_speed"), 0.0, 13.0)
    # ddynrec.add_variable("diverter_speed", "float/double variable", rospy.get_param("note_diverter_speed"), 0.0, 13.0)
    # ddynrec.start(dyn_rec_callback)

    server = IntakingServer(rospy.get_name())
    rospy.spin()