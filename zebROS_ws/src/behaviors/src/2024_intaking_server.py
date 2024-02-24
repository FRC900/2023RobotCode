#! /usr/bin/env python3

import rospy
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result
from behavior_actions.msg import NoteDiverterAction, NoteDiverterFeedback, NoteDiverterResult, NoteDiverterGoal
from behavior_actions.msg import Clawster2024Action, Clawster2024Feedback, Clawster2024Result, Clawster2024Goal
from behavior_actions.msg import Arm2024Action, Arm2024Feedback, Arm2024Result, Arm2024Goal
from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse

# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64
class Intaking2024Server(object):
    # create messages that are used to publish feedback/result
    feedback = Intaking2024Feedback()
    result = Intaking2024Result()

    def __init__(self, name):
        self.action_name = name
        
        #self.arm_client = actionlib.SimpleActionClient('/arm/move_arm_server_2024', Arm2024Action)
        #rospy.loginfo("2024_intaking_server: waiting for arm server")
        #self.arm_client.wait_for_server()
        rospy.logerr("ARM CLIENT NOT INITALIZED BECAUSE IT DOESN'T EXIST")

        self.diverter_client = actionlib.SimpleActionClient('/diverter/diverter_server_2024', NoteDiverterAction)
        rospy.loginfo("2024_intaking_server: waiting for diverter server")
        self.diverter_client.wait_for_server()
        self.clawster_client = actionlib.SimpleActionClient('/clawster/clawster_server_2024', Clawster2024Action)
        rospy.loginfo("2024_intaking_server: waiting for clawster server")
        self.clawster_client.wait_for_server()
        rospy.loginfo(f"Clawster client {self.clawster_client}")

        self.intake_client = rospy.ServiceProxy("/frcrobot_jetson/intake_talonfxpro_controller/command", Command)

        self.intaking_speed = rospy.get_param("intaking_speed")
        self.intaking_timeout = rospy.get_param("intaking_timeout")

        self.server = actionlib.SimpleActionServer(self.action_name, Intaking2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()
        
    def execute_cb(self, goal: Intaking2024Goal):
        r = rospy.Rate(60)

        self.feedback.state = self.feedback.DIVERTING
        self.server.publish_feedback(self.feedback)

        diverter_goal = NoteDiverterGoal()
        if goal.destination == goal.SHOOTER:
            diverter_goal.mode = diverter_goal.TO_SHOOTER 
        elif goal.destination == goal.CLAW:
            diverter_goal.mode = diverter_goal.TO_CLAW
        rospy.loginfo("2024_intaking_server: running diverter")
        self.diverter_client.send_goal(diverter_goal)

        self.feedback.state = self.feedback.CLAWSTERING
        self.server.publish_feedback(self.feedback)

        clawster_goal = Clawster2024Goal()
        clawster_goal.mode = clawster_goal.INTAKE
        clawster_done = False
        clawster_result: Clawster2024Result = None
        def clawster_done_cb(state, result):
            nonlocal clawster_done, clawster_result
            clawster_done = True
            clawster_result = result
        
        if goal.destination == goal.SHOOTER:
            clawster_goal.destination = clawster_goal.PRESHOOTER
            rospy.loginfo("2024_intaking_server: running preshooter")
        elif goal.destination == goal.CLAW:
            rospy.loginfo("2024_intaking_server: running claw")
            clawster_goal.destination = clawster_goal.CLAW

        self.clawster_client.send_goal(clawster_goal, done_cb=clawster_done_cb)

        self.feedback.state = self.feedback.INTAKING
        self.server.publish_feedback(self.feedback)

        intake_srv = CommandRequest()
        intake_srv.command = self.intaking_speed
        self.intake_client.call(intake_srv)

        start = rospy.Time()
        # if run until preempt want to just go for the entire auto
        while goal.run_until_preempt or (not (clawster_done or rospy.is_shutdown() or (rospy.Time() - start).to_sec() > self.intaking_timeout)):
            rospy.loginfo_throttle(0.5, f"2024_intaking_server: waiting for {'preshooter' if goal.destination == goal.SHOOTER else 'claw'}")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_intaking_server: preempted")

                # stop intake
                intake_srv = CommandRequest()
                intake_srv.command = 0.0
                self.intake_client.call(intake_srv)

                # stop diverter
                self.diverter_client.cancel_goals_at_and_before_time(rospy.Time())
                self.clawster_client.cancel_goals_at_and_before_time(rospy.Time())
                # stop preshooter and claw
                self.server.set_preempted()
                return
            r.sleep()
        
        # stop intake
        intake_srv = CommandRequest()
        intake_srv.command = 0.0
        self.intake_client.call(intake_srv)

        # stop diverter
        self.diverter_client.cancel_goals_at_and_before_time(rospy.Time())
        # stop preshooter and claw
        self.clawster_client.cancel_goals_at_and_before_time(rospy.Time())

        if clawster_done:
            if clawster_result.has_game_piece:
                rospy.loginfo("2024_intaking_server: succeeded")
                self.result.success = True
                self.server.set_succeeded(self.result)
                return
            else:
                rospy.loginfo("2024_intaking_server: clawster done, but no game piece??")
                self.result.success = False
                self.server.set_succeeded(self.result)
                return

        if (rospy.Time() - start) > self.intaking_timeout:
            rospy.loginfo("2024_intaking_server: timed out")
            self.result.success = False
            self.server.set_succeeded(self.result)
            return

        rospy.loginfo(f"2024_intaking_server: we haven't succeeded, so we've failed. is rospy shutdown? {rospy.is_shutdown()}")
        self.result.success = False
        self.server.set_succeeded(self.result)
        

       
if __name__ == '__main__':
    rospy.init_node('intaking_server_2024')
    server = Intaking2024Server(rospy.get_name())
    rospy.spin()