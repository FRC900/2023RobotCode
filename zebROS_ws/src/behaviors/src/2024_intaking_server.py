#! /usr/bin/env python3

import rospy
from behavior_actions.msg import intaking_2024Action, intaking_2024Goal, intaking_2024Feedback, intaking_2024Result
from behavior_actions.msg import NoteDiverterAction, NoteDiverterFeedback, NoteDiverterResult, NoteDiverterGoal
from behavior_actions.msg import Claw2024Action, Claw2024Feedback, Claw2024Result, Claw2024Goal
from behavior_actions.msg import Arm2024Action, Arm2024Feedback, Arm2024Result, Arm2024Goal

# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64
class intaking2024_server(object):
     # create messages that are used to publish feedback/result
     _feedback = intaking_2024Feedback()
     _result = intaking_2024Result()

     def __init__(self, name):
          self._action_name = name
          self._as = actionlib.SimpleActionServer(self._action_name, intaking_2024Action, execute_cb=self.execute_cb, auto_start = False)
          self._as.start()
          self.arm_client = actionlib.SimpleActionClient('arm_server_2024', Arm2024Action)
          print('waiting for arm server')
          self.arm_client.wait_for_server()
          self.diverter_client = actionlib.SimpleActionClient('diverter_server_2024', NoteDiverterAction)     
          print('waiting for diverter server')
          self.diverter_client.wait_for_server()
          self.claw_client = actionlib.SimpleActionClient('claw_server_2024', Claw2024Action)     
          print('waiting for claw server')
          self.claw_client.wait_for_server()
          self.intake_pub = rospy.Publisher("/frcrobot_jetson/note_intake_controller/command", Float64, queue_size=1)
          self.preshooter_client = actionlib.SimpleActionClient('preshooter_server_2024', Claw2024Action)     
          print('waiting for preshooter server')
          # self.preshooter_client.wait_for_server()


     def execute_cb(self, goal):
          if goal.destination == goal.SHOOTER:
               a = NoteDiverterGoal.TO_SHOOTER 
          elif goal.destination == goal.CLAW:
               a = NoteDiverterGoal.TO_CLAW
          goal = NoteDiverterGoal(mode=a)
          self.diverter_client.send_goal(goal)
          if goal.destination == goal.SHOOTER:
               a = 2
               claw_goal = Claw2024Goal(mode=Claw2024Goal.INTAKE_CLAW)
               #self.preshooter_client.send_goal(claw_goal)
          elif goal.destination == goal.CLAW:
               claw_goal = Claw2024Goal(mode=Claw2024Goal.INTAKE_CLAW)
               self.claw_client.send_goal(claw_goal)
          intake_message = Float64()
          intake_message.data = a 
          self.intake_pub.publish(intake_message)
          while not rospy.is_shutdown():
               d = rospy.Duration(.05)
               claw_wait_result = self.claw_client.wait_for_result(d)
               diverter_wait_result = self.diverter_client.wait_for_result(d)
               if claw_wait_result == True and diverter_wait_result == True:
                    break
               if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    success = False
                    break
                    #preempted both the claw and diverter

          claw_result = self.claw_client.get_result()
          diverter_result = self.diverter_client.get_result()
          #rate = rospy.Rate()

          # while not rospy.is_shutdown():
          #     if self._as.is_preempt_requested():
          #         rospy.loginfo('%s: Preempted' % self._action_name)
          #         self._as.set_preempted()
          #         success = False

          if success:
               self._result.sequence = self._feedback.sequence
               rospy.loginfo('%s: Succeeded' % self._action_name)
               self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('intaking_2024')
    server = intaking2024_server(rospy.get_name())
    rospy.spin()