#! /usr/bin/env python3

import roslib
import rospy
import actionlib
import std_msgs.msg
import time

from talon_state_msgs.msg import TalonFXProState
from behavior_actions.msg import ShooterPivot2024Action, ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result

_result = ShooterPivot2024Result()
_feedback = ShooterPivot2024Feedback()


#if motion magic value is ever zero it will throw an error and abort

#preempt doesnt actualyl preempt, as in, the values in sim stikll head towards the value that we want.
#ask ben or kevin about hte second issue


global shooter_pivot_pub
global motion_magic_value

#initialize motion magic value

motion_magic_value = 1.0


def callback(data):
    global motion_magic_value
    #rospy.loginfo(data.name)
    #rospy.loginfo("actualy callbasck")
    for i in range(len(data.name)):
        if (data.name[i] == "shooter_pivot_motionmagic_joint"): 
            #rospy.loginfo("reached first if condition")    #pretty sure its just the joint name
            motion_magic_value = data.position[i]
            #rospy.loginfo(left_joint_velocity)
            break

#_result = Shooter2024Result()

class ShooterPivotServer2024:
    def __init__(self):
        global shooter_pivot_pub
        self.server = actionlib.SimpleActionServer('set_shooter_pivot', ShooterPivot2024Action, self.execute_cb, auto_start = False)

        rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, callback)
        #subscribing here so that we can figure out the actual speed of said motor at a given time, talonfxpro_states gives us these values
        #maybe we subscibe to the voltage velocity controller instead of hte fx pro

        shooter_pivot_pub = rospy.Publisher("/frcrobot_jetson/shooter_pivot_motionmagicvoltage_controller/command", std_msgs.msg.Float64, queue_size=1)
   
        self.server.start()

    def execute_cb(self, goal):
        global shooter_pivot_pub
        global motion_magic_value
        r = rospy.Rate(50)

        #inside cb, use the passed goal values and set them to the motor values? though is this a client interaction or a server interaction?
        #like, when we send our goal to the ros thingy, like do we sned goal values thoruhg hte client or this server?

        initial_motion_magic_value = motion_magic_value
        rospy.loginfo("publishing pivot poisiton")
        shooter_pivot_pub.publish(std_msgs.msg.Float64(goal.pivot_position))
        rospy.loginfo("published pivot position")

        #if motionmagic value / motion magic value inputeted >= .95 or motionmagic value / mogtionmagicvalue passed <= 1.05:
        #code
        #self.server.set_succeeded(_result)
        #time.sleep(.5)
        current_motion_magic_value = motion_magic_value




        
        if ((goal.pivot_position - current_motion_magic_value) == 0.0):
            #setfeedback to be true since we are where we need to be.
            self.server.set_succeeded(_result)

        while ((((current_motion_magic_value - initial_motion_magic_value) / (goal.pivot_position - initial_motion_magic_value)) >= .6) != True):
            r.sleep()
            if self.server.is_preempt_requested():
                shooter_pivot_pub.publish(std_msgs.msg.Float64(current_motion_magic_value))
                #then prempt the stuff...
                #self.server.set_preempted()
            elif (((current_motion_magic_value - initial_motion_magic_value) / (goal.pivot_position - initial_motion_magic_value)) >= .6):
                self.server.set_succeeded(_result)
        
        #    
        #try:
        #    if ((goal.pivot_position / motion_magic_value) >= .95) and ((goal.pivot_position / motion_magic_value) <= 1.05):
#
        #        #if the inputted motion magic value and the acutal pivot position is within a 5 percent bound of error, set as successs for acitnlib
        #        #rospy.loginfo("Success")
        #        rospy.loginfo("Set pivot_position value is: %s" % goal.pivot_position)
        #        rospy.loginfo("Actual motion magic value %s" % motion_magic_value)
        #        rospy.loginfo("Motion magic asked value and motion magic actual value is within a plus or minus five difference")
        #        self.server.set_succeeded(_result)
#
#
        #
        #    else:
        #        while (((goal.pivot_position / motion_magic_value) >= .95) and ((goal.pivot_position / motion_magic_value) <= 1.05)) != True:
        #            r.sleep()
        #            if self.server.is_preempt_requested():
        #                self.server.set_preempted()
#
        #            elif ((goal.pivot_position / motion_magic_value) >= .95) and ((goal.pivot_position / motion_magic_value) <= 1.05):
        #                rospy.loginfo("Set pivot_position value is: %s" % goal.pivot_position)
        #                rospy.loginfo("Actual motion magic value %s" % motion_magic_value)
        #                rospy.loginfo("Motion magic asked value and motion magic actual value is within a plus or minus five difference")
        #                self.server.set_succeeded(_result)
        #
        #except ZeroDivisionError as e:
        #    print(e)
        #    if goal.pivot_position == 0.0:
        #        print("exception thrown due to zero edge case")
        #        self.server.set_succeeded(_result)
        #    else:
        #        print("got into this loop")
        #        while (((goal.pivot_position / motion_magic_value) >= .95) and ((goal.pivot_position / motion_magic_value) <= 1.05)) != True:
        #            r.sleep()
        #            if self.server.is_preempt_requested():
        #                self.server.set_preempted()
        #            elif ((goal.pivot_position / motion_magic_value) >= .95) and ((goal.pivot_position / motion_magic_value) <= 1.05):
        #                rospy.loginfo("Set pivot_position value is: %s" % goal.pivot_position)
        #                rospy.loginfo("Actual motion magic value %s" % motion_magic_value)
        #                rospy.loginfo("Motion magic asked value and motion magic actual value is within a plus or minus five difference")
        #                self.server.set_succeeded(_result)
#
        ##figure out how to create a loop that sleeps while hte condition above is not true



if __name__ == '__main__':
    rospy.init_node('shooter_pivot_server_2024')

    server = ShooterPivotServer2024()
    rospy.spin()
