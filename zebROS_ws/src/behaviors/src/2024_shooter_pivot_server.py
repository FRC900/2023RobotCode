#! /usr/bin/env python3

import roslib
import rospy
import actionlib
import std_msgs.msg

from talon_state_msgs.msg import TalonFXProState
from behavior_actions.msg import ShooterPivot2024Action, ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result


_result = ShooterPivot2024Result()
_feedback = ShooterPivot2024Feedback()

global shooter_pivot_pub
global motion_magic_value

#initialize motion magic value

motion_magic_value = 100.0

def callback(data):
    global motion_magic_value
    #rospy.loginfo(data.name)
    #rospy.loginfo("actualy callbasck")
    for i in range(len(data.name)):
        if (data.name[i] == "shooter_pivot_motionmagic_joint"): 
            #rospy.loginfo("reached first if condition")    #pretty sure its just the joint name
            motion_magic_value = data.position[i]
            #rospy.loginfo(left_joint_velocity)

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
        #inside cb, use the passed goal values and set them to the motor values? though is this a client interaction or a server interaction?
        #like, when we send our goal to the ros thingy, like do we sned goal values thoruhg hte client or this server?

        rospy.loginfo("publishing pivot poisiton")
        shooter_pivot_pub.publish(std_msgs.msg.Float64(goal.pivot_position))
        rospy.loginfo("published pivot position")

        #if motionmagic value / motion magic value inputeted >= .95 or motionmagic value / mogtionmagicvalue passed <= 1.05:
        #code
        #self.server.set_succeeded(_result)

        if ((goal.pivot_position / motion_magic_value) >= .95) and ((goal.pivot_position / motion_magic_value) <= 1.05):

            #if the inputted motion magic value and the acutal pivot position is within a 5 percent bound of error, set as successs for acitnlib
            #rospy.loginfo("Success")
            rospy.loginfo("Set pivot_position value is: %s" % goal.pivot_position)
            rospy.loginfo("Actual motion magic value %s" % motion_magic_value)
            rospy.loginfo("Motion magic asked value and motion magic actual value is within a plus or minus five difference")
            self.server.set_succeeded(_result)


if __name__ == '__main__':
    rospy.init_node('shooter_pivot_server_2024')

    server = ShooterPivotServer2024()
    rospy.spin()
