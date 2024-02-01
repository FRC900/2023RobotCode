#! /usr/bin/env python3

import roslib
import rospy
import actionlib
import std_msgs.msg
from talon_state_msgs.msg import TalonFXProState
from behavior_actions.msg import Shooter2024Action, Shooter2024Goal, Shooter2024Feedback, Shooter2024Result



#todo: Figure out how to set acutal motors to the control velocity given? Probably involves pid tuning from the inputs given to said motors
#figure out the pivot position angles etc 


global left_joint_velocity
global left_joint_control_velocity
global right_joint_velocity
global right_joint_control_velocity
global left_joint_shooter_pub
global right_joint_shooter_pub

_result = Shooter2024Result()
_feedback = Shooter2024Feedback()

left_joint_velocity = 0.0
left_joint_control_velocity = 0.0
right_joint_velocity = 0.0
right_joint_control_velocity = 0.0

def initialize_vars():
    global left_joint_velocity
    global left_joint_control_velocity
    global right_joint_velocity
    global right_joint_control_velocity
   
    left_joint_velocity = 900.0
    left_joint_control_velocity = 900.0

    right_joint_velocity = 900.0
    right_joint_control_velocity = 900.0

def callback(data):
    #the point of this function is to callback whenever the subscriber sees someting on the talonfxpro_states
    #then, read the values on the actual motors that we've set to spin up and assign them to variables
    global left_joint_velocity
    global left_joint_control_velocity
    global right_joint_velocity
    global right_joint_control_velocity
    for i in range(len(data.name)):
        if (data.name[i] == "left_joint_shooter_motor"):     #pretty sure its just the joint name
            left_joint_velocity = data.velocity[i]
            left_joint_control_velocity = data.control_velocity[i]
        elif (data.name[i] == "right_joint_shooter_motor"):
            right_joint_velocity = data.velocity[i]    #pretty sure its just hte point name as well should make these more descriptive
            right_joint_control_velocity = data.control_velocity[i]
    return

class ShooterServer2024:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('shoot_notes', Shooter2024Action, self.execute_cb, auto_start = False)
        global left_joint_shooter_pub
        global right_joint_shooter_pub

        #figure out how client stuff works
        rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, callback)
        #subscribing here so that we can figure out the actual speed of said motor at a given time, talonfxpro_states gives us these values
        #maybe we subscibe to the voltage velocity controller instead of hte fx pro

        left_joint_shooter_pub = rospy.Publisher("/frcrobot_jetson/left_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
        right_joint_shooter_pub = rospy.Publisher("/frcrobot_jetson/right_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
   
        self.server.start()

    def execute_cb(self, goal):
        global left_joint_velocity
        global left_joint_control_velocity
        global right_joint_velocity
        global right_joint_control_velocity
        global left_joint_shooter_pub
        global right_joint_shooter_pub

        #inside cb, use the passed goal values and set them to the motor values? though is this a client interaction or a server interaction?
        #like, when we send our goal to the ros thingy, like do we sned goal values thoruhg hte client or this server?

        left_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.left_shooter_speed))
        right_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.right_shooter_speed))
        #send the asked motor speeds to the topics, this should allow for the motors to begin spinning up to said speed


        if ((left_joint_velocity or left_joint_control_velocity or right_joint_velocity or right_joint_control_velocity) == 0.0):
            initialize_vars() #this is for sim since we can't really stimulate the talonfxpro_states topic

        if (((left_joint_velocity / left_joint_control_velocity)  >= .95) and ((left_joint_velocity / left_joint_control_velocity) <= 1.05)):
                if (((right_joint_velocity / right_joint_control_velocity)  >= .95) and ((right_joint_velocity / right_joint_control_velocity) <= 1.05)):
                    #need to fix this since its not even right lmao ^
                    #loginfo percent diff plus or minsu 5
                    #log speed
                    #roslog.info
                    rospy.loginfo("Shooter Velocity is within 5 percent difference of set value")
                    rospy.loginfo("Set control velocity on left: %s Set control velocity on right: %s" % (goal.left_shooter_speed, goal.right_shooter_speed))
                    rospy.loginfo("Logging left_joint_velocity: %s Logging right_joint_velocity: %s" % (left_joint_velocity,  right_joint_velocity))
                    _feedback.is_shooting_at_speed = True

                    #set success
                    #set feedback to true because velocity vs control is within error margin

                    self.server.set_succeeded(_result)


if __name__ == '__main__':
    rospy.init_node('shoot_notes_server_2024')

    server = ShooterServer2024()
    rospy.spin()
