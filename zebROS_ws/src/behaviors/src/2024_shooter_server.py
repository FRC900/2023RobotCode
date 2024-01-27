#! /usr/bin/env python
# not the actual shooter controller, but probably the fastest way to get running

import roslib
import rospy
import actionlib
import std_msgs.msg
from talon_state_msgs.msg import TalonFXProState
from behavior_actions.msg import Shooter2024Action, Shooter2024Goal, Shooter2024Feedback


global left_joint_velocity
global left_joint_control_velocity
global right_joint_velocity
global right_joint_control_velocty

_goal = Shooter2024Goal()
_feedback = Shooter2024Feedback()



def callback(data):
    global left_joint_velocity
    global left_joint_control_velocity
    global right_joint_velocity
    global right_joint_control_velocty
    for i in range(len(data.name)):
        if (data.name[i] == "left_joint"):
            left_joint_velocity = data.velocity[i]
            left_joint_control_velocity = data.control_velocity[i]
        elif (data.name[i] == "right_joint"):
            right_joint_velocity = data.velocity[i]
            right_joint_control_velocty = data.control_velocity[i]
    return


class ShooterServer2024:
    def __init__(self):


        

        self.server = actionlib.SimpleActionServer('shoot_notes', Shooter2024Action, self.execute_cb, auto_start = False)
        #figure out how client stuff works
        rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, callback)


        
        self.server.start()

    def execute_cb(self, goal):
        global left_joint_velocity
        global left_joint_control_velocity
        global right_joint_velocity
        global right_joint_control_velocty

        #values are taken from the callback function of the subscriber
        #_goal.left_shooter_speed
        #_goal.right_shooter_speed
        
        
        if (((left_joint_velocity / left_joint_control_velocity)  >= .95) and ((right_joint_velocity / right_joint_control_velocty) >= 1.05)):
            #loginfo percent diff plus or minsu 5
            #log speed
            #roslog.info
            rospy.loginfo("Shooter Velocity is within 5 percent difference of set value")
            rospy.loginfo("Set control velocity on left: %s Set control velocity on right: %s" % (_goal.left_shooter_speed, _goal.right_shooter_speed))
            rospy.loginfo("Logging left_joint_velocity: %s Logging right_joint_velocity: %s" % (left_joint_velocity,  right_joint_velocity))
            _feedback.is_shooting_at_speed = True


            #set feedback to true because velocity vs control is within error margin







    
if __name__ == '__main__':
    rospy.init_node('shoot_notes_server_2024')
    
    server = ShooterServer2024()
    rospy.spin()

