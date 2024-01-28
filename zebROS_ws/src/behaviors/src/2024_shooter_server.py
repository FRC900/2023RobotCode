#! /usr/bin/env python

import roslib
import rospy
import actionlib
import std_msgs.msg
from talon_state_msgs.msg import TalonFXProState
from behavior_actions.msg import Shooter2024Action, Shooter2024Goal, Shooter2024Feedback



#todo: Figure out how to set acutal motors to the control velocity given? Probably involves pid tuning from the inputs given to said motors
#figure out the pivot position angles etc 


global left_joint_velocity
global left_joint_control_velocity
global right_joint_velocity
global right_joint_control_velocty
global left_joint_shooter_pub
global right_joint_shooter_pub

_goal = Shooter2024Goal()
_feedback = Shooter2024Feedback()



def callback(data):
    global left_joint_velocity
    global left_joint_control_velocity
    global right_joint_velocity
    global right_joint_control_velocty
    for i in range(len(data.name)):
        if (data.name[i] == "left_joint_shooter_motor"):     #pretty sure its just the joint name
            left_joint_velocity = data.velocity[i]
            left_joint_control_velocity = data.control_velocity[i]
        elif (data.name[i] == "right_joint_shooter_motor"):
            right_joint_velocity = data.velocity[i]    #pretty sure its just hte point name as well should make these more descriptive
            right_joint_control_velocty = data.control_velocity[i]
    return


class ShooterServer2024:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('shoot_notes', Shooter2024Action, self.execute_cb, auto_start = False)
        global left_joint_shooter_pub
        global right_joint_shooter_pub

        #figure out how client stuff works
        rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, callback)
        left_joint_shooter_pub = rospy.Publisher("/frcrobot_jetson/left_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
        right_joint_shooter_pub = rospy.Publisher("/frcrobot_jetson/right_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
        #TALON_SHOT
        #rospy.loginfo(f"Left speed {left_speed}, Right speed {right_speed}, percent {percent}")
        #_goal.left_shooter_speed
        #_goal.right_shooter_speed
        # So setting the left and right publishing...    
        # left_joint_shooter_pub.publish(std_msgs.msg.Float64(_goal.left_shooter_speed))
        # right_joint_shooter_pub.publish(std_msgs.msg.FLoat64(_goal.right_shooter_speed))
        #
        #left_pub.publish(std_msgs.msg.Float64(left_speed))
        #right_pub.publish(std_msgs.msg.Float64(right_speed))

        


        #TALON_SHOT




        #left_joint_shooter_motor = rospy.Publisher("/frcrobot_jetson/left_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
        #right_joint_shooter_motor = rospy.Publisher("/frcrobot_jetson/right_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)


        
        self.server.start()

    def execute_cb(self, goal):
        global left_joint_velocity
        global left_joint_control_velocity
        global right_joint_velocity
        global right_joint_control_velocty
        global left_joint_shooter_pub
        global right_joint_shooter_pub

        #values are taken from the callback function of the subscriber
        #_goal.left_shooter_speed
        #_goal.right_shooter_speed
        
        #inside cb, use the passed goal values and set them to the motor values? though is this a client interaction or a server interaction?
        #like, when we send our goal to the ros thingy, like do we sned goal values thoruhg hte client or this server?

        left_joint_shooter_pub.publish(std_msgs.msg.Float64(_goal.left_shooter_speed))
        right_joint_shooter_pub.publish(std_msgs.msg.Float64(_goal.right_shooter_speed))
        
        
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

