#! /usr/bin/env python3

import roslib
import rospy
import actionlib
import std_msgs.msg

from talon_state_msgs.msg import TalonFXProState
from behavior_actions.msg import Shooter2024Action, Shooter2024Goal, Shooter2024Feedback, Shooter2024Result




global left_joint_velocity
global right_joint_velocity
global left_joint_shooter_pub
global right_joint_shooter_pub



left_joint_velocity = 0.0
right_joint_velocity = 0.0



#create feedback state

def callback(data):
    #the point of this function is to callback whenever the subscriber sees someting on the talonfxpro_states
    #then, read the values on the actual motors that we've set to spin up and assign them to variables
    global left_joint_velocity
    global right_joint_velocity
    for i in range(len(data.name)):
        
        if (data.name[i] == "left_shooter_joint"): 
            left_joint_velocity = data.velocity[i]
        elif (data.name[i] == "right_shooter_joint"):
            right_joint_velocity = data.velocity[i]  
        break  
    return

class ShooterServer2024:
    _result = Shooter2024Result()
    _feedback = Shooter2024Feedback()
    def __init__(self):
        self.server = actionlib.SimpleActionServer('set_shooter_speed', Shooter2024Action, self.execute_cb, auto_start = False)
        global left_joint_shooter_pub
        global right_joint_shooter_pub

        #figure out how client stuff works
        rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, callback)
        #subscribing here so that we can figure out the actual speed of said motor at a given time, talonfxpro_states gives us these values
        

        left_joint_shooter_pub = rospy.Publisher("/frcrobot_jetson/left_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
        right_joint_shooter_pub = rospy.Publisher("/frcrobot_jetson/right_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
   
        self.server.start()
    

    def execute_cb(self, goal):
        global left_joint_velocity
        global right_joint_velocity
        global left_joint_shooter_pub
        global right_joint_shooter_pub
        
        r = rospy.Rate(50)
        #inside cb, use the passed goal values and set them to the motor values? though is this a client interaction or a server interaction?
        #like, when we send our goal to the ros thingy, like do we sned goal values thoruhg hte client or this server?
        initial_left_speed = left_joint_velocity
        initial_right_speed = right_joint_velocity


        left_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.left_shooter_speed))
        right_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.right_shooter_speed))
        #send the asked motor speeds to the topics, this should allow for the motors to begin spinning up to said speed



        #need to figure out rosepy if shtudown case and what not.
       

        current_left_speed = left_joint_velocity
        current_right_speed = right_joint_velocity

    


        if ((goal.left_shooter_speed - initial_left_speed) or (goal.right_shooter_speed - initial_right_speed)) == 0.0:
            #setfeedback state to true nad set success as the thing is alreayd hwere it needs to be
            self.server.set_succeeded(self._result)
        
        
        while (((((current_left_speed - initial_left_speed) / (goal.left_shooter_speed - initial_left_speed)) >= .9) and ((current_right_speed - initial_right_speed) / (goal.right_shooter_speed - initial_right_speed )) >= .9) != True):
            r.sleep()
            #while sleepign, publish the feedabck states, in percetnage to completion
            #self._feedback.left_percent_complete = (current_left_speed - initial_left_speed) / (goal.left_shooter_speed - initial_left_speed)
            #self._feedback.right_percent_complete = (current_right_speed - initial_right_speed) / (goal.right_shooter_speed - initial_right_speed)
            #self.server.publish_feedback(self._feedback)
            if self.server.preempt_requested():
                left_joint_shooter_pub.publish(std_msgs.Float64(current_left_speed))
                right_joint_shooter_pub.publish(std_msgs.Float64(current_right_speed))
                #preempt
            elif ((((current_left_speed - initial_left_speed) / (goal.left_shooter_speed - initial_left_speed)) >= .9) and (((current_right_speed - initial_right_speed) / (goal.right_shooter_speed - initial_right_speed ))) >= .9):
                self.server.set_succeeded(self._result)







if __name__ == '__main__':
    rospy.init_node('set_shooter_speed_server_2024')

    server = ShooterServer2024()
    rospy.spin()
