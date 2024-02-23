#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import AlignToShooterSpeakerGoal, AlignToShooterSpeakerResult, AlignToShooterSpeakerFeedback, AlignToShooterSpeakerAction   
from behavior_actions.msg import AlignToSpeaker2024Goal, AlignToSpeakerResult, AlignToSpeakerFeedback, AlignToSpeakerAction
from behavior_actions.msg import Shooting2024Goal, Shooting2024Feedback, Shooting2024Result, Shooting2024Action
from behavior_actions.msg import AutoAlignSpeaker

#from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from std_msgs.msg import Float64



global dist_value
global speaker_angle

class AlignAndShoot(object):
    global dist_value
    global speaker_angle

    def __init__(self, name):
        self.action_name = name
        self.result = AlignToShooterSpeakerResult()
        self.feedback = AlignToShooterSpeakerFeedback()


        self.align_to_speaker_client = actionlib.SimpleActionClient('',AlignToSpeakerAction) #figure out the name for the server thingy namespace
        rospy.loginfo("align to speaker client ")
        self.align_to_speaker_client.wait_for_server()
        self.shooting_client = actionlib.SimpleActionClient('', Shooting2024Action)
        rospy.loginfo("shooting client")
        self.shooting_client.wait_for_server()

        self.dist_sub = rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, distance_and_angle_callback) #use this to find the distance that we are from the spaerk thing

        self.server = actionlib.SimpleActionServer(self.action_name, AlignToShooterSpeakerAction, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

    def execute_cb(self, goal: AlignToShooterSpeakerGoal):
        global dist_value
        global speaker_angle


        if goal.mode == goal.align_and_shoot:
            #self.server.publish_feedback(self.feedback)

            #AlignToShooterSpeaker = AlignToShooterSpeakerGoal()


            shooting_goal = Shooting2024Goal()
            shooting_goal.distance = dist_value
            self.shooting_client.send_goal(shooting_goal)
            #so send the distance value to the shooting server
            rospy.loginfo("sent shooting server shooting distance value")


            align_to_speaker_goal = AlignToSpeaker2024Goal()
            align_to_speaker_goal.align_forever = True #sets the align to speaker thing to true? which should actually enable the stuff to move accordingly?
            self.align_to_speaker_client.send_goal(align_to_speaker_goal)
            rospy.loginfo("told align to spekaer to begin aligning")
            self.result.success = True
            self.server.set_succeeded(self.result.success)


        if goal.mode == goal.do_nothing:
            rospy.loginfo("set to do nothing")
            self.result.success = True
            self.server.set_succeeded(self.result.success)
        #AlignToSpeaker.distance = dist_value
        #AlignToSpeaker.angle = speaker_angle
        #not sure what i'm doing here tbh
        #given the dist value, send this to the shooting server
        if self.server.is_preempt_requested():
            rospy.loginfo("premept reuqrested")



        


def distance_and_angle_callback(data):
    global dist_value
    global speaker_angle

    #data.distance
    #data.angle
    dist_value = data.distance
    speaker_angle = data.angle


    
if __name__ == '__main__':
    rospy.init_node('align_and_shoot_server')
    
    server = AlignAndShoot(rospy.get_name())
    rospy.spin()