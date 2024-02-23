#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import AlignToShooterSpeaker2024Goal, AlignToShooterSpeaker2024Result, AlignToShooterSpeaker2024Feedback, AlignToShooterSpeaker2024Action   
from behavior_actions.msg import AlignToSpeaker2024Goal, AlignToSpeaker2024Result, AlignToSpeaker2024Feedback, AlignToSpeaker2024Action
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
        self.result = AlignToShooterSpeaker2024Result()
        self.feedback = AlignToShooterSpeaker2024Feedback()


        self.align_to_speaker_client = actionlib.SimpleActionClient('/align_to_speaker/align_to_speaker_2024',AlignToSpeaker2024Action) #figure out the name for the server thingy namespace
        rospy.loginfo("align to speaker client ")
        self.align_to_speaker_client.wait_for_server()
        self.shooting_client = actionlib.SimpleActionClient('/shooting/shooting_server_2024', Shooting2024Action) #figure out waht the name or the serverthingy name space
        rospy.loginfo("shooting client")
        self.shooting_client.wait_for_server()

        rospy.loginfo("if its gotten this far then both clients are ready")

        self.dist_sub = rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, self.distance_and_angle_callback) #use this to find the distance that we are from the spaerk thing
        #insinuates that hte publisher to the topic above has to be running ^
        rospy.loginfo("if its gotten this far then the subscriber exists")

        self.server = actionlib.SimpleActionServer(self.action_name, AlignToShooterSpeaker2024Action, execute_cb=self.execute_cb, auto_start = False)
        rospy.loginfo("creating actoinlib server ")
        rospy.loginfo("starting server")
        self.server.start()
        rospy.loginfo("server started")

    def distance_and_angle_callback(data):
        global dist_value
        global speaker_angle

        dist_value = data.distance
        speaker_angle = data.angle

    def execute_cb(self, goal: AlignToShooterSpeaker2024Goal):
        global dist_value
        global speaker_angle

        r= rospy.Rate(50)
        align_to_speaker_goal = AlignToSpeaker2024Goal()
        shooting_goal = Shooting2024Goal()


        while True:
            if rospy.is_shutdown():
                break
            r.sleep()    

            if goal.mode == goal.align_and_shoot:
                

                align_to_speaker_goal.align_forever = True #sets the align to speaker thing to true? which should actually enable the stuff to move accordingly?
                self.align_to_speaker_client.send_goal(align_to_speaker_goal)
                rospy.loginfo("told align to spekaer to begin aligning")

                
                shooting_goal.distance = dist_value
                shooting_goal.distance = 0
                self.shooting_client.send_goal(shooting_goal)
                #so send the distance value to the shooting server, by sending this, it shoot activateh shooting server?
                #also sets the location of where we are trying to shoot
                rospy.loginfo("sent shooting server shooting distance value")

                self.result.success = True
                self.server.set_succeeded(self.result)
                break
                
            if goal.mode == goal.do_nothing:
                rospy.loginfo("set to do nothing")
                self.result.success = True
                self.server.set_succeeded(self.result)
                break
         
            if self.server.is_preempt_requested():
                rospy.loginfo("premept reuqrested")
                #figure out what we should be doing if we do prempt
                #maybe turn off the shooting stuff and then break the align feature
                #so stop shooting:

                align_to_speaker_goal.align_forever = False
                shooting_goal.leave_spinning = True
                rospy.loginfo("setting up states to send things in")
                self.shooting_client.send_goal(shooting_goal)
                self.align_to_speaker_client.send_goal(align_to_speaker_goal)
                rospy.loginfo("sending goals to the clients so that they respond acordingly")
                self.server.set_prempted()
                rospy.loginfo("preempted")
                break


        
    
if __name__ == '__main__':
    rospy.init_node('align_and_shoot_server_2024')
    
    server = AlignAndShoot(rospy.get_name())
    rospy.spin()