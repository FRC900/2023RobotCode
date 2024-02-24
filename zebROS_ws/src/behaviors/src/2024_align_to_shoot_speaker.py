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

    def distance_and_angle_callback(self, data):
        global dist_value
        global speaker_angle

        dist_value = data.distance
        speaker_angle = data.angle

    def execute_cb(self):
        global dist_value
        global speaker_angle

        r = rospy.Rate(50)
        align_to_speaker_goal = AlignToSpeaker2024Goal()
        shooting_goal = Shooting2024Goal()

        align_to_speaker_goal.align_forever = False #sets the align to speaker thing to true? which should actually enable the stuff to move accordingly?
        align_to_speaker_done = False
        rospy.loginfo("set align thing to align")

        def align_to_speaker_done_cb(state, result):
            nonlocal align_to_speaker_done
            align_to_speaker_done = True
        self.align_to_speaker_client.send_goal(align_to_speaker_goal, done_cb=align_to_speaker_done_cb)
        rospy.loginfo("told align to spekaer to begin aligning")

        while (not (align_to_speaker_done) and not rospy.is_shutdown()):
            rospy.loginfo_throttle(0.5, "2024_align_and_shoot: aligning")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024 align and shoot server preempted")
                self.align_to_speaker_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_prempted()
                return
            r.sleep()

        shooting_goal.mode = shooting_goal.SPEAKER
        shooting_goal.distance = dist_value #sets the dist value for goal ditsance with resepct ot hte calblack
        rospy.loginfo("set distance value")
        shooting_send_done = False

        def shooting_send_done_cb(state, result):
            nonlocal shooting_send_done
            shooting_send_done = True
        self.shooting_client.send_goal(shooting_goal, done_cb=shooting_send_done_cb)
        rospy.loginfo("told shooting client to begin shooting or spinning or something")

        while (not (shooting_send_done) and not rospy.is_shutdown()):
            rospy.loginfo_throttle(0.5, "2024_align_and_shoot: waiting for shooting server to finish")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024 align and shoot server preempted")
                self.align_to_speaker_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_prempted()
                return
            r.sleep()

        rospy.loginfo("succeeded")
        self.result.success = True
        self.server.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('align_and_shoot_server_2024')
    
    server = AlignAndShoot(rospy.get_name())
    rospy.spin()