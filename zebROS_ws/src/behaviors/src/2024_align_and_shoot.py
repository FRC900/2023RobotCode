#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import AlignAndShoot2024Goal, AlignAndShoot2024Result, AlignAndShoot2024Feedback, AlignAndShoot2024Action   
from behavior_actions.msg import AlignToSpeaker2024Goal, AlignToSpeaker2024Result, AlignToSpeaker2024Feedback, AlignToSpeaker2024Action
from behavior_actions.msg import Shooting2024Goal, Shooting2024Feedback, Shooting2024Result, Shooting2024Action
from behavior_actions.msg import AutoAlignSpeaker
import behavior_actions.msg
import std_srvs.srv

#from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from std_msgs.msg import Float64
from std_msgs.msg import Header

class AlignAndShoot:
    def __init__(self, name):
        self.action_name = name
        self.result = AlignAndShoot2024Result()
        self.feedback = AlignAndShoot2024Feedback()

        self.align_to_speaker_client = actionlib.SimpleActionClient('/align_to_speaker/align_to_speaker_2024',AlignToSpeaker2024Action) #figure out the name for the server thingy namespace
        rospy.loginfo("2024_align_and_shoot: waiting for align to speaker server")
        self.align_to_speaker_client.wait_for_server()
        self.shooting_client = actionlib.SimpleActionClient('/shooting/shooting_server_2024', Shooting2024Action) #figure out waht the name or the serverthingy name space
        rospy.loginfo("2024_align_and_shoot: waiting for shooting server")
        # self.shooting_client.wait_for_server()

        self.dist_sub = rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, self.distance_and_angle_callback, tcp_nodelay=True, queue_size=1) #use this to find the distance that we are from the spaerk thing
        if self.dist_sub.get_num_connections() > 0: rospy.loginfo("2024_align_and_shoot: distance and angle topic being published to")
        
        self.server = actionlib.SimpleActionServer(self.action_name, AlignAndShoot2024Action, execute_cb=self.execute_cb, auto_start = False)
        rospy.loginfo("2024_align_and_shoot: starting server")
        
        self.last_relocalize_sub = rospy.Subscriber("/last_relocalize", Header, self.relocalized_cb, tcp_nodelay=True, queue_size=1)

        self.last_relocalized = rospy.Time()
        self.align_to_speaker_done = False
        self.half_field_timer = rospy.Timer(rospy.Duration(1.0/5.0), self.half_field_timer_cb)

        self.server.start()
        rospy.loginfo("2024_align_and_shoot: server started")

    def relocalized_cb(self, msg: Header):
        rospy.loginfo_throttle(0.5, "2024_align_and_shoot : relocalized")
        self.last_relocalized = msg.stamp

    def distance_and_angle_callback(self, msg: AutoAlignSpeaker):
        self.dist_value = msg.distance
    
    def align_to_speaker_feedback_cb(self, feedback: AlignToSpeaker2024Feedback):
        self.align_to_speaker_done = feedback.aligned
        rospy.loginfo("2024 Align and shoot: Align to speaker")
    
    def shooting_done_cb(self, result: Shooting2024Result):
        self.shooting_done = result.success
        rospy.logwarn("2024 Align and shoot: Shooting done!")

    def half_field_timer_cb(self, event):
        self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())        
        shooting_goal = Shooting2024Goal()
        shooting_goal.mode = shooting_goal.SPEAKER # should use the dist and angle topic to keep adjusting speeds
        shooting_goal.distance = self.dist_value #sets the dist value for goal ditsance with resepct ot hte calblack
        shooting_goal.setup_only = True
        shooting_goal.leave_spinning = True
        self.shooting_client.send_goal(shooting_goal, done_cb=self.shooting_done_cb)

    def execute_cb(self, goal: AlignAndShoot2024Goal):
        r = rospy.Rate(20)
        align_to_speaker_goal = AlignToSpeaker2024Goal()
        shooting_goal = Shooting2024Goal()

        rospy.loginfo("2024_align_and_shoot: spinning up shooter")

        align_to_speaker_goal.align_forever = goal.align_forever
        align_to_speaker_goal.offsetting = goal.offsetting

        self.align_to_speaker_client.send_goal(align_to_speaker_goal, feedback_cb=self.align_to_speaker_feedback_cb)
        rospy.loginfo("2024_align_and_shoot: align goal sent")

        while not self.align_to_speaker_done and not self.shooting_done and not rospy.is_shutdown():
            rospy.loginfo_throttle(0.5, "2024_align_and_shoot: aligning")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_align_and_shoot: preempted")
                self.align_to_speaker_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                return
            r.sleep()

        rospy.loginfo("2024_align_and_shoot: done aligning")
        
        shooting_goal.mode = shooting_goal.SPEAKER
        shooting_goal.distance = self.dist_value #sets the dist value for goal ditsance with resepct ot hte calblack
        shooting_goal.setup_only = False
        shooting_goal.leave_spinning = goal.leave_spinning
        shooting_done = False

        def shooting_done_cb(state, result):
            nonlocal shooting_done
            shooting_done = True
        self.shooting_client.send_goal(shooting_goal, done_cb=shooting_done_cb)
        rospy.loginfo("2024_align_and_shoot: sent shooting client goal")

        while not shooting_done and not rospy.is_shutdown():
            rospy.loginfo_throttle(0.5, "2024_align_and_shoot: waiting for shooting server to finish")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_align_and_shoot: preempted")
                self.align_to_speaker_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                return
            r.sleep()

        rospy.loginfo("2024_align_and_shoot: succeeded")
        self.result.success = True
        self.server.set_succeeded(self.result)
        self.

if __name__ == '__main__':
    rospy.init_node('align_and_shoot_2024')
    
    server = AlignAndShoot(rospy.get_name())
    rospy.spin()