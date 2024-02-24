#!/usr/bin/env python3
# Imports
# Note (DELETE LATER) The reason I put ask Ben for all of these is because this is his code lol, I just copied it and adjusting it for the purposes of this case. You could probably ask Kevin and it would still work fine, but I feel like Ben would be easier because he already knows whats going on.
import rospy # This is ros, you should know this
import actionlib # For action servers

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure # No clue, self-reminder to ASK ben bc i CANNOT get this into my head for some reason

from behavior_actions.msg import SubShooter2024Goal, SubShooter2024Feedback, SubShooter2024Result, SubShooter2024Action # Get all of the stuff for this server
from behavior_actions.msg import Shooter2024Goal, Shooter2024Feedback, Shooter2024Result, Shooter2024Action # Controls the shooter
from behavior_actions.msg import ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result, ShooterPivot2024Action # Controls the pivot position of the shooter
from behavior_actions.msg import Clawster2024Goal, Clawster2024Feedback, Clawster2024Result, Clawster2024Action # Since both the claw and clawster use a limit switch (No clue what that is btw), they'll use the same server
from std_msgs.msg import Float64 # Getting Float64

class SubwooferShooterServer(object):

    def __init__(self, name): # 
        self.action_name = name # Ease of use
        self._result = SubShooter2024Result() # This is the result
        self._feedback = SubShooter2024Feedback() # This is the feedback
        self.shooter_client = actionlib.SimpleActionClient('/shooter/set_shooter_speed', Shooter2024Action) # This is the client that asks the shooter to shoot
        rospy.loginfo("2024_shoot_from_subwoofer_server: waiting for shooter")
        self.shooter_client.wait_for_server() # Wait for the Shooter server to finish
        self.pivot_client = actionlib.SimpleActionClient('/shooter/set_shooter_pivot', ShooterPivot2024Action) # This is the client that asks the shooter to pivot
        rospy.loginfo("2024_shoot_from_subwoofer_server: waiting for pivot")
        self.pivot_client.wait_for_server() # Wait for the ShooterPivot server to finish
        self.clawster_client = actionlib.SimpleActionClient('/clawster/clawster_server_2024', Clawster2024Action) # This is the client that asks the clawster to preshoot (Im so very funny)
        rospy.loginfo("2024_shoot_from_subwoofer_server: waiting for clawster")
        self.clawster_client.wait_for_server() # Wait for the Clawster server to finish
        rospy.loginfo("finished waiting for clients and intializing clients")

        # Since the robot is autoaligning and is up against the subwoofer, we can find exact values for these variables that the robot can use every time

        self.top_left_speed_param = rospy.get_param("top_left_speed")
        self.top_right_speed_param = rospy.get_param("top_right_speed")
        self.bottom_left_speed = rospy.get_param("bottom_left_speed")
        self.bottom_right_speed = rospy.get_param("bottom_right_speed")
        self.pivot_position_param = rospy.get_param("pivot_position_param")
        self.clawster_destination = 1
        self.clawster_mode = 1
         # Gets the value that we set for the angle that the shooter will shoot from
        # We can't shoot immediately after shooting, so we are gonna wait a bit.
        #self.delay_after_shooting = rospy.get_param("delay_after_shooting")
        rospy.loginfo("got params")

        self.server = actionlib.SimpleActionServer(self.action_name, SubShooter2024Action, execute_cb=self.execute_cb, auto_start = False) # Create our server
        rospy.loginfo("created the server")
        self.server.start() # Start up the server


    def execute_cb(self, goal: SubShooter2024Goal):
        # Look up speed and angle to send to shooter and pivot server
        r = rospy.Rate(50)


        rospy.loginfo("2024_shoot_from_subwoofer_server: pivoting")

        pivot_goal = ShooterPivot2024Goal()
        pivot_goal.pivot_position = self.pivot_position_param
        pivot_done = False
        def pivot_done_cb(state, result):
            nonlocal pivot_done
            pivot_done = True
        self.pivot_client.send_goal(pivot_goal, done_cb=pivot_done_cb)

        while (not (pivot_done) and not rospy.is_shutdown()):
            rospy.loginfo_throttle(0.5, "2024_subwoofer_shoot: pivoting in pivot")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024 subwoofr pivot preempted")
                self.pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                return
            r.sleep()
        rospy.loginfo(pivot_done)
        shooter_goal = Shooter2024Goal() # Set the goals stuff
        shooter_goal.top_left_speed = self.top_left_speed_param   
        shooter_goal.top_right_speed = self.top_right_speed_param
        shooter_goal.bottom_left_speed = self.bottom_left_speed
        shooter_goal.bottom_right_speed = self.bottom_right_speed

        shooter_done = False # The shooter isn't done.
        def shooter_feedback_cb(state, result):
            rospy.loginfo("call back happaned")
            nonlocal shooter_done
            shooter_done = True
        
        rospy.loginfo("sending goal")
        self.shooter_client.send_goal(shooter_goal, done_cb=shooter_feedback_cb) # For nested functions (defining functions inside a function), pretty much lets python know that it has to grab the variable from outside the nested function's definition (Kinda like "global" I guess)
        rospy.loginfo("goal was sent")

        while (not (shooter_done) and not rospy.is_shutdown()):
            rospy.loginfo(shooter_done)
            #rospy.loginfo_throttle(0.5, "2024_shoot_from_subwoofer_server: waiting for shooter and pivot")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_shoot_from_subwoofer_server: preempted in shooter")
                self.shooter_client.cancel_goals_at_and_before_time(rospy.Time())
                self.pivot_client.cancel_goals_at_and_before_time(rospy.Time())
                self.server.set_preempted()
                return
            r.sleep()

        rospy.loginfo("2024_shoot_from_subwoofer_server: shooting")

        clawster_goal = Clawster2024Goal()
        clawster_goal.mode = self.clawster_mode
        clawster_goal.destination = self.clawster_destination
        self.clawster_client.send_goal(clawster_goal)
        clawster_done = False
        clawster_result: Clawster2024Result = None

        def clawster_done_cb(state, result):
            nonlocal clawster_done, clawster_result
            clawster_done = True
            clawster_result = result
        self.clawster_client.send_goal(clawster_goal, done_cb=clawster_done_cb)
        while not clawster_done and not rospy.is_shutdown():
            rospy.loginfo_throttle(0.5, "2024_shoot_from_subwoofer_server: waiting for clawster")
            rospy.loginfo_throttle(0.5, clawster_done)
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_shoot_from_subwoofer_server: preempted in clawster")
                # ensure shooter turned off
                self.shooter_client.cancel_goals_at_and_before_time(rospy.Time())
                # ensure pivot stopped
                self.pivot_client.cancel_goals_at_and_before_time(rospy.Time())
                # stop clawster
                self.clawster_client.cancel_goals_at_and_before_time(rospy.Time())
                self.server.set_preempted()
                return
            r.sleep()

        rospy.loginfo("2024_shoot_from_subwoofer_server: +5 points hopefully")
        
        rospy.loginfo("2024_shoot_from_subwoofer_server: succeeded")

        self._result.success = True
        self.server.set_succeeded(self._result)
        return


if __name__ == '__main__':
    rospy.init_node('subwoofer_shooter_2024')
    server = SubwooferShooterServer(rospy.get_name())

    rospy.spin()