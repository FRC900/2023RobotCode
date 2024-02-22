#!/usr/bin/env python3
# Imports
# Note (DELETE LATER) The reason I put ask Ben for all of these is because this is his code lol, I just copied it and adjusting it for the purposes of this case. You could probably ask Kevin and it would still work fine, but I feel like Ben would be easier because he already knows whats going on.
import rospy # This is ros, you should know this
import actionlib # For action servers

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure # No clue, self-reminder to ASK ben bc i CANNOT get this into my head for some reason

from behavior_actions.msg import SubShooter2024Goal, SubShooter2024Feedback, SubShooter2024Result, SubShooter2024Action # Get all of the stuff for this server
from behavior_actions.msg import Shooter2024Goal, Shooter2024Feedback, Shooter2024Result, Shooter2024Action # Controls the shooter
from behavior_actions.msg import ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result, ShooterPivot2024Action # Controls the pivot position of the shooter
from behavior_actions.msg import Clawster2024Goal, Clawster2024Feedback, Clawster2024Result, Clawster2024Action # Since both the claw and preshooter use a limit switch (No clue what that is btw), they'll use the same server
from std_msgs.msg import Float64 # Getting Float64

class SubwooferShooterServer(object):

    def __init__(self, name): # "Init"iate variables
        self.action_name = name # Ease of use
        self._result = SubShooter2024Result() # This is the result
        self._feedback = SubShooter2024Feedback() # This is the feedback
        self.shooter_client = actionlib.SimpleActionClient('/shooter/set_shooter_speed', Shooter2024Action) # This is the client that asks the shooter to shoot
        rospy.loginfo("2024_shoot_from_subwoofer_server: waiting for shooter")
        self.shooter_client.wait_for_server() # Wait for the Shooter server to finish
        self.pivot_client = actionlib.SimpleActionClient('/shooter/set_shooter_pivot', ShooterPivot2024Action) # This is the client that asks the shooter to pivot
        rospy.loginfo("2024_shoot_from_subwoofer_server: waiting for pivot")
        self.pivot_client.wait_for_server() # Wait for the ShooterPivot server to finish
        self.preshooter_client = actionlib.SimpleActionClient('/preshooter/preshooter_server_2024', Clawster2024Action) # This is the client that asks the preshooter to preshoot (Im so very funny)
        rospy.loginfo("2024_shoot_from_subwoofer_server: waiting for preshooter")
        self.preshooter_client.wait_for_server() # Wait for the Clawster server to finish

        # Since the robot is autoaligning and is up against the subwoofer, we can find exact values for these variables that the robot can use every time
        self.left_speed = rospy.get_param("left_speed") # Gets the value that we set for the left motor speed of the shooter
        self.right_speed = rospy.get_param("right_speed") # Gets the value that we set for the right motor speed of the shooter
        self.pivot_angle = rospy.get_param("pivot_angle") # Gets the value that we set for the angle that the shooter will shoot from

        # We can't shoot immediately after shooting, so we are gonna wait a bit.
        self.delay_after_shooting = rospy.get_param("delay_after_shooting")

        self.server = actionlib.SimpleActionServer(self.action_name, SubShooter2024Action, execute_cb=self.execute_cb, auto_start = False) # Create our server
        self.server.start() # Start up the server

    def execute_cb(self, goal: SubShooter2024Goal):
        # Look up speed and angle to send to shooter and pivot server

        rospy.loginfo("2024_shoot_from_subwoofer_server: spinning up")


        # ASK Ben if I need to add "current_stage" to the feedback section of the yaml file (2024_subwoofer_shooter.yaml)
        self.feedback.current_stage = self.feedback.SPINNING # Set the current feedback as spinning
        self.server.publish_feedback(self.feedback) # Publish that feedback

        # ASK Ben if I need to add "left_shooter_speed" and/or "right_shooter_speed" to the goal section of the yaml file. I think I already asked and he said I didn't but I want to make sure
        shooter_goal = SubShooter2024Goal() # Set the goals stuff
        shooter_goal.left_shooter_speed = self.left_speed # The speed for the left motor is the speed that we set in the yaml file for the left motor (Wow so cool and not self-explanatory at all)
        shooter_goal.right_shooter_speed = self.right_speed # The speed for the right motor is the speed that we set in the yaml file for the right motor (Wow so cool and not self-explanatory at all) (Definitely didn't copy and paste this, whatttttt)

        shooter_done = False # The shooter isn't done.
        def shooter_feedback_cb(feedback: Shooter2024Feedback):
            nonlocal shooter_done # For nested functions (defining functions inside a function), pretty much lets python know that it has to grab the variable from outside the nested function's definition (Kinda like "global" I guess)
            # ASK Ben why I don't have to add "self." to the beginning of this
            shooter_done = feedback.is_shooting_at_speed # Is the shooter shooting at speed?

# After this the code needs editing to make it do the subwoofer shooter thing, bc this was ben's code originally and we're just editing it to match our case.
        self.shooter_client.send_goal(shooter_goal, feedback_cb=shooter_feedback_cb)

        rospy.loginfo("2024_shoot_from_subwoofer_server: pivoting")

        self.feedback.current_stage = self.feedback.PIVOTING
        self.server.publish_feedback(self.feedback)

        pivot_goal = ShooterPivot2024Goal()
        pivot_goal.pivot_position = pivot_angle

        pivot_done = False
        def pivot_done_cb(state, result):
            nonlocal pivot_done
            pivot_done = True
        self.pivot_client.send_goal(pivot_goal, done_cb=pivot_done_cb)

        r = rospy.Rate(60.0)

        while not (shooter_done and pivot_done):
            rospy.loginfo_throttle(0.5, "2024_shoot_from_subwoofer_server: waiting for shooter and pivot")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_shoot_from_subwoofer_server: preempted")

                # ensure shooter turned off
                self.shooter_client.cancel_goals_at_and_before_time(rospy.Time())

                # ensure pivot stopped
                self.pivot_client.cancel_goals_at_and_before_time(rospy.Time())

                self.server.set_preempted()

                return
            r.sleep()

        rospy.loginfo("2024_shoot_from_subwoofer_server: shooting")

        if not goal.setup_only:
            self.feedback.current_stage = self.feedback.SHOOTING
            self.server.publish_feedback(self.feedback)

            preshooter_goal = Clawster2024Goal()
            preshooter_goal.mode = preshooter_goal.OUTTAKE

            self.preshooter_client.send_goal(preshooter_goal)

            preshooter_done = False
            def preshooter_done_cb(state, result):
                nonlocal preshooter_done
                preshooter_done = True
            self.preshooter_client.send_goal(preshooter_goal, done_cb=preshooter_done_cb)

            while not preshooter_done and not rospy.is_shutdown():
                rospy.loginfo_throttle(0.5, "2024_shoot_from_subwoofer_server: waiting for preshooter")
                if self.server.is_preempt_requested():
                    rospy.loginfo("2024_shoot_from_subwoofer_server: preempted")

                    # ensure shooter turned off
                    self.shooter_client.cancel_goals_at_and_before_time(rospy.Time())

                    # ensure pivot stopped
                    self.pivot_client.cancel_goals_at_and_before_time(rospy.Time())

                    # stop preshooter
                    self.preshooter_client.cancel_goals_at_and_before_time(rospy.Time())

                    self.server.set_preempted()

                    return
                r.sleep()

            rospy.loginfo("2024_shoot_from_subwoofer_server: +5 points hopefully")
        
        if not goal.leave_spinning:
            rospy.loginfo("2024_shoot_from_subwoofer_server: spinning down")
            rospy.sleep(rospy.Duration(self.delay_after_shooting))

            # ensure shooter turned off
            self.shooter_client.cancel_goals_at_and_before_time(rospy.Time())

            # ensure pivot stopped
            self.pivot_client.cancel_goals_at_and_before_time(rospy.Time())

        # stop preshooter
        self.preshooter_client.cancel_goals_at_and_before_time(rospy.Time())

        self.result.success = True
        rospy.loginfo("2024_shoot_from_subwoofer_server: succeeded")
        self.server.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('subwoofer_shooter_2024')
    
    server = ShootingServer(rospy.get_name())
    rospy.spin()