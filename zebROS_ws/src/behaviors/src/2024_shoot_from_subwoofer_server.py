#!/usr/bin/env python3

import rospy
import actionlib

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

from behavior_actions.msg import SubShooter2024Goal, SubShooter2024Feedback, SubShooter2024Result, SubShooter2024Action
from behavior_actions.msg import Shooting2024Goal, Shooting2024Feedback, Shooting2024Result, Shooting2024Action
from behavior_actions.msg import Shooter2024Goal, Shooter2024Feedback, Shooter2024Result, Shooter2024Action
from behavior_actions.msg import ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result, ShooterPivot2024Action
from behavior_actions.msg import Clawster2024Goal, Clawster2024Feedback, Clawster2024Result, Clawster2024Action
from std_msgs.msg import Float64

class SubwooferShooterServer(object):

    def __init__(self, name):
        self.action_name = name
        self._result = SubShooter2024Result()
        self._feedback = SubShooter2024Feedback()
        self.shooter_client = actionlib.SimpleActionClient('/shooter/set_shooter_speed', Shooter2024Action)
        rospy.loginfo("2024_shoot_from_subwoofer_server: waiting for shooter")
        self.shooter_client.wait_for_server()
        self.pivot_client = actionlib.SimpleActionClient('/shooter/set_shooter_pivot', ShooterPivot2024Action)
        rospy.loginfo("2024_shoot_from_subwoofer_server: waiting for pivot")
        self.pivot_client.wait_for_server()
        # The preshooter and claw are very similar (drive motor until limit switch pressed). They'll probably be the same server.
        self.preshooter_client = actionlib.SimpleActionClient('/preshooter/preshooter_server_2024', Clawster2024Action)
        rospy.loginfo("2024_shoot_from_subwoofer_server: waiting for preshooter")
        self.preshooter_client.wait_for_server()

        # speeds_map: [[distance: [left_speed, right_speed]], ...]
        self.left_speed = rospy.get_param("left_speed")
        self.right_speed = rospy.get_param("right_speed")
        self.pivot_angle = rospy.get_param("pivot_angle")


        self.delay_after_shooting = rospy.get_param("delay_after_shooting")

        self.server = actionlib.SimpleActionServer(self.action_name, Shooting2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

    def execute_cb(self, goal: SubShooter2024Goal):
        # Look up speed and angle to send to shooter and pivot server

        rospy.loginfo("2024_shooting_server: spinning up")

        self.feedback.current_stage = self.feedback.SPINNING
        self.server.publish_feedback(self.feedback)

        shooter_goal = SubShooter2024Goal()
        shooter_goal.left_shooter_speed = self.left_speed
        shooter_goal.right_shooter_speed = self.right_speed

        shooter_done = False
        def shooter_feedback_cb(feedback: Shooter2024Feedback):
            nonlocal shooter_done
            shooter_done = feedback.is_shooting_at_speed
        self.shooter_client.send_goal(shooter_goal, feedback_cb=shooter_feedback_cb)

        rospy.loginfo("2024_shooting_server: pivoting")

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
            rospy.loginfo_throttle(0.5, "2024_shooting_server: waiting for shooter and pivot")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_shooting_server: preempted")

                # ensure shooter turned off
                self.shooter_client.cancel_goals_at_and_before_time(rospy.Time())

                # ensure pivot stopped
                self.pivot_client.cancel_goals_at_and_before_time(rospy.Time())

                self.server.set_preempted()

                return
            r.sleep()

        rospy.loginfo("2024_shooting_server: shooting")

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
                rospy.loginfo_throttle(0.5, "2024_shooting_server: waiting for preshooter")
                if self.server.is_preempt_requested():
                    rospy.loginfo("2024_shooting_server: preempted")

                    # ensure shooter turned off
                    self.shooter_client.cancel_goals_at_and_before_time(rospy.Time())

                    # ensure pivot stopped
                    self.pivot_client.cancel_goals_at_and_before_time(rospy.Time())

                    # stop preshooter
                    self.preshooter_client.cancel_goals_at_and_before_time(rospy.Time())

                    self.server.set_preempted()

                    return
                r.sleep()

            rospy.loginfo("2024_shooting_server: +5 points hopefully")
        
        if not goal.leave_spinning:
            rospy.loginfo("2024_shooting_server: spinning down")
            rospy.sleep(rospy.Duration(self.delay_after_shooting))

            # ensure shooter turned off
            self.shooter_client.cancel_goals_at_and_before_time(rospy.Time())

            # ensure pivot stopped
            self.pivot_client.cancel_goals_at_and_before_time(rospy.Time())

        # stop preshooter
        self.preshooter_client.cancel_goals_at_and_before_time(rospy.Time())

        self.result.success = True
        rospy.loginfo("2024_shooting_server: succeeded")
        self.server.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('shooting_server_2024')
    
    server = ShootingServer(rospy.get_name())
    rospy.spin()