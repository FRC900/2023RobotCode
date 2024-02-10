#!/usr/bin/env python3

import rospy
import actionlib

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

from behavior_actions.msg import Shooting2024Goal, Shooting2024Feedback, Shooting2024Result, Shooting2024Action
from behavior_actions.msg import Shooter2024Goal, Shooter2024Feedback, Shooter2024Result, Shooter2024Action
from behavior_actions.msg import ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result, ShooterPivot2024Action
from behavior_actions.msg import Claw2024Goal, Claw2024Feedback, Claw2024Result, Claw2024Action
from std_msgs.msg import Float64
from interpolating_map import InterpolatingMap

class ShootingServer(object):

    def __init__(self, name):
        self.action_name = name
        self.result = Shooting2024Result()
        self.feedback = Shooting2024Feedback()
        self.shooter_client = actionlib.SimpleActionClient('/shooter/shooter_server_2024', Shooter2024Action)
        self.pivot_client = actionlib.SimpleActionClient('/pivot/pivot_server_2024', ShooterPivot2024Action)
        # The preshooter and claw are very similar (drive motor until limit switch pressed). They'll probably be the same server.
        self.preshooter_client = actionlib.SimpleActionClient('/preshooter/preshooter_server_2024', Claw2024Action)

        # speeds_map: [[distance: [left_speed, right_speed]], ...]
        speeds_map_param = rospy.get_param("speeds_map")

        self.left_map = InterpolatingMap()
        self.left_map.container = {l[0]: l[1][0] for l in speeds_map_param}

        self.right_map = InterpolatingMap()
        self.right_map.container = {l[0]: l[1][1] for l in speeds_map_param}

        self.angle_map = InterpolatingMap()
        self.angle_map.container = {l[0]: l[1] for l in rospy.get_param("angle_map")}

        self.server = actionlib.SimpleActionServer(self.action_name, Shooting2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

    def execute_cb(self, goal: Shooting2024Goal):
        # Look up speed and angle to send to shooter and pivot server
        left_speed = self.left_map[goal.distance]
        right_speed = self.right_map[goal.distance]
        pivot_angle = self.angle_map[goal.distance]

        rospy.loginfo("2024_shooting_server: spinning up")

        self.feedback.current_stage = self.feedback.SPINNING
        self.server.publish_feedback(self.feedback)

        shooter_goal = Shooter2024Goal()
        shooter_goal.left_shooter_speed = left_speed
        shooter_goal.right_shooter_speed = right_speed

        self.shooter_client.send_goal(shooter_goal)

        rospy.loginfo("2024_shooting_server: pivoting")

        self.feedback.current_stage = self.feedback.PIVOTING
        self.server.publish_feedback(self.feedback)

        pivot_goal = ShooterPivot2024Goal()
        pivot_goal.pivot_position = pivot_angle

        self.pivot_client.send_goal(pivot_goal)

        while self.shooter_client.get_state() 
        self.pivot_client.wait_for_result()

        rospy.loginfo("2024_shooting_server: shooting")

        if not goal.setup_only:
            self.feedback.current_stage = self.feedback.SHOOTING
            self.server.publish_feedback(self.feedback)

            preshooter_goal = Claw2024Goal()
            preshooter_goal.mode = preshooter_goal.OUTTAKE_CLAW

            self.preshooter_client.send_goal(preshooter_goal)

            self.preshooter_client.wait_for_result()

            rospy.loginfo("2024_shooting_server: +5 points hopefully")

        self.result.success = True
        rospy.loginfo("2024_shooting_server: succeeded")
        self.server.set_succeeded(self.result)

# def dyn_rec_callback(config, level):
#     rospy.loginfo("Received reconf call: " + str(config))
#     global conveyor_speed
#     global diverter_speed
#     conveyor_speed = config["conveyor_speed"]
#     diverter_speed = config["diverter_speed"]
#     return config

if __name__ == '__main__':
    rospy.init_node('shooting_server_2024')
    
    # ddynrec = DDynamicReconfigure("diverter_dyn_rec")
    # ddynrec.add_variable("conveyor_speed", "float/double variable", rospy.get_param("note_conveyor_speed"), 0.0, 13.0)
    # ddynrec.add_variable("diverter_speed", "float/double variable", rospy.get_param("note_diverter_speed"), 0.0, 13.0)
    # ddynrec.start(dyn_rec_callback)

    server = ShootingServer(rospy.get_name())
    rospy.spin()