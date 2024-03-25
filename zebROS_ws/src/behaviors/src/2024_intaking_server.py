#! /usr/bin/env python3

import rospy
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result
from behavior_actions.msg import ShooterPivot2024Action, ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result
from behavior_actions.msg import NoteDiverterAction, NoteDiverterFeedback, NoteDiverterResult, NoteDiverterGoal
from behavior_actions.msg import Clawster2024Action, Clawster2024Feedback, Clawster2024Result, Clawster2024Goal
from behavior_actions.msg import Arm2024Action, Arm2024Feedback, Arm2024Result, Arm2024Goal
from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from talon_state_msgs.msg import TalonFXProState
from sensor_msgs.msg import JointState

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64

SIM = True

class Intaking2024Server(object):
    # create messages that are used to publish feedback/result
    feedback = Intaking2024Feedback()
    result = Intaking2024Result()

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        self.intaking_speed = config["intaking_speed"]
        self.current_threshold = config["current_threshold"]
        return config

    def __init__(self, name):
        self.action_name = name
        
        self.arm_client = actionlib.SimpleActionClient('/arm/move_arm_server_2024', Arm2024Action)
        rospy.loginfo("2024_intaking_server: waiting for arm server")
        if not SIM: self.arm_client.wait_for_server()
        # rospy.logerr("ARM CLIENT NOT INITALIZED BECAUSE IT DOESN'T EXIST")

        self.shooter_pivot_client = actionlib.SimpleActionClient('/shooter/set_shooter_pivot', ShooterPivot2024Action)
        rospy.loginfo("2024_intaking_server: waiting for shooter pivot server")
        if not SIM: self.shooter_pivot_client.wait_for_server()
        self.diverter_client = actionlib.SimpleActionClient('/diverter/diverter_server_2024', NoteDiverterAction)
        rospy.loginfo("2024_intaking_server: waiting for diverter server")
        if not SIM: self.diverter_client.wait_for_server()
        self.clawster_client = actionlib.SimpleActionClient('/clawster/clawster_server_2024', Clawster2024Action)
        rospy.loginfo("2024_intaking_server: waiting for clawster server")
        if not SIM: self.clawster_client.wait_for_server()
        rospy.loginfo(f"Clawster client {self.clawster_client}")

        self.pivot_position = 0
        self.pivot_index = None

        self.shooter_pos_sub = rospy.Subscriber("/frcrobot_jetson/talonfxpro_states", TalonFXProState, self.shooter_pivot_callback)
        self.intake_client = rospy.ServiceProxy("/frcrobot_jetson/intake_talonfxpro_controller/command", Command)

        ddynrec = DDynamicReconfigure("intaking_dyn_rec")
        ddynrec.add_variable("intaking_speed", "float/double variable", rospy.get_param("intaking_speed"), 0.0, 1.0)
        ddynrec.add_variable("current_threshold", "float/double variable", rospy.get_param("current_threshold"), 0.0, 200.0)
        ddynrec.start(self.dyn_rec_callback)

        self.intaking_speed = rospy.get_param("intaking_speed")
        self.intaking_timeout = rospy.get_param("intaking_timeout")
        self.safe_shooter_angle = rospy.get_param("safe_shooter_angle")

        self.current_threshold = rospy.get_param("current_threshold")

        self.outtaking_speed = rospy.get_param("outtaking_speed")
        self.outtaking_time = rospy.get_param("outtaking_time")

        self.backwards_time = rospy.get_param("backwards_time")
        
        self.intaking_current = 0.0
        self.intaking_talon_idx = None
        self.talonfxpro_sub = rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.talonfxpro_states_cb)

        self.joint_state_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, callback=self.rio_callback)
        self.diverter_switch = False
        self.run_intake_backwards = None
        self.arm_done = False
        self.server = actionlib.SimpleActionServer(self.action_name, Intaking2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

    def rio_callback(self, data):
        # check diverter_switch
        if "diverter_limit_switch" in data.name:
            if self.feedback.note_hit_intake != data.position[data.name.index("diverter_limit_switch")]:
                rospy.loginfo("Intaking, note hit (or left) diverter")
                self.feedback.note_hit_intake = data.position[data.name.index("diverter_limit_switch")]
                self.server.publish_feedback(self.feedback)

            if self.diverter_switch and not data.position[data.name.index("diverter_limit_switch")]:
                rospy.loginfo("Running backwards in 0.5 seconds!")
                self.run_intake_backwards = rospy.Time.now() + rospy.Duration(0.5)
            self.diverter_switch = data.position[data.name.index("diverter_limit_switch")]
            #rospy.loginfo(f"Found {self.claw_switch_name} with value {self.claw_switch}")
        else:
            rospy.logwarn_throttle(1.0, f'2024_intaking_server: diverter_limit_switch not found')
            pass
            
    def shooter_pivot_callback(self, data):
        if self.pivot_index is None:
            for i in range(len(data.name)):
                if (data.name[i] == "shooter_pivot_motionmagic_joint"): 
                    self.pivot_position = data.position[i]
                    self.pivot_index = i
                    break
        else:
            self.pivot_position = data.position[self.pivot_index]

    def talonfxpro_states_cb(self, states: TalonFXProState):
        #rospy.loginfo_throttle(5, "Intaking node recived talonfx pro states")
        if (self.intaking_talon_idx == None):
            for i in range(len(states.name)):
                #rospy.loginfo(data.name[i])
                if (states.name[i] == "intake"): 
                    self.intaking_current = states.torque_current[i]
                    self.intaking_talon_idx = i
                    break
        else:
            self.intaking_current = states.torque_current[self.intaking_talon_idx]
    

    def execute_cb(self, goal: Intaking2024Goal):
        r = rospy.Rate(60)

        self.run_intake_backwards = None

        # self.feedback.have_note = False

        if SIM:
            rospy.loginfo(f"2024_intaking_server: SIMULATION MODE, goal = {goal}")
            import time
            time.sleep(2)
            # say that a note hit the intake
            self.feedback.note_hit_intake = True
            self.server.publish_feedback(self.feedback)
            # loop for 0.5 seconds and return if preempted
            start = time.time()
            while time.time() - start < 0.5:
                if self.server.is_preempt_requested() or rospy.is_shutdown():
                    rospy.loginfo("2024_intaking_server: preempted")
                    self.server.set_preempted()
                    return
                r.sleep()
            self.result.success = True
            self.server.set_succeeded(self.result)
            return

        if goal.destination == goal.OUTTAKE:
            intake_srv = CommandRequest()
            intake_srv.command = -self.outtaking_speed
            self.intake_client.call(intake_srv)

            start = rospy.Time.now()

            while goal.run_until_preempt or (not (rospy.is_shutdown() or (rospy.Time.now() - start).to_sec() > self.outtaking_time)):
                rospy.loginfo_throttle(0.5, f"2024_intaking_server: outtaking")
                    
                if self.server.is_preempt_requested():
                    rospy.loginfo("2024_intaking_server: preempted")

                    # stop intake
                    intake_srv = CommandRequest()
                    intake_srv.command = 0.0
                    self.intake_client.call(intake_srv)

                    self.server.set_preempted()
                    return
                r.sleep()
            
            # stop intake
            intake_srv = CommandRequest()
            intake_srv.command = 0.0
            self.intake_client.call(intake_srv)

            return

        self.feedback.state = self.feedback.SHOOTERPIVOTING
        self.server.publish_feedback(self.feedback)

        # need ensure claw is in correct position
        if goal.destination == goal.CLAW:
            self.arm_done = False
            def arm_result(state, arm_result: Arm2024Result):
                rospy.loginfo(f"Arm server finished with {arm_result.success} ")
                self.arm_done = True
                # not anything great to do if we failed, so just keep going
            
            arm_goal = Arm2024Goal()
            # move us to diverting position
            arm_goal.path = arm_goal.DIVERTER 
            self.arm_client.send_goal(arm_goal, done_cb=arm_result)
            while not self.arm_done and not rospy.is_shutdown():
                if self.server.is_preempt_requested():
                    rospy.loginfo("2024_intaking_server: preempted")
                    self.arm_client.cancel_goals_at_and_before_time(rospy.Time.now())
                    self.server.set_preempted()
                    return
                r.sleep()
                rospy.loginfo_throttle(0.5, "2024_intaking_server: waiting for arm")
            rospy.logwarn("2024_intaking_server: ARM DONE")

        if self.pivot_position > self.safe_shooter_angle:
            pivot_goal = ShooterPivot2024Goal()
            pivot_goal.pivot_position = self.safe_shooter_angle - 0.2
            self.shooter_pivot_client.send_goal(pivot_goal)
            while self.pivot_position > self.safe_shooter_angle and not rospy.is_shutdown():
                if self.server.is_preempt_requested():
                    rospy.loginfo("2024_intaking_server: preempted")
                    self.shooter_pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())
                    self.server.set_preempted()
                    return
                r.sleep()
                rospy.loginfo_throttle(0.5, "2024_intaking_server: waiting for shooter pivot")
        
            self.shooter_pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())

        self.feedback.state = self.feedback.DIVERTING
        self.server.publish_feedback(self.feedback)

        diverter_goal = NoteDiverterGoal()
        if goal.destination == goal.SHOOTER:
            diverter_goal.mode = diverter_goal.TO_SHOOTER 
        elif goal.destination == goal.CLAW:
            diverter_goal.mode = diverter_goal.TO_CLAW
        rospy.loginfo("2024_intaking_server: running diverter")
        self.diverter_client.send_goal(diverter_goal)

        self.feedback.state = self.feedback.CLAWSTERING
        self.server.publish_feedback(self.feedback)

        clawster_goal = Clawster2024Goal()
        clawster_goal.mode = clawster_goal.INTAKE
        clawster_done = False
        clawster_result: Clawster2024Result = None
        def clawster_done_cb(state, result):
            nonlocal clawster_done, clawster_result
            clawster_done = True
            clawster_result = result
        
        if goal.destination == goal.SHOOTER:
            clawster_goal.destination = clawster_goal.PRESHOOTER
            rospy.loginfo("2024_intaking_server: running preshooter")
        elif goal.destination == goal.CLAW:
            rospy.loginfo("2024_intaking_server: running claw")
            clawster_goal.destination = clawster_goal.CLAW

        self.clawster_client.send_goal(clawster_goal, done_cb=clawster_done_cb)

        self.feedback.state = self.feedback.INTAKING
        self.server.publish_feedback(self.feedback)

        intake_srv = CommandRequest()
        intake_srv.command = self.intaking_speed
        orig_command = intake_srv.command
        self.intake_client.call(intake_srv)

        backwards_start_time = None

        start = rospy.Time.now()
        # if run until preempt want to just go for the entire auto
        while goal.run_until_preempt or (not (clawster_done or rospy.is_shutdown() or (rospy.Time.now() - start).to_sec() > self.intaking_timeout)):
            rospy.loginfo_throttle(0.5, f"2024_intaking_server: waiting for {'preshooter' if goal.destination == goal.SHOOTER else 'claw'}")
            if self.run_intake_backwards is not None and rospy.Time.now() > self.run_intake_backwards:
                rospy.loginfo("Running backwards!")
                intake_srv.command = -self.outtaking_speed
                self.intake_client.call(intake_srv)
                self.run_intake_backwards = None
                backwards_start_time = rospy.Time.now()
            if backwards_start_time is not None and rospy.Time.now() > backwards_start_time + rospy.Duration(self.backwards_time):
                rospy.loginfo("Running forward again!")
                intake_srv.command = orig_command
                self.intake_client.call(intake_srv)
                backwards_start_time = None
            if self.intaking_current > self.current_threshold:
                rospy.logwarn(f"Intaking current = {self.intaking_current} is above {self.current_threshold}")
                # self.feedback.note_hit_intake = True
                self.server.publish_feedback(self.feedback)
            # if clawster_done:
            #     self.feedback.have_note = True
            #     self.server.publish_feedback(self.feedback)
            
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_intaking_server: preempted")

                # stop intake
                intake_srv = CommandRequest()
                intake_srv.command = 0.0
                self.intake_client.call(intake_srv)

                # stop diverter
                self.diverter_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.clawster_client.cancel_goals_at_and_before_time(rospy.Time.now())
                # stop preshooter and claw
                self.server.set_preempted()
                return
            r.sleep()
        
        # stop intake
        intake_srv = CommandRequest()
        intake_srv.command = 0.0
        self.intake_client.call(intake_srv)

        # stop diverter
        self.diverter_client.cancel_goals_at_and_before_time(rospy.Time.now())
        # stop preshooter and claw
        self.clawster_client.cancel_goals_at_and_before_time(rospy.Time.now())

        if clawster_done:
            if clawster_result.has_game_piece:
                rospy.loginfo("2024_intaking_server: succeeded")
                self.result.success = True
                self.server.set_succeeded(self.result)
                return
            else:
                rospy.loginfo("2024_intaking_server: clawster done, but no game piece??")
                self.result.success = False
                self.server.set_succeeded(self.result)
                return

        if (rospy.Time.now() - start).to_sec() > self.intaking_timeout:
            rospy.loginfo("2024_intaking_server: timed out")
            self.result.success = False
            self.server.set_succeeded(self.result)
            return

        rospy.loginfo(f"2024_intaking_server: we haven't succeeded, so we've failed. is rospy shutdown? {rospy.is_shutdown()}")
        self.result.success = False
        self.server.set_succeeded(self.result)
        

       
if __name__ == '__main__':
    rospy.init_node('intaking_server_2024')
    server = Intaking2024Server(rospy.get_name())
    rospy.spin()