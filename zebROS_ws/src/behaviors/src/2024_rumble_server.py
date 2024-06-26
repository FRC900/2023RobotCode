#! /usr/bin/env python3

import rospy
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result

from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from talon_state_msgs.msg import TalonFXProState
from sensor_msgs.msg import JointState

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64
#from norfair_ros.msg import Detections, Detection
from field_obj.msg import Detection
from frc_msgs.srv import RumbleCommand, RumbleCommandRequest, RumbleCommandResponse
from frc_msgs.msg import MatchSpecificData

import math
import time

class Rumble2024Server():

    def __init__(self, name):        
        #self.arm_client = actionlib.SimpleActionClient('/arm/move_arm_server_2024', Arm2024Action)
        #rospy.loginfo("2024_intaking_server: waiting for arm server")
        #self.arm_client.wait_for_server()
        rospy.loginfo("Rumble server starting")

        #self.shooter_pos_sub = rospy.Subscriber("/", TalonFXProState, shooter_pivot_callback)
        self.rumble_srv = rospy.ServiceProxy("/frcrobot_rio/rumble_controller/command", RumbleCommand)

        self.previous_intake_switch = 0
        self.previous_preshooter_switch = 0
        
        self.intaking_current = 0.0
        self.intaking_talon_idx = None
        self.talonfxpro_sub = rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.talonfxpro_states_cb)
        # self.norfair_sub = rospy.Subscriber('/norfair/output', Detections, self.notes_callback)
        self.norfair_sub = rospy.Subscriber('/tf_object_detection_zedx_front/object_detection_world', Detection, self.notes_callback)
        self.limit_switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, self.limit_switch_cb)
        
        self.notes_max_distance = rospy.get_param("note_distance_away")
        self.preshooter_limit_switch_name = rospy.get_param("preshooter_limit_switch_name")
        self.claw_limit_switch_name = rospy.get_param("claw_limit_switch_name")

        self.intake_limit_switch_name = rospy.get_param("intake_limit_switch_name")
        self.rumble_value = rospy.get_param("rumble_on_note")

        self.mode = 0
        self.DISABLED = 1
        self.AUTO = 2
        self.TELEOP = 3
        self.NOTE_SEEN = 4
        self.last_note_seen = rospy.Time(0)
        self.match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.match_data_cb) 
        self.rumble = rospy.Timer(rospy.Duration(1.0/20.0), self.rumble_loop)
        self.closest_note = 900
        self.time_touched_note = rospy.Time.now()

        self.intaking_rumble = False
        self.shot_note = False

        self.current_rumble_state = 0

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

    # recives norfair messages and finds the closest note
    def notes_callback(self, msg: Detection):
        closest_dist = None
        note_counter = 0
        for detection in msg.objects:
            if detection.id != "note":
                continue
            note_counter += 1
            dist = math.hypot(detection.location.x, detection.location.y)
            if (closest_dist is None) or (dist < closest_dist):
                closest_dist = dist
            #rospy.loginfo(f"Note detection at {x, y}")
        self.closest_note = closest_dist

        if note_counter > 0:
            self.last_note_seen = rospy.Time.now()
            if self.mode != self.NOTE_SEEN:
                rospy.loginfo(f'self.mode = {self.mode}, self.NOTE_SEEN = {self.NOTE_SEEN}')
                self.mode = self.NOTE_SEEN
        elif (self.mode == self.NOTE_SEEN) and ((rospy.Time.now() - self.last_note_seen)> rospy.Duration(1.0)):
            self.mode = 0
            rospy.loginfo(f'self.mode = {self.mode}, self.NONE')


    def match_data_cb(self, data: MatchSpecificData):
        if (self.mode != self.NOTE_SEEN):
            if not data.Enabled:
                if self.mode != self.DISABLED:
                    rospy.loginfo(f'self.mode = {self.mode}, self.DISABLED = {self.DISABLED}')
                    self.mode = self.DISABLED
            else: #enabled
                if data.Autonomous and (self.mode != self.AUTO):
                    rospy.loginfo(f'self.mode = {self.mode}, self.AUTO = {self.AUTO}')
                    self.mode = self.AUTO
                elif not data.Autonomous and (self.mode != self.TELEOP):
                    rospy.loginfo(f'self.mode = {self.mode}, self.TELEOP = {self.TELEOP}')
                    self.mode = self.TELEOP

        if data.Autonomous and data.Enabled:
            self.should_run_loop = False
        else:
            self.should_run_loop = True

    def limit_switch_cb(self, data):
        #rospy.loginfo_throttle(1, "2024_rumble_server: limit switch callback")
        # check claw switch
        
        # intake preshooter
        #   0        0      -> No change from previous state
        #   1        0      -> Start rumble
        #   0        1      -> Stop rumble
        #   1        1      -> Stop rumble

        if (not self.preshooter_limit_switch_name in data.name) or (not self.intake_limit_switch_name in data.name):
            rospy.logerr_throttle(1.0, f'2024_rumble_server: intake or preshooter limit not found')
            return
        
        
        current_intake_switch = data.position[data.name.index(self.intake_limit_switch_name)]
        current_preshooter_switch = data.position[data.name.index(self.preshooter_limit_switch_name)]
        # rospy.loginfo_throttle(1, f"Current intake {current_intake_switch} preshooter {current_preshooter_switch} \n previous intake {self.previous_intake_switch} shooter prev {self.previous_preshooter_switch}")
        # we see note, rumble until preshooter switch
        if self.previous_intake_switch == 0 and current_intake_switch:
            rospy.loginfo("Intaking rumble")
            self.intaking_rumble = True

        # note hits preshooter, stop rumble
        if self.previous_preshooter_switch == 0 and current_preshooter_switch:
            rospy.loginfo("Intaking rumble STOP")
            self.intaking_rumble = False
        
        # note left (shot), rumble for a half second
        if current_preshooter_switch == 0 and self.previous_preshooter_switch:
            rospy.loginfo("SHOT NOTE")
            self.shot_note = True
        
        self.previous_intake_switch = current_intake_switch
        self.previous_preshooter_switch = current_preshooter_switch

    # runs at some hz
    def rumble_loop(self, event):
        #rospy.loginfo_throttle(1, "Rumble running")
        rumble_srv = RumbleCommandRequest()
        rumble_srv.left = 0
        rumble_srv.right = 0
        
        #rospy.loginfo_throttle(1, "Rumble loop")
        # TODO make this a parameter
        # note should be gone after 1 second
        if self.intaking_rumble:
            if self.current_rumble_state != self.rumble_value:
                rumble_srv.left = self.rumble_value
                rumble_srv.right = self.rumble_value
                self.rumble_srv.call(rumble_srv) # could optimize this to not call rumble_srv a lot
                self.current_rumble_state = self.rumble_value
        else:
            if self.current_rumble_state != 0:
                rumble_srv.left = 0
                rumble_srv.right = 0
                self.rumble_srv.call(rumble_srv) 
                self.current_rumble_state = 0

        if self.shot_note:
            rospy.loginfo("Note left")
            rumble_srv.left = self.rumble_value
            rumble_srv.right = self.rumble_value
            self.rumble_srv.call(rumble_srv)
            time.sleep(1)
            rospy.loginfo("Sleep done note left")
            self.shot_note = False
            rumble_srv.left = 0
            rumble_srv.right = 0
            self.rumble_srv.call(rumble_srv)
        
        
if __name__ == '__main__':
    time.sleep(2)
    rospy.logwarn("STARTING RUMBLE SERVER")
    rospy.init_node('rumble_server_2024')
    server = Rumble2024Server(rospy.get_name())
    rospy.spin()