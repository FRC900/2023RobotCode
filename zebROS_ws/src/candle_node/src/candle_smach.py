#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import sys
import time
from candle_controller_msgs.srv import Colour, ColourRequest, Animation, AnimationRequest
from frc_msgs.msg import MatchSpecificData
from behavior_actions.msg import AutoMode
from behavior_actions.msg import AutoAlignSpeaker
from frc_msgs.msg import MatchSpecificData




def match_data_callback(msg):
    global is_disabled
    is_disabled = msg.Disabled
    global is_auto
    is_auto = msg.Autonomous
    global team_color
    if team_color == None:
        if msg.allianceColor == 0:
            team_color = (255, 0, 0)
        elif msg.allianceColor == 1:
            team_color = (0, 0, 255)

def auto_mode_callback(msg):
   global auto_mode
   auto_mode = msg.auto_mode

# def has_note_callback(msg):
#     We really should make this happen
#     global has_note
#     has_note = 

def distance_callback(msg):
   global in_range
   if (msg.distance < shooting_distance):
    in_range = True
   else:
    in_range = False

def send_colour(r_col, g_col, b_col):
    colour = ColourRequest()
    # Start/counts should be edited to match the real robot
    colour.start = 9
    colour.count = 15
    colour.red = r_col
    colour.green = g_col
    colour.blue = b_col
    colour.white = 0
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/colour')
    try:
        colour_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour', Colour)
        colour_client(colour)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def send_animation(speed, start, count, animation_type):
    animation = AnimationRequest()
    animation.speed = speed
    animation.start = start
    animation.count = count
    animation.animation_type = animation_type
    animation.red = 0
    animation.green = 0
    animation.blue = 0
    animation.white = 0
    animation.direction = 0
    animation.brightness = 0.75
    animation.reversed = False
    animation.param4 = 0
    animation.param5 = 0
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/animation')
    try:
        animation_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour', Animation)
        animation_client(animation)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

class head_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['disabled', 'auto_state', 'in_range', 'not_in_range', 'no_note', 'blue_test_state'])

    def execute(self, userdata):
        #if is_disabled or is_auto: # Do we want something different if god forbid we disable during the match?
        #    if auto_mode > 4:
        #        return 'disabled'
        #    return 'auto_state'
        if is_disabled:
            return 'disabled'
        elif is_auto:
            return 'auto_state'

        elif has_note:
                if (in_range == True):
                    return 'in_range'
                elif (in_range != True):
                    return 'not_in_range'

        return 'no_note'
        #else:
        #return 'blue_test_state'



    
class auto_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state auto_state (rainbow), prepare for enlightenment')
        send_animation(1, 0, 6, 3)
        while (is_auto): # A bit fishy
            r.sleep()
        return 'teleop'

class disabled(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state disabled, white light :')
        send_colour(255, 255, 255) # white :)
        while (is_disabled): # A bit fishy
            r.sleep()
        return 'teleop'



class blue_test_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['head_state'])
    
    def execute(self, userdata):
        rospy.loginfo('In blue tests teat mahcine thing')
        send_colour(0, 0, 255)
        return 'head_state'

class ready_to_shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_note', 'left_range'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state ready_to_shoot (green light), piece acquired')
        send_colour(255, 182, 193) # Pink
        while has_note and in_range: # Half-real
            r.sleep()
        if not has_note: # Not real
            return 'lost_note'
        return 'left_range'

class out_of_range(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_note', 'entered_range'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state out_of_range (green light)')
        send_colour(0, 255, 0) # Green
        while has_note and (not in_range): # Half-real
            r.sleep()
        if not has_note: # Not real
            return 'lost_note'
        return 'entered_range'

class noteless(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['got_note'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state noteless (alliance color light)')
        send_colour(team_color)
        while not has_note: # Not real
            r.sleep()
        return 'got_note'

if __name__ == '__main__':
    rospy.init_node('leds_state_machine')
    r = rospy.Rate(10)#10 hz
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['termination'])

    # These are not being used rn
    # sm.userdata.is_disabled = True
    # sm.userdata.is_auto = False
    # sm.userdata.auto_mode = 1
    # sm.userdata.has_note = False
    # sm.userdata.in_range = False

    is_disabled = False
    is_auto = False
    team_color = None
    auto_mode = 1
    has_note = False
    in_range = False
    #shooting_distance = rospy.get_param("effective_shooting_range")

    rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, match_data_callback)
    rospy.Subscriber("/auto/auto_mode", AutoMode, auto_mode_callback)
    rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, distance_callback)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('head_state', head_state(), transitions={'disabled': 'disabled', 'auto_state': 'auto_state', 'in_range': 'ready_to_shoot', 'not_in_range': 'out_of_range', 'no_note': 'noteless', 'blue_test_state':'blue_test_state'})

        smach.StateMachine.add('blue_test_state', blue_test_state(), transitions={'head_state':'head_state'})

        smach.StateMachine.add('disabled', disabled(), transitions={'teleop': 'head_state'})
        smach.StateMachine.add('auto_state', auto_state(), transitions={'teleop': 'head_state'})

        smach.StateMachine.add('ready_to_shoot', ready_to_shoot(), transitions={'lost_note': 'noteless', 'left_range': 'out_of_range'})
        smach.StateMachine.add('out_of_range', out_of_range(), transitions={'lost_note': 'noteless', 'entered_range': 'ready_to_shoot'})
        smach.StateMachine.add('noteless', noteless(), transitions={'got_note': 'head_state'})
    # Execute SMACH plan
    outcome = sm.execute()

# Old code if we decide we want it back sometime
# -----------------------------

# class blue_light(smach.State):
#     def __init__(self):
#         smach.State__init__(self, outcomes=['blue_succeed'])

#     def execute(self, userdata):
#         send_colour(0, 255, 0)
#         return 'blue_succeed'

# class red_light(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['red_succeed'])
  
#     def execute(self, userdata):
#         #if self.is_disabled = True
#         #user.disable_output == 1
#         send_colour(255, 0, 0)
#         return 'red_succeed'

# class note_state(smach.State):
#     def __init__(self, userdata):
#         smach.State.__init__(self, outcomes=['has_note'])
#         #subscriber state
#         #self.has_note = False
    
#     #def callback(self, data):
    
#     def execute(self,userdata):
#         '''
#         if self.has_note:
#             make colour obj red...
#             return purple
#         '''
#         send_colour(218, 45, 237) #color is purple #probably light htis up for a few seconds or rail commnds over for a few seconds?
#         return 'has_note'

# class distance_light(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['in_range', 'out_of_range'])
#         self.Subscriber = rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, self.callback)
#         self.Status = 'out_of_range'
#         self.shooting_distance = rospy.get_param("effective_shooting_range")


#     def callback(self, data):
#         distance = msg.distance
#         print("candle_smach: Updating with respect to distance. ")
#         if distance < self.shooting_distance:
#             self.Status = 'in_range'
#         else:
#             print("candle_smach: Not inside shooting range. ")
#             self.Status = 'out_of_range'

#     def execute(self, userdata):
#         rospy.loginfo("CANDLE_SMACH: EXECUTING RANGE CALLBACK")
#         return self.Status
    '''
    Begin at head, head check status of general stuff, if disabled not true, then proceed to range checking as well as purple light
    
    recall that, it doesnt make sense to be in range if we don't have a note



    proceed to purple light, if no note, go back to head
    in purple light, if there is a note, proceed to range check


    in range check, check if in range
    if in range, light green continue until not in range, or not ejected

    if not in range, light white, continue until in rnage, or note ejected



          /> if disable light redlight() -> head()         />  if not in range whitelight() -> head()

    So head_status -> note state, -> if note -> range check  ->  greenlight() -> head()
                               \> if no note -> head                 
       \> if neither bluelight() -> head()              


    i mean really, what we need this to do, is to cycle back to head, and then down to the lower levels
    
    need to figure out the hz rate of all of this
    also need to figure out how to store status values etc...
    '''
