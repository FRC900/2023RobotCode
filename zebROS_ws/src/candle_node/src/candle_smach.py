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
   in_range = msg.distance < shooting_distance

def make_colour_obj(start, count, r, g, b):
    colour = ColourRequest() #colourrequest  isn't a actual srv file? 
    colour.start = start
    colour.count = count
    colour.red = r
    colour.green = g
    colour.blue = b
    colour.white = 0
    return colour

def send_colours(r_col, g_col, b_col):
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/colour')
    try:
        colour_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour', Colour)
        # Start/counts should be edited to match the real robot
        colour_client(make_colour_obj(9, 2, r_col, g_col, b_col))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def make_animation(speed, start, count, animation_type):
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
    return animation

class head_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['boring_auto', 'cool_auto', 'in_range', 'not_in_range', 'no_note'])
        
    #     self.match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.match_data_callback) # Consider making this global-ler
    #     self.Status = 'enabled'

    #     #subscriber to disabling state
    #     #subscriber to has note state

    '''
    def note_callback(self, data):
        if ... :
            self.Status = 'head_has_note'

        else:
            self.Status = 'neither'
    '''

    def execute(self, userdata):
        if is_disabled or is_auto: # Do we want something different if god forbid we disable during the match?
            if auto_mode > 4:
                return 'boring_auto'
            return 'cool_auto'
        return self.Status

class blue_light(smach.State):
    def __init__(self):
        smach.State__init__(self, outcomes=['blue_succeed'])

    def execute(self, userdata):
        rospy.loginfo("inside callback blue light")
        send_colours(0, 255, 0)
        rospy.loginfo("sent the blue values inside blue light")
        return 'blue_succeed'

class red_light(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['red_succeed'])
  
    def execute(self, userdata):
        #if self.is_disabled = True
        #user.disable_output == 1
        send_colours(255, 0, 0)
        return 'red_succeed'

class note_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['has_note'])
        #subscriber state
        #self.has_note = False
    
    #def callback(self, data):
    
    def execute(self, userdata):
        '''
        if self.has_note:
            make colour obj red...
            return purple
        '''
        send_colours(218, 45, 237) #color is purple #probably light htis up for a few seconds or rail commnds over for a few seconds?
        return 'has_note'

class white_light(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['white_succeed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state white_light, not cool auto :(')
        send_colours(255, 255, 255) # white :)
        return 'white_succeed'
        # Go back to alliance colour

# define state green_light
class green_light(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['green_succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state green_light, piece acquired')
        send_colours(0, 255, 0) # Green :)
        #time.sleep(2) # Leave the green up for a little bit
        return 'green_succeed' # Go back to alliance colour


class distance_light(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['in_range', 'out_of_range'])
        self.Subscriber = rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, self.callback)
        self.Status = 'out_of_range'
        self.shooting_distance = rospy.get_param("effective_shooting_range")


    def callback(self, data):
        distance = msg.distance
        print("candle_smach: Updating with respect to distance. ")
        if distance < self.shooting_distance:
            self.Status = 'in_range'
        else:
            if has_note: # Not real
                if in_range:
                    return 'in_range'
                return 'not_in_range'
            return 'no_note'

class cool_auto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state cool_auto (rainbow), prepare for enlightenment')
        make_animation(1, 0, 6, 3)
        while (is_disabled or is_auto): # A bit fishy
            r.sleep()
        return 'teleop'

class boring_auto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state boring_auto, white light :')
        send_colours(255, 255, 255) # white :)
        while (is_disabled or is_auto): # A bit fishy
            r.sleep()
        return 'teleop'

class ready_to_shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_note', 'left_range'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state ready_to_shoot (green light), piece acquired')
        send_colours(0, 255, 0) # Green :)
        while has_note and in_range: # Half-real
            r.sleep()
        if not has_note: # Not real
            return 'lost_note'
        return 'left_range'

class out_of_range(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_note', 'entered_range'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state out_of_range (orange light)')
        send_colours(255, 165, 0) # Orange :D
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
        send_colours(team_color)
        while not has_note: # Not real
            r.sleep()
        return 'got_note'

if __name__ == '__main__':
    r = rospy.Rate(10)
    rospy.init_node('leds_state_machine')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['termination'])

    # These are not being used rn
    # sm.userdata.is_disabled = True
    # sm.userdata.is_auto = False
    # sm.userdata.auto_mode = 1
    # sm.userdata.has_note = False
    # sm.userdata.in_range = False
    #termination state
    #sm.userdata.note_state = 0 #0 if there is no note inside, 1 if there is a note inside
    #sm.userdata.disable_state = 0 #0 if not disabled 1 if disabled

    is_disabled = False
    is_auto = False
    team_color = None
    auto_mode = 1
    has_note = False
    in_range = False
    shooting_distance = rospy.get_param("effective_shooting_range")

    rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, match_data_callback)
    rospy.Subscriber("/auto/auto_mode", AutoMode, auto_mode_callback)
    rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, distance_callback)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('head_state', head_state(), transitions={'boring_auto': 'boring_auto', 'cool_auto': 'cool_auto', 'in_range': 'ready_to_shoot', 'not_in_range': 'out_of_range', 'no_note': 'noteless'})
        smach.StateMachine.add('boring_auto', boring_auto(), transitions={'teleop': 'head_stat'})
        smach.StateMachine.add('cool_auto', cool_auto(), transitions={'teleop': 'head_state'})
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
#         send_colours(0, 255, 0)
#         return 'blue_succeed'

# class red_light(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['red_succeed'])
  
#     def execute(self, userdata):
#         #if self.is_disabled = True
#         #user.disable_output == 1
#         send_colours(255, 0, 0)
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
#         send_colours(218, 45, 237) #color is purple #probably light htis up for a few seconds or rail commnds over for a few seconds?
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
    '''

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('head_state', head_state(), transitions={'head_disable':'red_light', 'head_has_note':'note_state', 'neither':'blue_light'})
        smach.StateMachine.add('red_light', red_light(), transitions={'red_succeed':'head_state'})
        smach.StateMachine.add('note_state', note_state(), transitions={'has_note':'range_check'}) #is also purple light()
        smach.StateMachine.add('blue_light', blue_light(), transitions={'blue_succeed':'head_state'})
        smach.StateMachine.add('green_light', green_light(), transitions={'green_succeed':'head_state'})
        smach.StateMachine.add('range_check', distance_light(), transitions={'in_range':'green_light', 'out_of_range':'note_state'})


    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
