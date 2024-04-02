#!/usr/bin/env python

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




#def team_colour_callback(msg):
#    global disabled
#    
#    disabled = msg.disabled
#    
#    if msg.disabled != disabled:
#        print("Updating auto LEDs...")
#        disabled = msg.disabled
#
#def auto_mode_callback(msg):
#    global auto_mode
#    
#    auto_mode = msg.auto_mode
#    
#    if msg.auto_mode != auto_mode:
#        print("Updating auto LEDs...")
#        disabled = msg.disabled
#
#def distance_callback(msg):
#    global distance  
#
#    distance = msg.distance
#    print("candle_smach: Updating with respect to distance. ")
#    if distance < shooting_distance:
#        print("candle_smach: In shooting range. ")

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
        the_colour = make_colour_obj(9, 2, r_col, g_col, b_col)
        colour_client(the_colour)
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



# define state rainbow_animation
# For when we're about to do something cool

#class rainbow_animation(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['team_colours'],  input_keys = ['Autonomous', 'Disable'])
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state rainbow_animation, prepare for enlightenment')
#        while message.Autonomous == True:
#            make_animation(1, 0, 6, 3)
#        while message.Disable == True:
#            make_animation(1, 0, 6, 3)


#puprple whenever we pick up a piece

class head_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['head_disable', 'head_has_note', 'neither'])
        self.Status = 'neither'
        self.match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.disable_callback) 

        #subscriber to disabling state
        #subscriber to has note state
    
    def disable_callback(self, data):
        if data.Disabled: 
            self.Status = 'head_disable'
        
        else:
            self.Status = 'neither'

    #def note_callback(self, data):
    '''
        if ... :
            self.Status = 'head_has_note'

        else:
            self.Status = 'neither'
    '''

    def execute(self, userdata):
        return self.Status

class blue_light(smach.State):
    def __init__(self):
        smasch.State__init__(self, outcomes=['blue_succeed'])

    def execute(self, userdata):
        send_colours(0, 255, 0)
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
    def __init__(self, userdata):
        smach.State.__init__(self, outcomes=['has_note'])
        #subscriber state
        #self.has_note = False
    
    #def callback(self, data):
    
    def execute(self,userdata):
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
            print("candle_smach: Not inside shooting range. ")
            self.Status = 'out_of_range'

    def execute(self, userdata):
        rospy.loginfo("CANDLE_SMACH: EXECUTING RANGE CALLBACK")
        return self.Status
        

#nah i need to set up some sort of architecture, rn this entire thing is just hardcoded for being in range :skull:


def main():
    rospy.init_node('leds_state_machine')                                                               #need to place these subscribers in side of object                
    #rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, team_colour_callback)
    #rospy.Subscriber("/auto/auto_mode", AutoMode, auto_mode_callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['termination'])

    #termination state
    sm.userdata.note_state = 0 #0 if there is no note inside, 1 if there is a note inside
    sm.userdata.disable_state = 0 #0 if not disabled 1 if disabled

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

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('head_state', head_state(), transitions={'head_disable':'red_light', 'head_has_note':'note_state', 'neither':'blue_light'})

        smach.StateMachine.add('red_light', red_light(), transitions={'red_succeed':'head_state'})
        smach.StateMachine.add('note_state', note_state(), transitions={'has_note':'range_check'}) #is also purple light()
        smach.StateMachine.add('blue_light', blue_light(), transitions={'blue_succeed':'head_state'})

        smach.StateMachine.add('green_light', green_light(), transitions={'green_succeed':'head_state'})
        #smach.StateMachine.add('white_light', white_light(), transitions={'white_succeed':'head_state'})
        smach.StateMachine.add('range_check', distance_light(), transitions={'in_range':'green_light', 'out_of_range':'note_state'})


    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()