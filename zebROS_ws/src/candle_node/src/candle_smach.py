#!/usr/bin/env python

import rospy
import smach
import smach_ros
import sys
import time
from candle_controller_msgs.srv import Colour, ColourRequest, Animation, AnimationRequest
from frc_msgs.msg import MatchSpecificData
from behavior_actions.msg import AutoMode


def team_colour_callback(msg):
    global disabled
    
    disabled = msg.disabled
    
    if msg.disabled != disabled:
        print("Updating auto LEDs...")
        disabled = msg.disabled

def auto_mode_callback(msg):
    global auto_mode
    
    auto_mode = msg.auto_mode
    
    if msg.auto_mode != auto_mode:
        print("Updating auto LEDs...")
        disabled = msg.disabled

def make_colour_obj(start, count, r, g, b):
    colour = ColourRequest()
    colour.start = start
    colour.count = count
    colour.red = r
    colour.green = g
    colour.blue = b
    colour.white = 0
    return colour

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


# define state team_colour
# This is going to be our default

class team_colour(smach.state):
    def __init__(self):
        smach.State.__init__(self, outcomes=['green_light', 'rainbow_animation', 'termination', 'white_light'], input_keys = ['allianceColor', 'auto_mode']) # Outcomes are where we can go from this state, input keys are items we get from our publishers

    def execute(self,  message):
        rospy.loginfo('Executing state team_colour')
        if message.allianceColor == 0: # If we're red, 
            make_colour_obj(0, 6, 255, 0, 0) # Make us red
        if message.allianceColor == 1: # If we're blue, 
            make_colour_obj(0, 6, 0, 0, 255) # Make us blue
        if message.allianceColor == -1: # If we don't know, 
            make_animation(1, 0, 6, 6) # Go crazy (Strobe)
        if message.auto_mode < 4: # if auto_mode 1-3,
            return 'rainbow_animation' # Go Rainbow
        elif message.auto_mode > 3: # if any other auto mode,
            return 'white_light' # Go White

# define state green_light
# For whenever we pick up a piece

class green_light(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['team_colour'])
        

    def execute(self):
        rospy.loginfo('Executing state green_light, piece acquired')
        make_colour_obj(0, 6, 0, 255, 0) # Green :)
        time.sleep(2) # Leave the green up for a little bit
        return 'team_colour' # Go back to alliance colour

# define state rainbow_animation
# For when we're about to do something cool

class rainbow_animation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['team_colours'],  input_keys = ['Autonomous', 'Disable'])

    def execute(self, message):
        rospy.loginfo('Executing state rainbow_animation, prepare for enlightenment')
        while message.Autonomous == True:
            make_animation(1, 0, 6, 3)
        while message.Disable == True:
            make_animation(1, 0, 6, 3)


class white_light(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['team_colour'], input_keys = ['Autonomous'])
        

    def execute(self, message):
        rospy.loginfo('Executing state white_light, not cool auto :(')
        while message.Autonomous == True:
            make_colour_obj(0, 6, 255, 255, 255) # white :)
        return 'team_colour'
        # Go back to alliance colour

                
        


def main():
    rospy.init_node('leds_state_machine')
    rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, team_colour_callback)
    rospy.Subscriber("/auto/auto_mode", AutoMode, auto_mode_callback)
    rospy.Subscriber("/speaker_align/dist_and_ang")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['termination'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('team_colour', team_colour(), transitions={'piece_pickup':'green_light', 'vibin':'rainbow_animation', 'doom':'termination'}) # in the format ['name of transition':'actual state it goes to']
        smach.StateMachine.add('green_light', green_light(), transitions={'default':'team_colour'})
        smach.StateMachine.add('white_light', white_light(), transitions={'default':'team_colour'})
        smach.StateMachine.add('rainbow_animation', rainbow_animation(), transitions={'default':'team_colour'})

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()