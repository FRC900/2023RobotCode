#! /usr/bin/env/python3

# Importing stuff, self-explanatory, but I'll explain it anyway because I'm bored :D
import rospy # If you don't know what this is, either you're on mechanical or VERY behind
import actionlib # Lets us do server stuff
from behavior_actions import Arm, ArmGoal, ArmFeedback, ArmResult # The ".msg" of action servers

class ArmAction(): # Creates ArmAction class
    # Defining our feedback and result stuff (Don't ask why there's an underscore at the front because I don't know either)
    _feedback = ArmFeedback()
    _result = ArmResult()

    def __init__(self, name):
        self._action_name = name # Give it a name, used as a namespace (For people confuse [like me :D], a namespace is pretty much a dictionary that stores different stuff)
        self._as = actionlib.SimpleActionServer(self._action_name, Arm.ArmAction, execute_cb=self.execute_cb, auto_start = False) # Create the Action Server, and set autostart to False bc ros says so
        self._as.start() # "Start up" the server 
   
    def execute_cb(self, goal):
        # Quality of life variables
        r = rospy.Rate(1)
        success = True

        self._feedback.percent_complete = 0 # Set the initial feedback (percent_complete) equal to 0
        rospy.loginfo(f"{self._action_name}: Executing. Moving arm to {ArmGoal}") # This will give info to the person running the server of whats going on
        
        """
        I'm supposed to be ✨moving the motor✨ here
        """

        # Check if the goal is preempted (canceled) and end the action if it is
        if self._as.is_preempt_requested():
                rospy.loginfo(f'{self._action_name}: Preempted')
                self._as.set_preempted()
                success = False # You did not succeed :(
        # Show off your percent_completion value 🥳
        self._as.publish_feedback(self._feedback)
        if success: # Yay you did it :D (This should be self-explanatory)
            self._result.success = True
            rospy.loginfo(f"{self._action_name}: Succeeded")
            self._as.set_succeeded(self._result)

# Run everything
if __name__ == '__main__': 
    rospy.init_node('arm_control', anonymous=True) # Initiates the node (wow really?)
    server = ArmAction(rospy.get_name()) # Pretty much ros finds the name of the code automagically
    rospy.spin() # SpInNnNn (A fancy "while: True" loop)
