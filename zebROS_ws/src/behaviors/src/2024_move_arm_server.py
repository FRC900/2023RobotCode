#!/usr/bin/env python3

# Importing stuff, self-explanatory, but I'll explain it anyway because I'm bored :D
import rospy # If you don't know what this is, either you're on mechanical or VERY behind
import actionlib # Lets us do server stuff
from behavior_actions.msg import Arm2024Action, Arm2024Goal, Arm2024Feedback, Arm2024Result # The ".msg" of action servers
from std_msgs.msg import Float64
from talon_state_msgs.msg import TalonFXProState

class ArmAction(): # Creates ArmAction class
    # Defining our feedback and result stuff (Don't ask why there's an underscore at the front because I don't know either)
    _feedback = Arm2024Feedback()
    _result = Arm2024Result()

    def __init__(self, name):
        self.rot_pos_goal = Float64()
        self.sub = rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.callback) # ASK
        self.diverter_position = rospy.get_param("diverter_position")
        self.amp_position = rospy.get_param("amp_position")
        self.trap_position = rospy.get_param("trap_position")
        self.pub = rospy.Publisher('/frcrobot_jetson/arm_controller/command', Float64, queue_size=1)
        self._action_name = name # Give it a name, used as a namespace (For people confuse [like me :D], a namespace is pretty much a dictionary that stores different stuff)
        self._as = actionlib.SimpleActionServer(self._action_name, Arm2024Action, execute_cb=self.execute_cb, auto_start = False) # Create the Action Server, and set autostart to False bc ros says so
        self._as.start() # "Start up" the server
        

        
    def callback(self, data):
        for i in range(len(data.name)):
            if data.name[i] == "arm":
                self.rot_pos = data.position[i]

    def execute_cb(self, goal):
        # Quality of life variables
        r = rospy.Rate(20) 
        success = True
        start_rot_pos = self.rot_pos

        rospy.loginfo(f"{self._action_name}: Executing. Moving arm to {Arm2024Goal}") # This will give info to the person running the server of whats going on

        if goal.path == goal.DIVERTER:
            self.rot_pos_goal.data = self.diverter_position
            

        elif goal.path == goal.AMP:
            self.rot_pos_goal.data = self.amp_position
        

        elif goal.path == goal.TRAP: 
            self.rot_pos_goal.data = self.trap_position
        self.pub.publish(self.rot_pos_goal)
        
        while success:
            if rospy.is_shutdown():
                break
            self._feedback.percent_complete = ((self.rot_pos - start_rot_pos)/(self.rot_pos_goal.data - start_rot_pos)) * 100 # Let the feedback be equal to the position of the motor divided by where the motor wants to be. Multiply by 100 to make it a percent.

            # Check if the goal is preempted (canceled) and end the action if it is
            self._as.publish_feedback(self._feedback)
            if self._as.is_preempt_requested():
                    rospy.loginfo(f'{self._action_name}: Preempted')
                    self._as.set_preempted()
                    success = False # You did not succeed :(
            # Show off your percent_completion value 🥳
            percent_difference = ((abs(self.rot_pos - self.rot_pos_goal.data))/((self.rot_pos + self.rot_pos_goal.data)/2)) * 100
            if (percent_difference < 1): # Yay you did it :D (This should be self-explanatory)
                self._result.success = True
                self._feedback.percent_complete = 100.00
                self._as.publish_feedback(self._feedback)
                rospy.loginfo(f"{self._action_name}: Succeeded")
                self._as.set_succeeded(self._result)
                break
            r.sleep()

# Run everything
if __name__ == '__main__': 
    rospy.init_node('2024_move_arm_server', anonymous=True) # Initiates the node (wow really?)
    server = ArmAction(rospy.get_name()) # Pretty much ros finds the name of the code automagically
    rospy.spin() # SpInNnNn (A fancy "while: True" loop)