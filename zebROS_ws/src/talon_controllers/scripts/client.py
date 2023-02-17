#! /usr/bin/env python3


import rospy
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.msg import ConfigDescription



speed_fr = Client('/frcrobot_jetson/swerve_drive_controller/speed_joint_fr')
speed_fl = Client('/frcrobot_jetson/swerve_drive_controller/speed_joint_fl')
speed_br = Client('/frcrobot_jetson/swerve_drive_controller/speed_joint_br')
speed_bl = Client('/frcrobot_jetson/swerve_drive_controller/speed_joint_bl')


steering_fr = Client('/frcrobot_jetson/swerve_drive_controller/steering_joint_fr')
steering_fl = Client('/frcrobot_jetson/swerve_drive_controller/steering_joint_fl')
steering_br = Client('/frcrobot_jetson/swerve_drive_controller/steering_joint_br')
steering_bl = Client('/frcrobot_jetson/swerve_drive_controller/steering_joint_bl')



#ishowspeed

def handle_config_changes_motor_set_speed_joints(config_dict):
    speed_fr.update_configuration(config_dict)
    speed_fl.update_configuration(config_dict)
    speed_br.update_configuration(config_dict)
    speed_bl.update_configuration(config_dict)

def handle_config_changes_motor_set_steering_joints(config_dict):
    steering_fr.update_configuration(config_dict)
    steering_fl.update_configuration(config_dict)
    steering_br.update_configuration(config_dict)
    steering_bl.update_configuration(config_dict)


#set of clients motors
# the function is called for each different motor set when needed.




#another set of client motors

#def handle_config_changes_motor_set_2(config_dict):
#    client5.update_configuration(config_dict)
#    client6.update_configuration(config_dict)
#    client7.update_configuration(config_dict)
#    client8.update_configuration(config_dict)

# the function is called for each different motor set when needed.



#another set of client motors


def main():
    rospy.init_node('my_node')
    rospy.Subscriber('/talon_reconfigure_speed/parameter_updates',ConfigDescription, handle_config_changes_motor_set_speed_joints)
    
    rospy.Subscriber('/talon_reconfigure_server_steering/parameter_updates', ConfigDescription, handle_config_changes_motor_set_steering_joints)
    rospy.spin()

main()
