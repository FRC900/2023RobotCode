#!/usr/bin/env python3
# write a node that subscribes to talon states and publishes to gazebo topics
# this is a temporary solution until we get the real swerve control working

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
# import talon_state_msgs/TalonState
from talon_state_msgs.msg import TalonState

# sub to /frcrobot_jetson/talon_states
# pub to 8 topics
# /chimera/left_back_angle_setpoint_controller/command
# /chimera/left_back_drive_speed_controller/command
# /chimera/left_front_angle_setpoint_controller/command
# /chimera/left_front_drive_speed_controller/command
# /chimera/right_back_angle_setpoint_controller/command
# /chimera/right_back_drive_speed_controller/command
# /chimera/right_front_angle_setpoint_controller/command
# /chimera/right_front_drive_velocity_controller/command

FL_ANGLE_OFFSET = 0.85
FR_ANGLE_OFFSET = 1.1
BL_ANGLE_OFFSET = 3.15
BR_ANGLE_OFFSET = 1.0

FL_INVERT = -1
FR_INVERT = -1
BL_INVERT = 1
BR_INVERT = 1


class FakeSwerveControl:

    def __init__(self) -> None:
        self.bl_angle_pub = rospy.Publisher("/chimera/left_back_angle_setpoint_controller/command", Float64, queue_size=10)
        self.bl_drive_pub = rospy.Publisher("/chimera/left_back_drive_velocity_controller/command", Float64, queue_size=10)
        self.fl_angle_pub = rospy.Publisher("/chimera/left_front_angle_setpoint_controller/command", Float64, queue_size=10)
        self.fl_drive_pub = rospy.Publisher("/chimera/left_front_drive_velocity_controller/command", Float64, queue_size=10)
        self.br_angle_pub = rospy.Publisher("/chimera/right_back_angle_setpoint_controller/command", Float64, queue_size=10)
        self.br_drive_pub = rospy.Publisher("/chimera/right_back_drive_velocity_controller/command", Float64, queue_size=10)
        self.fr_angle_pub = rospy.Publisher("/chimera/right_front_angle_setpoint_controller/command", Float64, queue_size=10)
        self.fr_drive_pub = rospy.Publisher("/chimera/right_front_drive_velocity_controller/command", Float64, queue_size=10)
        rospy.loginfo("FakeSwerveControl initialized")
        #self.pub_all(0)
        self.rotate_to_offsets()
        rospy.sleep(1)
        self.talon_states_sub = rospy.Subscriber("/frcrobot_jetson/talon_states", TalonState, self.talon_states_callback)

        #self.drive_forward()

    def drive_forward(self):
        for i in range(10):
            self.bl_drive_pub.publish(10000 * BL_INVERT)
            self.fl_drive_pub.publish(10000 * FL_INVERT)
            self.br_drive_pub.publish(10000 * BR_INVERT)
            self.fr_drive_pub.publish(10000 * FR_INVERT)
            rospy.sleep(0.1)

    def pub_all(self, val):
        self.bl_angle_pub.publish(val)
        self.bl_drive_pub.publish(val)
        self.fl_angle_pub.publish(val)
        self.fl_drive_pub.publish(val)
        self.br_angle_pub.publish(val)
        self.br_drive_pub.publish(val)
        self.fr_angle_pub.publish(val)
        self.fr_drive_pub.publish(val)

    def rotate_to_offsets(self):
        for i in range(10):
            self.bl_angle_pub.publish(BL_ANGLE_OFFSET)
            self.fl_angle_pub.publish(FL_ANGLE_OFFSET)
            self.br_angle_pub.publish(BR_ANGLE_OFFSET)
            self.fr_angle_pub.publish(FR_ANGLE_OFFSET)
            rospy.sleep(0.1)

    def pub_all_angle(self, angle: float) -> None:
        # start at 0 and go until 2pi waiting until user presses enter to go to next angle
        start = 0
        end = 2 * 3.14
        step = 0.1
        # BL offset = 3.15
        # FL offset = 0.85
        # BR offset = 1.0
        # FR offset = 1.1
        rospy.loginfo("Publishing angles")
        for i in range(int(start/step), int(end/step)):
            angle = i * step
            self.fr_angle_pub.publish(angle)
            input(f"{angle} Press Enter to continue...")
        rospy.loginfo("Done publishing angles")


    def talon_states_callback(self, msg: JointState) -> None:
        '''
        - bl_angle
        - bl_drive
        - br_angle
        - br_drive
        - elevator_follower
        - elevator_leader
        - fl_angle
        - fl_drive
        - four_bar
        - fr_angle
        - fr_drive
        - intake_leader
        - turret_joint
        '''
        self.bl_angle_pub.publish(msg.set_point[0] + BL_ANGLE_OFFSET)
        self.bl_drive_pub.publish(msg.speed[1] * BL_INVERT)
        self.fl_angle_pub.publish(msg.set_point[6] + FL_ANGLE_OFFSET)
        self.fl_drive_pub.publish(msg.speed[7] * FL_INVERT)
        self.br_angle_pub.publish(msg.set_point[2] + BR_ANGLE_OFFSET)
        self.br_drive_pub.publish(msg.speed[3] * BR_INVERT)
        self.fr_angle_pub.publish(msg.set_point[9] + FR_ANGLE_OFFSET)
        self.fr_drive_pub.publish(msg.speed[10] * FR_INVERT)
        rospy.loginfo(f"Inital BL Angle: {msg.set_point[0]} Final BL Angle: {msg.set_point[0] + BL_ANGLE_OFFSET}")
        rospy.loginfo(f"Inital FL Angle: {msg.set_point[6]} Final FL Angle: {msg.set_point[6] + FL_ANGLE_OFFSET}")
        rospy.loginfo(f"Inital BR Angle: {msg.set_point[2]} Final BR Angle: {msg.set_point[2] + BR_ANGLE_OFFSET}")
        rospy.loginfo(f"Inital FR Angle: {msg.set_point[9]} Final FR Angle: {msg.set_point[9] + FR_ANGLE_OFFSET}")

        rospy.loginfo_throttle(1, "Published to all topics")

if __name__ == "__main__":
    rospy.init_node("fake_swerve_control")
    fake_swerve_control = FakeSwerveControl()
    rospy.spin()