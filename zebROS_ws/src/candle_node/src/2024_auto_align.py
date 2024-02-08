#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from candle_controller_msgs.srv import Colour, ColourRequest
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from time import sleep

def imu_callback(imu):
    global orientation
    orientation = imu.orientation

def wanted_point_callback(pose):
    global wanted_x
    global wanted_y
    global wanted_r
    wanted_x = pose.position.x
    wanted_y = pose.position.y
    wanted_r = euler_from_quaternion(pose.orientation)

def make_colour_obj(start, count, r, g, b):
    colour = ColourRequest()
    colour.start = start
    colour.count = count
    colour.red = r
    colour.green = g
    colour.blue = b
    colour.white = 0
    return colour

def send_colours(x_col, y_col, r_col):
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/colour')
    try:
        colour_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour', Colour)
        # Start/counts should be edited to match the real robot
        x_colour = make_colour_obj(0, 2, x_col[0], x_col[1], x_col[2])
        colour_client(x_colour)
        sleep(0.1) # Cameron's fault
        y_colour = make_colour_obj(2, 2, y_col[0], y_col[1], y_col[2])
        colour_client(y_colour)
        sleep(0.1)
        r_colour = make_colour_obj(4, 2, r_col[0], r_col[1], r_col[2])
        colour_client(r_colour)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

RED = [255, 0, 0]
GREEN = [0, 255, 0]
BLUE = [0, 0, 255]
wanted_x = None
orientation = None
if __name__ == "__main__":
    rospy.init_node("pregame_candle")
    orientation_sub = rospy.Subscriber("/imu/zeroed_imu", Imu, imu_callback)
    wanted_point_sub = rospy.Subscriber("/auto/first_point", geometry_msgs.Pose, wanted_point_callback)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        if wanted_x is not None:
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            trans = tfBuffer.lookup_transform('base_link', 'map', rospy.Time())
            if trans.transform.translation.x - wanted_x < 0.05:
                x_col = GREEN
            elif trans.transform.translation.x > wanted_x:
                x_col = RED
            else:
                x_col = BLUE
            if trans.transform.translation.y - wanted_y < 0.05:
                y_col = GREEN
            elif trans.transform.translation.y > wanted_y:
                y_col = RED
            else:
                y_col = BLUE
            if orientation is not None:
                euler_orientation = euler_from_quaternion(orientation)
                if euler_orientation[2] - wanted_r < 5:
                    r_col = GREEN
                elif euler_orientation[2] > wanted_r:
                    r_col = RED
                else:
                    r_col = BLUE
            send_colours(x_col, y_col, r_col)
        # Way to make it stop? goes here
        r.sleep()