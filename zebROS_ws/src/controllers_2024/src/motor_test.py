# not the actual shooter controller, but probably the fastest way to get running

import rospy
import std_msgs.msg


percent = 0

def percent_cb(msg):
    global percent

# takes in speed in rad/s and percent from 0 to 1 
def speed_cb(msg):
    speed = msg.data
    left_speed = speed + speed * percent
    right_speed = speed - speed * percent

    pass
    


rospy.init_node("motor_test")
rospy.Publisher

rospy.Subscriber("/shooter_speed", std_msgs.msg.Float64, speed_cb)
rospy.Subscriber("/shooter_percent", std_msgs.msg.Float64, percent_cb)
rospy.spin()