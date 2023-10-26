#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu

def rezero_accelerator(req):
    global accelerometer
    x = accelerometer.raw_x
    y = accelerometer.raw_y
    z = accelerometer.raw_z

    accelerometer.offset = (
    round(-x / 4),
    round(-y / 4),
    round(-(z - 20) / 4))  # Z should be '20' at 1g (49mg per bit)  

pub = rospy.Publisher("vroom",Imu,queue_size=1)
s = rospy.Service('Zane', Empty, rezero_accelerator)


rospy.init_node("vroom", anonymous=True)


import time
import board
import adafruit_adxl37x

i2c = board.I2C()  # uses board.SCL and board.SDA
global accelerometer
accelerometer = adafruit_adxl37x.ADXL375(i2c)

r = rospy.Rate(10)


while not rospy.is_shutdown():
    message=Imu()
    message.header.stamp = rospy.get_rostime()
    message.linear_acceleration.x = accelerometer.acceleration[0]
    message.linear_acceleration.y = accelerometer.acceleration[1]
    message.linear_acceleration.z = accelerometer.acceleration[2]
    pub.publish(message)
    r.sleep()

    






