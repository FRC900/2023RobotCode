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
from adafruit_adxl34x import DataRate

i2c = board.I2C()  # uses board.SCL and board.SDA
global accelerometer
accelerometer = adafruit_adxl37x.ADXL375(i2c)
accelerometer.data_rate  = DataRate.RATE_800_HZ

r = rospy.Rate(800)


while not rospy.is_shutdown():
    message=Imu()
    message.header.stamp = rospy.get_rostime()
    x,y,z = accelerometer.acceleration
    message.linear_acceleration.x = x
    message.linear_acceleration.y = y
    message.linear_acceleration.z = z
    pub.publish(message)
    r.sleep()

    






