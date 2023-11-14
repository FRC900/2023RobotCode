#!/usr/bin/env python3

import rospy
from beginner_tutorials.msg import ZebraData

pub = rospy.Publisher("/new_zebra_data", ZebraData, queue_size=1)

def callback(new_message):
    print("e")
    new_message.num_stripes = int(input(f"{new_message.num_stripes} was recieved. What do you want to change it to? "))
    new_message.has_horn = bool(input(f"{new_message.has_horn} was recieved. What do you want to change it to? "))
    new_message.location.position.x = int(input(f"{new_message.location.position.x} was recieved. What do you want to change it to? "))
    new_message.location.position.y = int(input(f"{new_message.location.position.y} was recieved. What do you want to change it to? "))
    new_message.location.position.z = int(input(f"{new_message.location.position.z} was recieved. What do you want to change it to ? "))
    new_message.name = input(f"{new_message.name} was recieved. What do you want to change it to? ")
    pub.publish(new_message)
sub = rospy.Subscriber("/clara/jeremy/claremy", ZebraData, callback, queue_size=1)

rospy.init_node("joeren", anonymous=True)

rospy.spin()