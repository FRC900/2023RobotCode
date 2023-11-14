#!/usr/bin/env python3

import rospy
from beginner_tutorials.msg import ZebraData

def callback(zebra_msg):
    print("The zebra has", zebra_msg.num_stripes, "stripes")
    if zebra_msg.has_horn:
        print("The zebra has a horn.. Or does it?")
    else:
        print("It's a wierd zebra without a horn!")
    print("The zebra is at", zebra_msg.location)
    print(f"The zebra, {zebra_msg.name} has been created!")


sub = rospy.Subscriber("/new_zebra_data", ZebraData, callback, queue_size=1)

rospy.init_node("renjoe", anonymous=True)

rospy.spin()