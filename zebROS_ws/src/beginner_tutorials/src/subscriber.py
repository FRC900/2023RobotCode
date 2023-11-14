#!/usr/bin/env python3
import rospy
from beginner_tutorials.msg import ZebraData

def callback(zebra_msg):
    print(f"The zebra has {zebra_msg.num_stripes} stripes")
    if zebra_msg.has_horn:
        print("The zebra has a horn")
    else:
        print("The zebra is not a zebracorn")

    print("The zebra is definitely somewhere")

    print(f"The zebra's name is {zebra_msg.name}")

sub = rospy.Subscriber("/doom", ZebraData, callback, queue_size=1)

rospy.init_node("renjoe", anonymous=True)

rospy.spin()
