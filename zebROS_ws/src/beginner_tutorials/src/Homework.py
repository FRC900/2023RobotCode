#!/usr/bin/env python3
import rospy
#from package.msg import Message
from beginner_tutorials.msg import ZebraData

pub = rospy.Publisher("/doom", ZebraData, queue_size=1)

def call(message: ZebraData):
    new_message = message 
    new_message.name = (new_message.name + "!")
    new_message.num_stripes = (new_message.num_stripes + 1)
    new_message.has_horn = True

    pub.publish(new_message)

sub = rospy.Subscriber("/clara/jeremy/claremy", ZebraData, call, queue_size=1)

rospy.init_node("simple", anonymous=True)

rospy.spin()