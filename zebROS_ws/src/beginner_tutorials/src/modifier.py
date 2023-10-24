#!/usr/bin/env python3
import rospy

# from package.msg import Message
from std_msgs.msg import String

pub = rospy.Publisher("/output", String, queue_size=1)

def callback(message):
    s = message.data + " " + input(f"We received {message.data}, what do you want to add? ")

    new_message = String()
    new_message.data = s

    pub.publish(new_message)

sub = rospy.Subscriber("/input", String, callback, queue_size=1)

rospy.init_node("jenroe", anonymous=True)

rospy.spin()