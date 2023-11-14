#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

pub = rospy.Publisher("/output", String, queue_size=1)

def callback(message):
    s = message.data + " " + input(f"i got in the mail {message.data}, got anything to add? ")

    new_message = String()
    new_message.data = s

    pub.publish(new_message)

sub = rospy.Subscriber("input", String, callback, queue_size=1)

rospy.init_node("/output", anonymous=True)
rospy.spin()
