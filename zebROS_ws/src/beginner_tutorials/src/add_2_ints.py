#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int64
from beginner_tutorials.msg import TwoInts

pub = rospy.Publisher("/sum", Int64, queue_size=1)

def callback(numbers_msg):
    sum_message = Int64()
    sum_message.data = numbers_msg.a + numbers_msg.b + 2
    pub.publish(sum_message)

sub = rospy.Subscriber("/numbers", TwoInts, callback, queue_size=1)

rospy.init_node("add_2_ints_node", anonymous=True)

rospy.spin()