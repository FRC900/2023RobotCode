#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int64
from beginner_tutorials.msg import TwoInts

rospy.init_node("add_two_ints_node", anonymous=True)

pub = rospy.Publisher("/sum", Int64, queue_size=1)

def callback(numbers_msg: TwoInts):
    sum_message = Int64()
    sum_message.data = numbers_msg.a + numbers_msg.b
    pub.publish(sum_message)

sub = rospy.Subscriber("/numbers", TwoInts, callback, queue_size=1)

rospy.spin()