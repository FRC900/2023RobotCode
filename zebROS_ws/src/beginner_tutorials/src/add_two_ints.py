#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from beginner_tutorials.msg import TwoInts

rospy.init_node('additionizer', anonymous=True)

pub = rospy.Publisher('/calculator/calculated', Int64, queue_size=1)

def callback(ints):
    sum_msg = Int64()
    sum_msg.data = ints.a + ints.b
    pub.publish(sum_msg)

sub = rospy.Subscriber('/add/adding', TwoInts, callback, queue_size=1)

rospy.spin()