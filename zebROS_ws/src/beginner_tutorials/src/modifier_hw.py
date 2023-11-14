#!/usr/bin/env python3
import rospy
from beginner_tutorials.msg import ZebraData
pub = rospy.Publisher("output", ZebraData, queue_size=1)



def other_attributes_callback(zebra_msg):
    if zebra_msg.has_horn:
        zebra_msg.attribute_horn_color = str(input("What color is it's horn? "))
    
    zebra_msg.attribute_wing = (input("Does the zebra have wings (y/n)?")== "y")
    
    zebra_msg.attribute_eye_color = input("what color eyes does the zeeby have? ")
    
    
    pub.publish(zebra_msg)

    #print('recieved')


sub = rospy.Subscriber("output", ZebraData, other_attributes_callback, queue_size=1)

rospy.init_node("zeeby", anonymous=True)
rospy.spin()

