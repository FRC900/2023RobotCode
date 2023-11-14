#!/usr/bin/env python3
import rospy
from beginner_tutorials.msg import ZebraData

def main():
    pub = rospy.Publisher("output", ZebraData, queue_size=1)
    rospy.init_node("output", anonymous=True)
    
    while not rospy.is_shutdown():
        message=ZebraData()
        message.num_stripes = int(input("What is the number of stripes? "))
        message.has_horn = (input("Does the zebra have a horn (y/n)?")== "y")
        message.location.position.x = .009
        message.location.position.y = .09
        message.location.position.z = 7
        message.name = input("jgkjfger")
        message.attribute_eye_color = "pik"

        pub.publish(message)

        print("lowkey i published yo message dawg")

main()