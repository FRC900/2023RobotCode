#!/usr/bin/env python3
import rospy
from beginner_tutorials.msg import ZebraData

def main():
    pub = rospy.Publisher("/clara/jeremy/claremy", ZebraData, queue_size=1)
    rospy.init_node("joeren", anonymous=True)

    while not rospy.is_shutdown():
        message = ZebraData()
        message.num_stripes = int(input("What is the number of stripes?"))
        message.has_horn = (input("Does the zebra have a horn? (y/n)") == "y")
        message.location.position.x = 0.0
        message.location.position.y = 0.0
        message.location.position.z = 0.0
        message.name = input("What's its name? ")

        pub.publish(message)

        print("Published message, yay!")

main()