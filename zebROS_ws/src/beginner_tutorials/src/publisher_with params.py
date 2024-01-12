#!/usr/bin/env python3
import rospy
from beginner_tutorials.msg import ZebraData

pub = rospy.Publisher(rospy.get_param("pub_topic"), ZebraData, queue_size=1)
rospy.init_node("publisher_node", anonymous=True)

while not rospy.is_shutdown():
    message = ZebraData()
    message.num_stripes = int(input("What is the number of stripes? "))
    message.has_horn = (input("Does the zebra have a horn (y/n)?") == "y")
    message.location.position.x = rospy.get_param("x_value")
    message.location.position.y = rospy.get_param("y_value")
    message.location.position.z = rospy.get_param("z_value")
    message.name = input("What's its name? ")
    
    pub.publish(message)

    print("Published message, yay!")










