#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

rospy.init_node('alter_zed_odom', anonymous=True)

topic = rospy.get_param("~topic")
stdev = float(rospy.get_param("~standard_deviation"))

pub = rospy.Publisher(topic + "_overridden", Odometry)

def callback(odom):
    mut = list(odom.pose.covariance)
    for i in range(6):
        mut[i*6+i] = stdev * stdev
        odom.pose.covariance = tuple(mut)
    pub.publish(odom)

rospy.Subscriber(topic, Odometry, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()