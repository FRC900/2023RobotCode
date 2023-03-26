#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Odometry

rospy.init_node('fake_odom_pub')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

r = rospy.Rate(100)

pub = rospy.Publisher("/fake_odometry", Odometry, queue_size=1)
import time
time.sleep(0.25)

# run this: rosrun tf2_ros static_transform_publisher 0.1 0.1 0.0 0.0 0.0 0.0 base_link odom_frame
while not rospy.is_shutdown():
    trans = tfBuffer.lookup_transform("base_link", "odom", rospy.Time(), rospy.Duration(0.1))
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "odom_frame"
    msg.pose.pose.position.x = trans.transform.translation.x
    msg.pose.pose.position.y = trans.transform.translation.y
    msg.pose.pose.position.z = trans.transform.translation.z
    msg.pose.pose.orientation = trans.transform.rotation
    pub.publish(msg)
    r.sleep()