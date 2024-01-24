#!/usr/bin/env python3  
import rospy
import math

# Because of transformations

import tf2_ros
import geometry_msgs.msg


rospy.init_node('tf2_turtle_broadcaster')
map_broadcaster = tf2_ros.StaticTransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()

t.header.stamp = rospy.Time.now()
t.header.frame_id = "map"
t.child_frame_id = "blue"
t.transform.translation.x = 0
t.transform.translation.y = 0
t.transform.translation.z = 0
t.transform.rotation.x = 0
t.transform.rotation.y = 0
t.transform.rotation.z = 0
t.transform.rotation.w = 1

map_broadcaster.sendTransform(t)

rospy.spin()