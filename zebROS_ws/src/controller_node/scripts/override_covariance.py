#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

rospy.init_node('override_covariance', anonymous=True)

topic = rospy.get_param("~topic")

standard_deviation_names = ["x", "y", "z", "x_rotation", "y_rotation", "z_rotation", "x_velocity", "y_velocity", "z_velocity", "x_acceleration", "y_acceleration", "z_acceleration"]

variances = {}
for name in standard_deviation_names:
    if rospy.has_param(f"~standard_deviation/{name}"):
        stdev = float(rospy.get_param(f"~standard_deviation/{name}"))
        variances[name] = stdev * stdev

string_type = rospy.get_param("~type") # Imu or Odometry currently

msg_type = eval(string_type)

pub = rospy.Publisher(topic + "_overridden", msg_type)

def callback(msg):
    if msg_type == Odometry:
        assert type(msg) == Odometry
        matrix = list(msg.pose.covariance)
        if "x" in variances:
            matrix[0*6+0] = variances["x"]
        if "y" in variances:
            matrix[1*6+1] = variances["y"]
        if "z" in variances:
            matrix[2*6+2] = variances["z"]
        if "x_rotation" in variances:
            matrix[3*6+3] = variances["x_rotation"]
        if "y_rotation" in variances:
            matrix[4*6+4] = variances["y_rotation"]
        if "z_rotation" in variances:
            matrix[5*6+5] = variances["z_rotation"]
        msg.pose.covariance = matrix
        pub.publish(msg)
        return
    elif msg_type == Imu:
        assert type(msg) == Imu
        orientation_covariance = list(msg.orientation_covariance)
        if "x_rotation" in variances:
            orientation_covariance[0*3+0] = variances["x_rotation"]
        if "y_rotation" in variances:
            orientation_covariance[1*3+1] = variances["y_rotation"]
        if "z_rotation" in variances:
            orientation_covariance[2*3+2] = variances["z_rotation"]
        angular_velocity_covariance = list(msg.angular_velocity_covariance)
        if "x_velocity" in variances:
            angular_velocity_covariance[0*3+0] = variances["x_velocity"]
        if "y_velocity" in variances:
            angular_velocity_covariance[1*3+1] = variances["y_velocity"]
        if "z_velocity" in variances:
            angular_velocity_covariance[2*3+2] = variances["z_velocity"]
        linear_acceleration_covariance = list(msg.linear_acceleration_covariance)
        if "x_acceleration" in variances:
            linear_acceleration_covariance[0*3+0] = variances["x_acceleration"]
        if "y_acceleration" in variances:
            linear_acceleration_covariance[1*3+1] = variances["y_acceleration"]
        if "z_acceleration" in variances:
            linear_acceleration_covariance[2*3+2] = variances["z_acceleration"]
        msg.orientation_covariance = orientation_covariance
        msg.angular_velocity_covariance = angular_velocity_covariance
        msg.linear_acceleration_covariance = linear_acceleration_covariance
        pub.publish(msg)
    else:
        print("Unrecognized message type! Check your configuration.")
        

rospy.Subscriber(topic, msg_type, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()





# path = type.split("/")

# print(f"Using message type {path[0]}/{path[1]}")

# exec(f"from {path[0]}.msg import {path[1]}")

# msg = eval(f"{path[1]}")

# print(msg._slot_types)

# """
# Start at base
# Go to each slot, tracking its name and path as it gets appended
# If there is a field with a slot name that is covariance and is a list
# Then modify it to have the desired std. dev.
# """

# def print_slots(m):
#     print(f"{m}:")
#     for slot in m.__slots__:
#         try:
#             print_slots(eval(f"m.{slot}"))
#         except:
#             continue

# print_slots(msg)
