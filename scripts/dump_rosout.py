import os
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

#Bagfile name from command line
bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

out = "/rosout"
goal = "/align_and_place_grid/goal"
auto = False

last_match_time = 0

for topic, msg, t in bag.read_messages([out, goal]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    if topic == out:
        if "align_to_grid" in m["file"]:
            print(f'{m["msg"]} at {t.to_sec()} = {last_match_time}. auto = {auto}')
    elif topic == goal:
        print(f"{m['goal']['grid_id']} sent ****************************************************************")