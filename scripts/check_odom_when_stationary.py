import os
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

#Bagfile name from command line
bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

cmd_vel_topic = "/frcrobot_jetson/swerve_drive_controller/cmd_vel"
odom_topic = "/zed_objdetect/odom"
pose_topic = "/zed_objdetect/pose"
md = "/frcrobot_rio/match_data"

vel = [0, 0, 0]
last_stopped = True
time = 0
enabled = False
for topic, msg, t in bag.read_messages([cmd_vel_topic, odom_topic, pose_topic, md]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    #print(m)

    if topic == md:
        time = m["matchTimeRemaining"]
        enabled = m["Enabled"]
        continue

    if topic == cmd_vel_topic:
        vel = [m["linear"]["x"], m["linear"]["y"], m["angular"]["z"]]
        if vel[0] == 0 and vel[1] == 0:
            if not last_stopped:
                print(f"-------- @ {time}")
                last_stopped = True
        else:
            last_stopped = False
        continue
    if vel[0] == 0 and vel[1] == 0 and vel[2] != 0 and enabled:
        if topic == pose_topic:
            #print(f"pose: ({round(m['pose']['position']['x'], 2)}, {round(m['pose']['position']['y'], 2)}) in frame id {m['header']['frame_id']}, {vel[2]} @ {time}")
            print(f"{round(m['pose']['position']['x'], 2)},{round(m['pose']['position']['y'], 2)},{vel[2]},{time}")
        else:
            pass
            #print(f"odom: ({round(m['pose']['pose']['position']['x'], 2)}, {round(m['pose']['pose']['position']['y'], 2)}) in frame id {m['header']['frame_id']} -> {m['child_frame_id']}, {vel[2]} @ {time}")