#!/usr/bin/env python3

import rospy
from frc_msgs.msg import MatchSpecificData
import red_to_blue_map.py
import blue_to_red_map.py

def callback(MatchSpecificData):
    if MatchSpecificData.allianceColor == -1:
        if MatchSpecificData.allianceColor == 1:
            t.child_frame_id = "blue_map"
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = 0
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1
        if MatchSpecificData.allianceColor == 0:
            t.child_frame_id = "red_map"
            t.transform.translation.x = 16.541
            t.transform.translation.y = 0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0
            t.transform.rotation.y = 1
            t.transform.rotation.z = 0
            t.transform.rotation.w = 0