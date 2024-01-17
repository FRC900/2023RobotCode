#!/usr/bin/env python3

# This node autonomously targets and drives to an object detected using vision.
# We've already done the targeting part semi-successfully in the past for teleop, see teleop_joystick_control/src/2023_snap_to_nearest_conecube.cpp for an example
# However, we want to be able to do this in auto also ideally (which requires driving)
# And it's easier for the driver in teleop if they can just press a button to have the robot align to a game piece
# Detections will be published on the topic /tf_object_detection/object_detection_world
# I'm not currently sure of the best way to drive toward the object. For a note, we need to be facing it, so some ideas are:
# - turn to face the object then drive forward (while our heading is locked on to the object)
# - follow a very simple object-relative path (e.g. have a point 0.5m behind the object and then a point at the object), see nav_msgs/Path for a message type we could use

# Goal (at minimum):
# - name of object to target (e.g. "note")
# - the object-relative path if we decide to go that route
# Feedback (at minimum):
# - linear distance away from object
# - angular distance away from object
# Result:
# - success or failure