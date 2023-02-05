#!/usr/bin/env python3

import rospy
import sys
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

show_video = False

class Video2ROS:
    def __init__(self):
        rospy.init_node('pub_video', anonymous=False)
        rospy.on_shutdown(self.cleanup)
        
        #Default config
        self.filename = "testvid1.mp4"
        self.pub_topic = "/obj_detection/c920/rect_image"
        self.framerate = 30
        self.show_video = True
        self.empty_video = False
        #added ~ to all the topics
        if rospy.has_param('~pub_topic'):
            self.pub_topic = rospy.get_param('~pub_topic')
            rospy.loginfo("Pub topic is " + str(self.pub_topic))
        if rospy.has_param('~filename'):
            self.filename = rospy.get_param('~filename')
            rospy.loginfo("Filename being used is " + str(self.filename))
        if rospy.has_param('~framerate'):
            self.framerate = rospy.get_param('~framerate')
            rospy.loginfo("Framerate being used is " + str( self.framerate))
        if rospy.has_param('~show_video'):
            self.show_video = rospy.get_param('~show_video')
        if rospy.has_param('~empty_video'):
            self.empty_video = rospy.get_param('~empty_video')

        if not self.empty_video:
            rospack = rospkg.RosPack()
            image_path = rospack.get_path('tf_object_detection') + '/src/'
            self.capture = cv2.VideoCapture(image_path + str(self.filename))
        bridge = CvBridge()

        image_pub = rospy.Publisher(self.pub_topic, Image, queue_size=10)
        retfail = 0
        while not rospy.is_shutdown():
            if not self.empty_video:
                ret, frame = self.capture.read()
            else:
                ret = True
                size = 1080, 1920, 3
                frame = np.zeros(size, dtype=np.uint8)

            #If it fails a few times just restarts the video, defintly not to be used on the real robot
            if retfail > 3:
                self.capture = cv2.VideoCapture(image_path + str(self.filename))
                retfail = 0

            #Prevents a error which crashes the program
            if ret == False:

                #print("Frame grab failed?")
                retfail += 1
                continue

            try:
                image_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except CvBridgeError as e:
                print(e)

            if show_video:
                display_image = frame.copy()
                cv2.imshow("Video Playback", display_image)

            self.keystroke = cv2.waitKey(round(1000 / self.framerate))

    def cleanup(self):
        print("Shutting down video pub node.")
        cv2.destroyAllWindows()

def main(args):
    try:
        v2r = Video2ROS()
    except KeyboardInterrupt:
        v2r.cleanup()

if __name__ == '__main__':
    main(sys.argv)
