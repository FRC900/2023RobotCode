#! /usr/bin/env python3

# Ordering imports by length is cool
import os
import sys
import cv2
import time
import uuid
import cupy
import rospy
import torch
import rospkg
import numpy as np
from sensor_msgs.msg import Image
from file_changed import file_changed
from visualization import BBoxVisualization
from cv_bridge import CvBridge, CvBridgeError
from field_obj.msg import TFDetection, TFObject
import math
import sys
import numpy
from sys import path
import argparse
import cv2
from baseYOLO import YOLO900

bridge = CvBridge()
pub, pub_debug, vis = None, None, None
min_confidence = 0.1
global rospack, THIS_DIR, PATH_TO_LABELS, DETECTRON

rospack = rospkg.RosPack()
THIS_DIR = os.path.join(rospack.get_path('tf_object_detection'), 'src/')
init = False
# // all caps to show its important
DETECTRON: YOLO900 = None

def run_inference_for_single_image(msg):
    global init, Engine, engine_W, engine_H, ratio, dwdh, gpu_output_buffer
    rospy.logwarn("Callback recived!")
    debug = False
    if pub_debug.get_num_connections() > 0:
        debug = True
    
    ori = bridge.imgmsg_to_cv2(msg, "bgr8")
    
    # type hinting is really nice here
    # also looks so much nicer having the logic somewhere else from the ros code
    detections = DETECTRON.gpu_preprocess(ori, debug=True).infer()
    debug_image = DETECTRON.draw_bboxes()    

    pub_debug.publish(bridge.cv2_to_imgmsg(debug_image, encoding="bgr8"))


    height, width, channels = ori.shape
    boxes = []
    confs = []
    clss = []
    detection = TFDetection()
    detection.header = msg.header
    # Will end transformed to optical frame in screen to world
    # detection.header.frame_id = detection.header.frame_id.replace("_optical_frame", "_frame")
    
    # Converts numpy array to list becuase extracting indviual items from a list is faster than numpy array
    # Might be a more optimized way but this takes negligible time
    ''' 
    output = output.tolist()
    debug = False
    if pub_debug.get_num_connections() > 0:
        debug = True
    for i in range(int(len(output) / model.layout)):
        prefix = i * model.layout

        conf = float(output[prefix + 2])
        if conf < min_confidence:
            continue

        index = int(output[prefix + 1])
        label = str(category_dict[int(output[prefix + 1])])
        #conf = float(output[prefix + 2])
        xmin = float(output[prefix + 3] * width)
        ymin = float(output[prefix + 4] * height)
        xmax = float(output[prefix + 5] * width)
        ymax = float(output[prefix + 6] * height)

        # ROSifying the code
        obj = TFObject()
        obj.confidence = conf
        obj.tl.x = xmax
        obj.tl.y = ymax
        obj.br.x = xmin
        obj.br.y = ymin
        obj.id = index
        obj.label = label
        detection.objects.append(obj)
        if debug == True:
            boxes.append([output[prefix + 4], output[prefix + 3], output[prefix + 6], output[prefix + 5]])
            clss.append(int(output[prefix + 1]))
            confs.append(output[prefix + 2])
    pub.publish(detection)
    #Visualize
    if debug == True:
        viz.draw_bboxes(ori, boxes, confs, clss, min_confidence)
        pub_debug.publish(bridge.cv2_to_imgmsg(ori, encoding="bgr8"))

        #cv2.imwrite("result.jpg", ori)
        #cv2.imshow("result", ori)
        #key = cv2.waitKey(.5) & 0x000000FF
    '''


def main():
    global pub, category_index, pub_debug, min_confidence, vis, DETECTRON

    os.chdir(THIS_DIR)
    DETECTRON = YOLO900()

    sub_topic = "/obj_detection/c920/rect_image"
    pub_topic = "obj_detection_msg"
    rospy.init_node('tf_object_detection', anonymous=True)
    min_confidence = 0.1

    if rospy.has_param('min_confidence'):
        min_confidence = rospy.get_param('min_confidence')
        rospy.loginfo("Min confidence of " + str(min_confidence) + " loaded from config")
    else:
        rospy.logwarn("Unable to get min confidence, defaulting to 0.1")

    if rospy.has_param('image_topic'):
        sub_topic = rospy.get_param('image_topic')
        rospy.loginfo("Image topic of " + str(sub_topic) + " loaded from config")
    else:
        rospy.logwarn("Unable to get image topic, defaulting to c920/rect_image")

    sub = rospy.Subscriber(sub_topic, Image, run_inference_for_single_image, queue_size=10000)
    pub = rospy.Publisher(pub_topic, TFDetection, queue_size=1)
    pub_debug = rospy.Publisher("debug_image", Image, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()