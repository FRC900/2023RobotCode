#! /usr/bin/env python3
"""
Adaped from	https://github.com/AastaNV/TRT_object_detection.git
A faster way to optimize models to run on the Jetson
This script has 2 parts. First is to convert the model to UFF format and then
optimize that using tensorRT.  This produces a .bin file.
The .bin file is then loaded and used to run inference on a video.
"""
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

bridge = CvBridge()
pub, pub_debug, vis = None, None, None
min_confidence = 0.1
global rospack, THIS_DIR, PATH_TO_LABELS
rospack = rospkg.RosPack()
THIS_DIR = os.path.join(rospack.get_path('tf_object_detection'), 'src/')

device = torch.device("cuda:0")

global init
init = False

yolo_preprocess = cupy.RawKernel(
    r"""
extern "C" __global__
void cupy_custom_kernel_fwd(const float* input, float* output, int size) {
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < size)
        y[tid] = log(x[tid]);
}
""",
    "yolo_preprocess",
)

gpu_output_buffer = None
# H, W = Engine.inp_info[0].shape[-2:]
engine_H, engine_W = (640, 640)


def show_img(img):
    cv2.imshow('result', img)
    key = cv2.waitKey(1) & 0x000000ff


def iDivUp(a, b):
    if (a % b != 0):
        return a // (b + 1) # int division to replicate c++
    else:
        return a / b # should be an int

def run_inference_for_single_image(msg):
    rospy.logwarn("Callback recived!")
    
    if not init:
        ori = bridge.imgmsg_to_cv2(msg, "bgr8")
        show_img(ori)
        inital_gpu_image = cupy.asarray(ori, dtype=cupy.float32) 
        print(inital_gpu_image.dtype)
        print(inital_gpu_image)
        print(inital_gpu_image.shape)
        inital_gpu_image 
        gpu_output_buffer = cupy.empty((engine_H, engine_W, 3), dtype=cupy.float32)
        output_size = gpu_output_buffer.size

        block_size = 64 # 32, 128? idk
        yolo_preprocess(
            (block_size,), ((output_size + block_size - 1) // block_size,), 
            (inital_gpu_image, gpu_output_buffer, x_size)
        )       
        exit()

    ori = bridge.imgmsg_to_cv2(msg, "bgr8")
    # Trying with gpu
    

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

    os.chdir(THIS_DIR)

    global pub, category_index, pub_debug, min_confidence, vis
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

    sub = rospy.Subscriber(sub_topic, Image, run_inference_for_single_image, queue_size=1)
    pub = rospy.Publisher(pub_topic, TFDetection, queue_size=1)
    pub_debug = rospy.Publisher("debug_image", Image, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()