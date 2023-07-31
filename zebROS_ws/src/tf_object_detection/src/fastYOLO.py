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
import torch

from sys import path
path.append('/home/ubuntu/YOLOv8-TensorRT')
from models import TRTModule  # isort:skip
path.append('/home/ubuntu/tensorflow_workspace/2023Game/models')
import timing

from config_frc2023 import OBJECT_CLASSES, COLORS
from models.torch_utils import det_postprocess
from models.utils import blob, letterbox, path_to_list


bridge = CvBridge()
pub, pub_debug, vis = None, None, None
min_confidence = 0.1
global rospack, THIS_DIR, PATH_TO_LABELS
rospack = rospkg.RosPack()
THIS_DIR = os.path.join(rospack.get_path('tf_object_detection'), 'src/')


global init
init = False

yolo_preprocess = cupy.RawKernel(
    r"""
extern "C" __global__
// assumes that the letterbox will be drawn above and below the image rather than on the sides 
// should be fine given we want squares from long rectangles
// 1024x1024 resize still works with this which should be our max sqaure size

// input is bgr
// output is filled with color for letterbox, does bilinear interpolation with a shift to keep aspect ratio, scales 0-1, transposes to all reds, blues and greens
void yolo_preprocess(const float* input, float* output, int oWidth, int oHeight, int iWidth, int iHeight, int rowsToShiftDown) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

	if( x >= oWidth || y >= oHeight ) {
        //printf("failed x %i y %i", x, y);
        return;
    }

    const double new_x = float(x) / float(oWidth) * float(iWidth);
    const double new_y = float(y) / float(oHeight) * float(iHeight);

    int i;

    // loop for R G and B since no float3 :( 
    for (i=0;i<3;i++) {
		const float bx = new_x - 0.5f;
		const float by = new_y - 0.5f;

		const float cx = bx < 0.0f ? 0.0f : bx;
		const float cy = by < 0.0f ? 0.0f : by;

		const int x1 = int(cx);
		const int y1 = int(cy);
			
		const int x2 = x1 >= iWidth - 1 ? x1 : x1 + 1;	// bounds check
		const int y2 = y1 >= iHeight - 1 ? y1 : y1 + 1;
		
		const float samples[4] = {
			input[(y1 * iWidth + x1) * 3 + i],
			input[(y1 * iWidth + x2) * 3 + i],
			input[(y2 * iWidth + x1) * 3 + i],
			input[(y2 * iWidth + x2) * 3 + i]};

		// compute bilinear weights
		const float x1d = cx - float(x1);
		const float y1d = cy - float(y1);

		const float x1f = 1.0f - x1d;
		const float y1f = 1.0f - y1d;

		const float x2f = 1.0f - x1f;
		const float y2f = 1.0f - y1f;

		const float x1y1f = x1f * y1f;
		const float x1y2f = x1f * y2f;
		const float x2y1f = x2f * y1f;
		const float x2y2f = x2f * y2f;

        // add to Y here to move the image down and add the letterbox part
        // 2 - i for bgr to rgb transform
        const int rgb_offset = (oWidth * oHeight); // should be safe from int division as  
        // initally used  + (2 - i) for bgr -> rbg, but for transposing now using rgb_offset * (2 - i)
		output[rgb_offset * (2 - i) + (((y + rowsToShiftDown) * oWidth + x) * 3)] = (samples[0] * x1y1f + samples[1] * x2y1f + samples[2] * x1y2f + samples[3] * x2y2f) / 255;
    }
}
""",
    "yolo_preprocess",
)

gpu_output_buffer = None
# H, W = Engine.inp_info[0].shape[-2:]
engine_H, engine_W = 640, 640


def blob(im, return_seg: bool = False):
    seg = None
    if return_seg:
        seg = im.astype(np.float32) / 255
    im = im.transpose([2, 0, 1])
    im = im[np.newaxis, ...]
    im = np.ascontiguousarray(im).astype(np.float32) / 255
    if return_seg:
        return im, seg
    else:
        return im

def iDivUp(a, b):
    if (a % b != 0):
        return a // (b + 1) # int division to replicate c++
    else:
        return a // b # should be an int


def run_inference_for_single_image(msg):
    global init
    rospy.logwarn("Callback recived!")
    
    ori = bridge.imgmsg_to_cv2(msg, "bgr8")
    print(ori)
    print(type(ori[0][0][0]))
    #new_dtype = np.float32
    #ori = ori.astype(new_dtype)
    #ori = numpy.zeros_like(ori)

    # print(ori)
    #time.sleep(2)
    #ret_im, _, _1 = letterbox(ori)
    #ret_im = cv2.cvtColor(ret_im, cv2.COLOR_BGR2RGB) # standard color conversion
    
    #ret_im = blob(ret_im, return_seg=False) # convert to float, transpose, scale from 0.0->1.0

    #print(ret_im)
    #print(ret_im.shape)
    #cv2.imshow("LAME cpu letterbox", ret_im)
    
    #key = cv2.waitKey(0) & 0x000000ff 

    inital_gpu_image = cupy.asarray(ori, dtype=cupy.float32)

    if not init:
        gpu_output_buffer = cupy.full((engine_H, engine_W, 3), 114, dtype=cupy.float32)
        init = True

    block_size_1d = 256
    block_sqrt = int(math.sqrt(block_size_1d))
    block_size = (block_sqrt, block_sqrt)

    height, width = ori.shape[:2]
    shape = ori.shape[:2]

    print(f"Height {height} Width {width}")

    r = min(engine_W / shape[1], engine_H / shape[0])
    # Compute padding [width, height]
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    # will shift image down this much, and use to determine where to draw letterbox color
    pixels_to_shift_down = engine_H - new_unpad[1]
    pixels_to_shift_down //= 2 # int division in place!

    stream1 = cupy.cuda.stream.Stream()
    with stream1:
        yolo_preprocess(                    # X                      # Y
            (iDivUp(engine_W, block_sqrt), iDivUp(engine_H, block_sqrt)), (block_size), 
            (inital_gpu_image, gpu_output_buffer, new_unpad[0], new_unpad[1], width, height, pixels_to_shift_down))
    
    stream1.synchronize() # for printing, seems to be exactly the same with and without

    print(f"block size {block_size}")
    print(f"X threads {iDivUp(engine_W, block_sqrt)}, Y {iDivUp(engine_H, block_sqrt)}")

    numpy_array = cupy.asnumpy(gpu_output_buffer)
    # convert to ints           
    numpy_array = numpy_array.astype(np.uint8)

    opencv_mat = numpy_array # cv2.cvtColor(numpy_array, cv2.COLOR_RGB2BGR)  # Assuming RGB data
    # wtf is this
    cv2.imshow("super cool gpu letterbox", opencv_mat)
    key = cv2.waitKey(0) & 0x000000ff 

    ori = bridge.imgmsg_to_cv2(msg, "bgr8")
    print(f"Shape of cpu {ret_im.shape} Gpu {opencv_mat.shape}")
    simmilarity_array = np.equal(ret_im, opencv_mat) 
    values, counts = np.unique(simmilarity_array, return_counts=True)
    print(simmilarity_array)
    print(values)
    print(counts)
    print(f"There are {values}")
    print("Images are equal?")
    time.sleep(1)
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
    device = torch.device("cuda:0")

    Engine = TRTModule("FRC2023m.engine", device)
    H, W = Engine.inp_info[0].shape[-2:]

    # set desired output names order
    Engine.set_desired(['num_dets', 'bboxes', 'scores', 'labels'])
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

    sub = rospy.Subscriber(sub_topic, Image, run_inference_for_single_image, queue_size=10000)
    pub = rospy.Publisher(pub_topic, TFDetection, queue_size=1)
    pub_debug = rospy.Publisher("debug_image", Image, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()