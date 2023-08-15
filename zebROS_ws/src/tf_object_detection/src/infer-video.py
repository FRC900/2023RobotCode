#!/usr/bin/env python3

import argparse
import cv2
from baseYOLO import YOLO900
import timing

USE_CPU_PREPROCESS = False

def main(args: argparse.Namespace) -> None:
    DETECTRON = YOLO900(engine_path=args.engine, device_str=args.device, use_timings=True, regen_trt=False)

    cap = cv2.VideoCapture(args.input_video)
    t = timing.Timings()
    DETECTRON.t = t

    while True:
        t.start('frame')
        t.start('vid')
        ret, bgr = cap.read()
        t.end('vid')
        if not ret:
            break

        if not USE_CPU_PREPROCESS:
            detections = DETECTRON.gpu_preprocess(bgr, debug=args.show).infer() 
        else:
            detections = DETECTRON.cpu_preprocess(bgr, debug=args.show).infer() 

        t.start("viz")
        if args.show:
            cv2.imshow('result', DETECTRON.draw_bboxes())
            t.end('viz')
            key = cv2.waitKey(1) & 0x000000ff
            if key == 27:
                break
        else:
            t.end('viz')

        t.end('frame')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--engine', type=str, help='Engine file')
    parser.add_argument('--input-video', type=str, help='Input video')
    parser.add_argument('--show',
                        action='store_true',
                        help='Show the detection results')
    parser.add_argument('--device',
                        type=str,
                        default='cuda:0',
                        help='TensorRT infer device')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    main(args)