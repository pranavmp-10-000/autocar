#!/usr/bin/env python3

import tensorrt
import pycuda
import rospy
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import threading
from libs.cv_libs import check_signal_state
from libs.onnx_inference import ObjectDetection
import logging

MODEL_PATH  = 'res/models/human_recog.onnx'

class TrafficLightDetectionNode:
    def __init__(self) -> None:
        self.traffic_engine = ObjectDetection(MODEL_PATH,True)
        logging.log(logging.INFO,'Engine Created')
        self.bridge = CvBridge()
        self._init_publisher()
        self._init_subscriber()

    def _init_publisher(self):
        self.signal_pub = rospy.Publisher('signal_state',Int32, queue_size=2)
        self.image_det_pub = rospy.Publisher('camera_top/detected_image',Image,queue_size=2)
    
    def _init_subscriber(self):
        self.image_sub = rospy.Subscriber('cv_camera/image_raw', Image, self.run_inference,queue_size=2)

    def run_inference(self, data):
        img_data = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        boxes, scores, classes = self.traffic_engine.infer(img_data)
        signal_state = check_signal_state(img_data, boxes)
        self.draw_boxes(img_data, boxes)

    def draw_boxes(self,img, bboxes):
        img = cv2.resize(img,(512,512))
        bboxes = bboxes.astype(int)
        for i in bboxes:
            x1, y1, x2, y2 = i
            img = cv2.rectangle(img,(x1,y1), (x2,y2), (0,255,255),1)
        
        return self.image_det_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

if __name__=='__main__':
    rospy.init_node('traffic_light_detection', anonymous=True, log_level= rospy.ERROR)
    node_name = rospy.get_name()
    try:
        traffic_sign_detector = TrafficLightDetectionNode()
        rospy.spin()
    except KeyboardInterrupt:
        print('Traffic Sign Detection Node Closed')
    except Exception as e:
        print(e)
