#!/usr/bin/env python3

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

MODEL_PATH = 'res/models/human_recog.onnx'


class PedestrianDetectionNode:
    def __init__(self) -> None:
        
        logging.log(logging.INFO, 'Engine Created')
        self.bridge = CvBridge()
        self._init_publisher()
        self._init_subscriber()
        self.traffic_engine = ObjectDetection(MODEL_PATH, True)
    def _init_publisher(self):
        self.signal_pub = rospy.Publisher(
            'pedestrian_presence', Int32, queue_size=1)
        self.image_det_pub = rospy.Publisher(
            'camera_top/ped_detected_image', Image, queue_size=1)

    def _init_subscriber(self):
        self.image_sub = rospy.Subscriber(
            'camera_topic/image', Image, self.run_inference, queue_size=1)

    def run_inference(self, data):
        img_data = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #img_data = np.float16(img_data/255.0)
        boxes, scores, classes = self.traffic_engine.infer(img_data, conf_thres=0.4, class_det=0)
        self.draw_boxes(img_data, boxes)
        if len(boxes)>0:
            self.signal_pub.publish(1)
        else:
            self.signal_pub.publish(0)


    def draw_boxes(self, img, bboxes):
        img = cv2.resize(img, (512, 512))
        bboxes = bboxes.astype(int)
        for i in bboxes:
            x1, y1, x2, y2 = i
            img = cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 1)

        return self.image_det_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))


if __name__ == '__main__':
    rospy.init_node('pedestrian_detection',
                    anonymous=True, log_level=rospy.ERROR)
    node_name = rospy.get_name()
    try:
        traffic_sign_detector = PedestrianDetectionNode()
        rospy.spin()
    except KeyboardInterrupt:
        print('Traffic Sign Detection Node Closed')
    except Exception as e:
        print(e)
