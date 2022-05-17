#!/usr/bin/env python3

from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraNode:
    def __init__(self):
        self.bridge = CvBridge()
        self._init_publisher()
        self._init_subscriber()

    def _init_publisher(self):
        self.cam_pub = rospy.Publisher(
            'camera_topic/image', Image, queue_size=4)

    def _init_subscriber(self):
        self.cam_sub = rospy.Subscriber(
            'cv_camera/image_raw', Image, self.process_image)

    def process_image(self, image):
        image = self.bridge.imgmsg_to_cv2(image)
        image = cv2.resize(image, (512, 512))
        # #image = image.astype(np.float16)
        # pred_inp_img = image[np.newaxis, :, :, :]
        # pred_inp_img = np.ascontiguousarray(np.rollaxis(pred_inp_img, 3, 1))
        process_image = self.bridge.cv2_to_imgmsg(image,'bgr8')
        self.cam_pub.publish(process_image)


if __name__ == '__main__':
    rospy.init_node('autocar_camera_node', anonymous=True)
    try:
        camera_node = CameraNode()
        rospy.loginfo("Camera node has started")
        rospy.spin()
    except Exception as e:
        print(e)
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard interrupteds')
