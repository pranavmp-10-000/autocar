import tensorrt
import pycuda
import rospy
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import threading


class TrafficLightDetectionNode:
    def __init__(self) -> None:
        pass