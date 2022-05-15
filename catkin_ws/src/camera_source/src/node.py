#!/usr/bin/env python3

from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge

class CameraNode:
    def __init__(self):
        self.bridge = CvBridge()
        self._init_publisher()
        self._init_subscriber()
    def _init_publisher(self):
        self.cam_pub = rospy.Publisher('camera_topic/image', Image, queue_size=10)

    def _init_subscriber(self):
        self.cam_sub = rospy.Subscriber('cv_camera/image_raw', Image, self.process_image)
        
    def process_image(self, image):
        self.image = image
        self.cam_pub.publish(self.image)
if __name__=='__main__':
    rospy.init_node('autocar_camera_node',anonymous=True)
    try:
        camera_node = CameraNode()
        rospy.loginfo("Camera node has started")
        rospy.spin()
    except Exception as e:
        print(e)
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard interrupteds')