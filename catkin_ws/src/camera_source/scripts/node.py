from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String
class CameraNode:
    def __init__(self) -> None:
        rospy.init_node('camera_node')
        self.bridge = CvBridge()

    def _init_publisher(self):
        self.cam_pub = rospy.Publisher('camera_topic/image', Image)

    def _init_subscriber(self):
        self.cam_sub = rospy.Subscriber('image_raw', Image, self.process_image)
        
    def process_image(self, image):
        self.image = image

if __name__=='__main__':
    rospy.init_node('autocar_camera_node')
    rospy.loginfo("Camera node has started")