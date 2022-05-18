#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

class MotionPlanner:
    def __init__(self):
        self._init_subscriber()
    
    def _init_subscriber(self):
        self.traffic_signal_state_sub = rospy.Subscriber('signal_state',Int32, self.process_signal)
    
    def process_signal(self, data):
        print(data)


if __name__=='__main__':
    rospy.init_node('motion_planner_node',anonymous=True, log_level=rospy.ERROR)
    node_name = rospy.get_name()
    try:
        traffic_sign_detector = MotionPlanner()
        rospy.spin()
    except KeyboardInterrupt:
        print('Traffic Sign Detection Node Closed')
    except Exception as e:
        print(e)
        