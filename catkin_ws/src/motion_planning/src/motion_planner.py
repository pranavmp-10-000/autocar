#!/usr/bin/env python3

import rospy
import message_filters
from std_msgs.msg import Int32, Int16

class MotionPlanner:
    def __init__(self):
        self._init_subscriber()
        self._init_publisher()
        self.move = 0
        self.tsig = 0
        self.omsg = Int32()
    def _init_subscriber(self):
        self.traffic_signal_state_sub = rospy.Subscriber('signal_state',Int32, self.process_tsig)
        self.human_presence_state_sub = rospy.Subscriber('pedestrian_presence',Int32, self.process_signal)
    
    def _init_publisher(self):
        self.jetson_signal = rospy.Publisher('jetson_signal',Int16, queue_size=1)
    
    def process_tsig(self, sig):
        self.tsig = sig
    def process_signal(self, human_p):
        if human_p==1:
            self.omsg.data = 1
            self.jetson_signal.publish(0)
            print('Stop')
        else:
            if self.tsig==1:
                self.omsg.data = 0
                self.jetson_signal.publish(0)
                print('Stop')
            else:
                self.omsg.data = 2
                self.jetson_signal.publish(2)
                print('Go')


if __name__=='__main__':
    rospy.init_node('motion_planner_node',anonymous=True, log_level=rospy.ERROR)
    node_name = rospy.get_name()
    try:
        mp = MotionPlanner()
        #ts = message_filters.TimeSynchronizer([mp.human_presence_state_sub, mp.traffic_signal_state_sub], 1)
        #ts.registerCallback(mp.process_signal)
        rospy.spin()
    except KeyboardInterrupt:
        print('Traffic Sign Detection Node Closed')
    except Exception as e:
        print(e)
        