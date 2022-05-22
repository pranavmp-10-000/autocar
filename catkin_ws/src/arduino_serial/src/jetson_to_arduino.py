#!/usr/bin/env python3

from numpy import unicode_
import serial
from time import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16, Float32
import io
class ArduinoSerial:
    def __init__(self) -> None:
        self.ser = serial.Serial('/dev/ttyACM0')
        self.ser.baudrate = 9600
        self.ser.write('Serial Initialized'.encode('utf-8'))
        self._init_subscriber()

    def _init_subscriber(self):
        self.state_subscriber = rospy.Subscriber('motion_planner',Int16, self.send_commands)

    def send_commands(self, data):
        print(data)
        if data==0:
            self.ser.write(str('0').encode('utf-8'))
            self.ser.flush()
        elif data==1:
            self.ser.write(str('1').encode('utf-8'))
            self.ser.flush()
        elif data==2:
            self.ser.write(str('2').encode('utf-8'))
            self.ser.flush()
        else:
            self.ser.write(str('0').encode('utf-8'))
            self.ser.flush()

    
    def close(self):
        self.ser.close()

if __name__=='__main__':
    rospy.init_node('serial_node', anonymous=True, log_level= rospy.ERROR)
    node_name = rospy.get_name()
    try:
        serial_node = ArduinoSerial()
        rospy.spin()
    except KeyboardInterrupt:
        print('Arduino Serial Closed')
        serial_node.close()
    except Exception as e:
        print(e)


    