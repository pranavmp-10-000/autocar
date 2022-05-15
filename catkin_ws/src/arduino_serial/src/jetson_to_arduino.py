import serial
from time import time
import rospy
from sensor_msgs.msg import String
class ArduinoSerial:
    def __init__(self) -> None:
        self.ser = serial.Serial('/dev/ttyUSB0')
        self.ser.baudrate = 19200
        self.ser.write('Serial Initialized')

    def _init_publisher(self):
        self.pub = rospy.Publisher('embedded/serial',)


    