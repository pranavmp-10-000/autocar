from tracemalloc import start
import cv2
from onnx_inference import TrafficSignDetection
import numpy as np
import time

start_time = time.time()
engine = TrafficSignDetection('/home/pranav/Desktop/Projects/autocar/traffic_sign.onnx',True)
end_time = time.time()
print('Engine loaded in :',str(end_time-start_time))


image = cv2.imread('/home/pranav/Pictures/traffic-signal.jpeg')

start_time = time.time()
engine.infer(image)
end_time = time.time()
print('Inference in:',str(end_time-start_time))