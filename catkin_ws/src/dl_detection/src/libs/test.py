from tracemalloc import start
import cv2
from onnx_inference import ObjectDetection
import numpy as np
import time

start_time = time.time()
engine = ObjectDetection('~/Desktop/Projects/autocar/catkin_ws/res/models/yolov5s.onnx',False)
end_time = time.time()
print('Engine loaded in :',str(end_time-start_time))


image = cv2.imread('~/Pictures/pedestrians.jpg')
image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
start_time = time.time()
boxes, scores, classes = engine.infer(image, 0.5)
end_time = time.time()
bboxes = boxes.astype(int)
img = cv2.resize(image,(512,512))
for i in bboxes:
    x1, y1, x2, y2 = i
    img = cv2.rectangle(img,(x1,y1), (x2,y2), (0,255,255),1)
cv2.imwrite('test.jpg',img)
print('Inference in:',str(end_time-start_time))