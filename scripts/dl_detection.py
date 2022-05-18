import cv2
import numpy as np
import logging
from threading import Thread
from onnx_inference import ObjectDetection
from cv_libs import check_signal_state
import time

TRAFFIC_DET_MODEL_PATH = 'scripts/models/traffic_sign.onnx'
HUMAN_DET_MODEL_PATH = 'scripts/models/human_recog.onnx'


class DLObjectDetecction():
    def __init__(self) -> None:
        self.traffic_engine = ObjectDetection(TRAFFIC_DET_MODEL_PATH, True)
        logging.info('Traffic TensorRT engine intialized')
        self.human_engine = ObjectDetection(HUMAN_DET_MODEL_PATH, True)
        logging.info('Human TensorRT engine intialized')
        self.human_state = 0
        self.traffic_state = 0
        self.preproc_img_data = None

    def _preprocess_image(self, image):
        self.image = cv2.resize(image, (512, 512))
        pred_inp_img = np.float16(self.image/255.0)[np.newaxis, :, :, :]
        self.preproc_img_data = np.ascontiguousarray(
            np.rollaxis(pred_inp_img, 3, 1))

    def traffic_run_inference(self):
        boxes, scores, classes = self.traffic_engine.infer(
            self.preproc_img_data, conf_thres=0.2, class_det=-1)
        signal_state = check_signal_state(self.image, boxes)
        # self.draw_boxes(self.image, boxes)
        self.traffic_state = signal_state

    def human_run_inference(self):
        boxes, scores, classes = self.human_engine.infer(
            self.preproc_img_data, conf_thres=0.2, class_det=0)
        #self.draw_boxes(self.image, boxes)
        if len(boxes) > 0:
            self.human_state = 1
        else:
            self.human_state = 0

    def run_inference(self, img_data):
        start_time = time.time()
        self._preprocess_image(img_data)
        self.human_run_inference()
        self.traffic_run_inference()
        end_time = time.time()
        elapsed_time = end_time - start_time
        return elapsed_time

    def draw_boxes(self, img, bboxes):
        #img = cv2.resize(img, (512, 512))
        bboxes = bboxes.astype(int)
        img = img.astype(np.uint8)
        for i in bboxes:
            x1, y1, x2, y2 = i
            img = cv2.rectangle(self.image, (x1, y1),
                                (x2, y2), (0, 255, 255), 1)
        cv2.imshow('Detection_boxes', img)
        cv2.waitKey(30)
