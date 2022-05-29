import cv2
import numpy as np
import logging
from threading import Thread
from onnx_inference import ObjectDetection
from cv_libs import check_signal_state
import time

TRAFFIC_DET_MODEL_PATH = 'scripts/models/trafficsign.onnx'
HUMAN_DET_MODEL_PATH = 'scripts/models/human_recog.onnx'
INPUT_W = 512
INPUT_H = 512


class DLObjectDetecction():
    def __init__(self) -> None:
        # self.traffic_engine = ObjectDetection(TRAFFIC_DET_MODEL_PATH, True)
        # logging.info('Traffic TensorRT engine intialized')
        self.human_engine = ObjectDetection(HUMAN_DET_MODEL_PATH, True)
        logging.info('Human TensorRT engine intialized')
        self.human_state = 0
        self.traffic_state = 0
        self.preproc_img_data = None
        self.drawn_img = None

    def image_resize(self, image):
        h, w, _ = image.shape
        r_w = INPUT_W / w
        r_h = INPUT_H / h
        if r_h > r_w:
            tw = INPUT_W
            th = int(r_w * h)
            tx1 = tx2 = 0
            ty1 = int((INPUT_H - th) / 2)
            ty2 = INPUT_H - th - ty1
        else:
            tw = int(r_h * w)
            th = INPUT_H
            tx1 = int((INPUT_W - tw) / 2)
            tx2 = INPUT_W - tw - tx1
            ty1 = ty2 = 0

        # Resize the image with long side while maintaining ratio
        # Pad the short side with (128,0128,128)
        image = cv2.resize(image, (tw, th), cv2.INTER_NEAREST)
        image = cv2.copyMakeBorder(
            image, ty1, ty2, tx1, tx2, cv2.BORDER_CONSTANT, (128, 128, 128)
        )
        return image

    def _preprocess_image(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.image = self.image_resize(image)  # cv2.resize(image, (512, 512))
        pred_inp_img = np.float16(self.image/255.0)[np.newaxis, :, :, :]
        self.preproc_img_data = np.ascontiguousarray(
            np.rollaxis(pred_inp_img, 3, 1))

    def traffic_run_inference(self):
        boxes, scores, classes = self.traffic_engine.infer(
            self.preproc_img_data, conf_thres=0.7, class_det=8)
        signal_state = check_signal_state(self.image, boxes)
        #self.draw_boxes(self.image, boxes, 'traffic')
        if signal_state == None:
            self.traffic_state = 1
        else:
            self.traffic_state = signal_state

    def human_run_inference(self):
        boxes, scores, classes = self.human_engine.infer(
            self.preproc_img_data, conf_thres=0.3, class_det=0)
        # ser.baudrate = 115200self.drawn_img = self.draw_boxes(self.image, boxes, 'human')
        if len(boxes) > 0:
            self.human_state = 1
        else:
            self.human_state = 0

    def run_inference(self, img_data):
        start_time = time.time()
        self._preprocess_image(img_data)
        self.human_run_inference()
        # self.traffic_run_inference()
        end_time = time.time()
        elapsed_time = end_time - start_time
        return elapsed_time, self.drawn_img

    def draw_boxes(self, img, bboxes, type=''):
        #img = cv2.resize(img, (512, 512))
        bboxes = bboxes.astype(int)
        img = img.astype(np.uint8)
        for i in bboxes:
            x1, y1, x2, y2 = i
            img = cv2.rectangle(self.image, (x1, y1),
                                (x2, y2), (0, 255, 255), 1)
        #cv2.imshow(f'{type} Detection_boxes', img)
        # cv2.waitKey(30)
        return img
