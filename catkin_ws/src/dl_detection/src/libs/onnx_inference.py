import onnxruntime as ort
import cv2
import numpy as np
from time import time

OUTPUT_SIZE = (1,16128,6)

class TrafficSignDetection:
    def __init__(self, model_path, trt = False) -> None:
        if trt:
            self.sess = ort.InferenceSession(model_path, providers=['TensorrtExecutionProvider', 'CUDAExecutionProvider'])
        else:
            self.sess = ort.InferenceSession(model_path, providers=[('CUDAExecutionProvider'),'CPUExecutionProvider'])
            
    def _preprocess_image(self, image):
        image = cv2.resize(image,(512,512))
        pred_inp_img = np.float16(image)[np.newaxis, :, :, :]
        pred_inp_img = np.ascontiguousarray(np.rollaxis(pred_inp_img, 3, 1))
        return pred_inp_img
    
    def infer(self, image, conf_thres = 0.5):
        processed_image = self._preprocess_image(image)
        pred = self.sess.run(None, {self.sess.get_inputs()[0].name: processed_image})
        self.process_outputs(pred, conf_thres)

    def _extract_bbs(self,outputs, conf_thres):
        outputs = outputs[outputs[:,4]>conf_thres]
        boxes = outputs[:,:4]
        scores = np.round(outputs[:, 4],4)
        classes = outputs[:,5:].astype(int)
        return boxes, scores, classes


    def process_outputs(self, outputs, conf_thres):
        outputs = np.array(outputs).reshape(OUTPUT_SIZE)
        boxes, scores, classes = self._extract_bbs(outputs[0], conf_thres)
        print('Classes:',classes)
        print('Boxes:',boxes)
        print('Scores:',scores)
            