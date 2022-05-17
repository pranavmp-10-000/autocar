import onnxruntime as ort
import cv2
import numpy as np
from time import time

OUTPUT_SIZE = (1,16128,-1)
INPUT_W = 512
INPUT_H = 512

class ObjectDetection:
    def __init__(self, model_path, trt = False) -> None:
        if trt:
            self.sess = ort.InferenceSession(model_path, providers=['TensorrtExecutionProvider', 'CUDAExecutionProvider'])
        else:
            self.sess = ort.InferenceSession(model_path, providers=[('CUDAExecutionProvider'),'CPUExecutionProvider'])
            
    def _preprocess_image(self, image):
        image = image/255.0#cv2.resize(image,(512,512))/255.0
        pred_inp_img = np.float16(image)[np.newaxis, :, :, :]
        pred_inp_img = np.ascontiguousarray(np.rollaxis(pred_inp_img, 3, 1))
        return pred_inp_img
    
    def infer(self, image, conf_thres = 0.5, class_det=0):
        processed_image = self._preprocess_image(image)
        pred = self.sess.run(None, {self.sess.get_inputs()[0].name: processed_image})
        boxes, scores, classes = self.process_outputs(pred, conf_thres, class_det)
        return boxes, scores, classes

    def _extract_bbs(self,outputs, conf_thres):
        boxes = outputs[:,:4]
        scores = np.round(outputs[:, 4],4)
        classes = outputs[:,5:]
        return boxes, scores, classes


    def _non_max_suppression(self, boxes, confs, classes, iou_thres=0.3):
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = confs.flatten().argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= iou_thres)[0]
            order = order[inds + 1]
        boxes = boxes[keep]
        confs = confs[keep]
        classes = classes[keep]
        return boxes, confs, classes

    def xywh2xyxy(self, x, origin_w=0, origin_h=0):
        """
        description:    Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2]
                        where xy1=top-left, xy2=bottom-right
        param:
            origin_h:   height of original image
            origin_w:   width of original image
            x:          A boxes tensor, each row is a box [center_x, center_y, w, h]
        return:
            y:          A boxes tensor, each row is a box [x1, y1, x2, y2]
        """
        y = np.zeros_like(x)
        r_w = INPUT_W / origin_w
        r_h = INPUT_H / origin_h
        if r_h > r_w:
            y[:, 0] = x[:, 0] - x[:, 2] / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2 - (INPUT_H - r_w * origin_h) / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2 - (INPUT_H - r_w * origin_h) / 2
            y /= r_w
        else:
            y[:, 0] = x[:, 0] - x[:, 2] / 2 - (INPUT_W - r_h * origin_w) / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2 - (INPUT_W - r_h * origin_w) / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2
            y /= r_h

        # y[:,0] = x[:, 0] - x[:, 2] / 2
        # y[:,2] = x[:, 2] + x[:, 2] / 2
        # y[:, 1] = x[:, 1] - x[:, 3] / 2
        # y[:, 3] = x[:, 1] + x[:, 3] / 2
        return y     
    def nms(self, pred, iou_thres = 0.3, origin_w=0, orign_h=0, class_det=0):
        boxes = self.xywh2xyxy(pred[...,0 :4], INPUT_W, INPUT_H)
        confs = pred[:, 4]
        class_to_detect = class_det+5
        classes = np.argmax(pred[:, class_to_detect:class_to_detect+1], axis=-1)
        return self._non_max_suppression(boxes, confs, classes, iou_thres)

    def process_outputs(self, outputs, conf_thres, class_det):
        outputs = np.array(outputs[-1]).reshape(OUTPUT_SIZE)
        outputs = outputs[outputs[...,4]>conf_thres]
        boxes, scores, classes = self.nms(outputs, class_det)
        # print('Classes:',classes.shape)
        # print('Boxes:',boxes)
        # print('Scores:',scores.shape)
        return boxes, scores, classes
    
            