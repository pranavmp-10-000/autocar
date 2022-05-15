import cv2
import numpy as np

def get_bbox_area(bboxes):
    areas = []
    for i in bboxes:
        width = i[2]-i[0]
        height = i[3] - i[1]
        area = width*height
        areas.append(area)
    max_area_idx, max_area = np.argmax(areas), np.max(areas)
    return max_area_idx, max_area

def check_green(image):
    ret, thres = cv2.threshold(image[:,:,1],200,255,0)
    edged = cv2.Canny(thres, 30, 225,4)
    contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours)>0:
        contours_s = np.vstack(contours)
        x2,y2, x1, y1 = np.max(contours_s[:,0,0]),np.max(contours_s[:,0,1]), np.min(contours_s[:,0,0]),np.min(contours_s[:,0,1])
        return True
    else:
        x2,y2, x1, y1 = 0,0,0,0
        return False

def check_red(image):
    ret, thres = cv2.threshold(image[:,:,0],200,255,0)
    edged = cv2.Canny(thres, 30, 225,4)
    contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours)>0:
        contours_s = np.vstack(contours)
        x2,y2, x1, y1 = np.max(contours_s[:,0,0]),np.max(contours_s[:,0,1]), np.min(contours_s[:,0,0]),np.min(contours_s[:,0,1])
        return True
    else:
        x2,y2, x1, y1 = 0,0,0,0
        return False

def check_signal_state(image, bboxes):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    max_idx, max_area = get_bbox_area(bboxes)
    x1,y1,x2,y2 = bboxes[max_idx].astype(int)
    cropped_image = image[y1:y2,x1:x2,:]
    is_green = check_green(cropped_image)
    is_red = check_red(cropped_image)
    if is_green:
        return 1
    elif is_red:
        return 0


