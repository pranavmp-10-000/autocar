import logging
import cv2
import dl_detection as dl


def init_camera():
    vid = cv2.VideoCapture(0)
    return vid


def main():
    cap = init_camera()
    # logging.info(f'Camera Initialized: {cap.get(3)}')
    dl_engine = dl.DLObjectDetecction()
    while(True):
        ret, frame = cap.read()
        if ret:
            el_t = dl_engine.run_inference(frame)
            el_t = round(el_t,3)
            print(
                f'Human State: {str(dl_engine.human_state)} @@@@@@ Traffic State: {str(dl_engine.traffic_state)} in {el_t}s')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Keyboard Interrupt Captured')
