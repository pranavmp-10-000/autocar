import logging
import time
import cv2
import dl_detection as dl
import serial


def init_camera():
    vid = cv2.VideoCapture(0)
    return vid


def init_serial():
    ser = serial.Serial('/dev/ttyACM0', 9600 ,timeout=.1)
    #ser.baudrate = 115200
    ser.write('Serial Initialized'.encode('utf-8'))
    return ser


def main():
    cap = init_camera()
    ser = init_serial()
    # logging.info(f'Camera Initialized: {cap.get(3)}')
    dl_engine = dl.DLObjectDetecction()
    while(True):
        ret, frame = cap.read()
        if ret:
            # cv2.imshow(f'Frame', frame)
            el_t = dl_engine.run_inference(frame)
            el_t = round(el_t, 3)
            if dl_engine.human_state == 1:
                ser.write(bytes('0','utf-8'))
                ser.flush()
                time.sleep(0.05)
            else:
                # if dl_engine.traffic_state == 1:
                #     ser.write(bytes('2','utf-8'))
                #     ser.flush()
                #     time.sleep(0.05)
                # else:
                ser.write(bytes('2','utf-8'))
                ser.flush()
                time.sleep(0.05)
                # data = ser.readline()
                # print(data)
                # time.sleep(0.1)
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
