import logging
import time
import cv2
import dl_detection as dl
import serial


def init_camera():
    vid = cv2.VideoCapture(-1)
    return vid


def init_serial():
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
    ser.write('Serial Initialized'.encode('utf-8'))
    return ser


def main():
    cap = init_camera()
    ser = init_serial()
    print(f'Camera Initialized: {cap.get(3)}')
    start_time = time.time()
    dl_engine = dl.DLObjectDetecction()
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f'Model Loaded in {elapsed_time}')
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))

    size = (frame_width, frame_height)

    i = 0
    states = 0
    while(True):
        ret, frame = cap.read()

        if ret:
            el_t, pred_img = dl_engine.run_inference(frame)
            # cv2.imwrite(f'/home/pranav/autocar/scripts/imgs/{str(i)}.jpg',pred_img)
            # time.sleep(.2)
            el_t = round(el_t, 3)
            i = i+1
            states = states + dl_engine.human_state
            print(
                f'Human State: {str(dl_engine.human_state)} @@@@@@ Traffic State: {str(dl_engine.traffic_state)} in {el_t}s')
            # if i<10:
            #     cv2.imwrite(f'/home/pranav/autocar/scripts/imgs/{str(i)}.jpg',pred_img)
            # else:
            #     i=0
            if dl_engine.human_state == 1:
                ser.write(bytes('0', 'utf-8'))
                ser.flush()
                time.sleep(0.05)
                states = 0
            elif(states == 0):
                # if dl_engine.traffic_state == 1:
                #     ser.write(bytes('2','utf-8'))
                #     ser.flush()
                #     time.sleep(0.05)
                # else:
                ser.write(bytes('1', 'utf-8'))
                ser.flush()
                time.sleep(0.05)
                # data = ser.readline()
                # print(data)
                # time.sleep(0.1)
            if states == 2:
                states = 0
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if KeyboardInterrupt:
            break
    cap.release()
    ser.close()
    # result.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Keyboard Interrupt Captured')
