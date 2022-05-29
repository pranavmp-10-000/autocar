from random import randint
import serial
import time

def init_serial():
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=0.1)
    #ser.baudrate = 9600
    ser.write('Serial Initialized'.encode('utf-8'))
    return ser
if __name__ == '__main__':
    try:
        ser = init_serial()
        while(KeyboardInterrupt):
            i = randint(0,1)
            ser.write(bytes(f'{i}','utf-8'))
            ser.flush()
            time.sleep(0.05)
            data = ser.readline()
            print(data)
    except KeyboardInterrupt:
        ser.write(bytes(f'0','utf-8'))
        ser.flush()
        ser.close()
        print('Keyboard Interrupt Captured')
