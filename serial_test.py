from random import randint
import serial
import time

def init_serial():
    ser = serial.Serial('/dev/ttyACM0')
    ser.baudrate = 9600
    ser.write('Serial Initialized'.encode('utf-8'))
    return ser
if __name__ == '__main__':
    try:
        ser = init_serial()
        while(KeyboardInterrupt):
            i = randint(0,2)
            ser.write(bytes(f'{i}','utf-8'))
            time.sleep(0.1)
            data = ser.readline()
            print(data)
    except KeyboardInterrupt:
        print('Keyboard Interrupt Captured')
