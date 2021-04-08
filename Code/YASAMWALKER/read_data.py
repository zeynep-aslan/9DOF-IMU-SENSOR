import serial
import time, datetime
import numpy as np
import matplotlib.pyplot as plt


ser = serial.Serial('COM6', 115200, timeout=0,
                            parity=serial.PARITY_EVEN, rtscts=1)
print(ser.is_open)
print(ser.name)
start = time.time()
while(1):
    # data = ser.readline().decode('Ascii')
    # data = data.split(',')
    data = ser.readline().decode('Ascii').replace('\r\n', '').split(',')
    if len(data)==11:
        print(data)
        # s1 = Sensor(data)
        # time.sleep(.1)
        # s1.displaySensorData()
        # time.sleep(.3)
        # s1.visualization()
        if time.time() - start > 30:
            ser.close()
            print(ser.is_open)
            break


