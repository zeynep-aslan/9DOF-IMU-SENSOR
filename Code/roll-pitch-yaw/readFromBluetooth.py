import serial
import time, datetime
import numpy as np
import matplotlib.pyplot as plt
import threading

from math import sqrt, atan2, asin, degrees, radians
from deltat import DeltaT

class Sensor:
    
    declination = 0                         # Optional offset for true north. A +ve value adds to heading
    def __init__(self, data, timediff=None):
        self.data = data
        # self.accel_x, self.accel_y, self.accel_z, self.gyro_x, self.gyro_y, self.gyro_z, self.mag_x, self.mag_y, self.mag_z = "","","","","","","","","",

        self.magbias = (0, 0, 0)            # local magnetic bias factors: set from calibration
        self.deltat = DeltaT(timediff)      # Time between updates
        self.q = [1.0, 0.0, 0.0, 0.0]       # vector to hold quaternion
        GyroMeasError = radians(40)         # Original code indicates this leads to a 2 sec response time
        self.beta = sqrt(3.0 / 4.0) * GyroMeasError  # compute beta (see README)
        self.pitch = 0
        self.heading = 0
        self.roll = 0
    """
    def getSeperateData(self):
    
        self.accel_x = data[0]
        self.accel_y = data[1]
        self.accel_z = data[2]

        self.gyro_x = data[3]
        self.gyro_y = data[4]
        self.gyro_z = data[5]

        self.mag_x = data[6]
        self.mag_y = data[7]
        self.mag_z = data[8]

        self.magbias = (0, 0, 0)            # local magnetic bias factors: set from calibration
        self.deltat = DeltaT(timediff)      # Time between updates
        self.q = [1.0, 0.0, 0.0, 0.0]       # vector to hold quaternion
        GyroMeasError = radians(40)         # Original code indicates this leads to a 2 sec response time
        self.beta = sqrt(3.0 / 4.0) * GyroMeasError  # compute beta (see README)
        self.pitch = 0
        self.heading = 0
        self.roll = 0

    def displaySensorData(self):

        self.getSeperateData()
        time.sleep(.1)
        print(f"Accelerometer: {self.accel_x} - {self.accel_y} - {self.accel_z}")
        print(f"Gyroscope: {self.gyro_x} - {self.gyro_y} - {self.gyro_z}")
        print(f"Magnetometer: {self.mag_x} - {self.mag_y} - {self.mag_z}")
        print("**************")
    """

    # async def start(self, slow_platform=False):
    #     data = await self.read_coro()
    #     if len(data) == 2 or (self.expect_ts and len(data) == 3):
    #         asyncio.create_task(self._update_nomag(slow_platform))
    #     else:
    #         asyncio.create_task(self._update_mag(slow_platform))


    # stopfunc: A function returning True when calibration is deemed complete: this could be a timer or an input from the user.
    def calibrate(self, getxyz, stopfunc, wait=0):  #  getxyz: A function returning a 3-tuple of magnetic x,y,z values.
        magmax = list(getxyz())             # Initialise max and min lists with current values
        magmin = magmax[:]
        while not stopfunc():
            if wait != 0:
                if callable(wait):
                    wait()
                else:
                    time.sleep(wait/1000)  # Portable
            magxyz = tuple(getxyz())
            for x in range(3):
                magmax[x] = max(magmax[x], magxyz[x])
                magmin[x] = min(magmin[x], magxyz[x])
        self.magbias = tuple(map(lambda a, b: (a +b)/2, magmin, magmax))
    """
    def update_nomag(self, data, ts=None):    # 3-tuples (x, y, z) for accel, gyro
        # ax, ay, az = accel                  # Units G (but later normalised)
        # gx, gy, gz = (radians(x) for x in gyro) # Units deg/s
        
        ax, ay, az = data[0], data[1], data[2]
        gyro = data[3], data[4], data[5]
        gx, gy, gz = (radians(x) for x in gyro) # Units deg/s

        q1, q2, q3, q4 = (self.q[x] for x in range(4))   # short name local variable for readability
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _4q1 = 4 * q1
        _4q2 = 4 * q2
        _4q3 = 4 * q3
        _8q2 = 8 * q2
        _8q3 = 8 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return # handle NaN
        norm = 1 / norm        # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay
        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        deltat = self.deltat(ts)
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm
        self.heading = 0
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))
    """
    def update(self, data, ts=None):     # 3-tuples (x, y, z) for accel, gyro and mag data
        # mx, my, mz = (mag[x] - self.magbias[x] for x in range(3)) # Units irrelevant (normalised)
        # ax, ay, az = accel                  # Units irrelevant (normalised)
        # gx, gy, gz = (radians(x) for x in gyro)  # Units deg/s

        gyro = float(data[3]), float(data[4]), float(data[5])
        mag = float(data[6]), float(data[7]), float(data[8])
        ax, ay, az = float(data[0]), float(data[1]), float(data[2])
        gx, gy, gz = (radians(x) for x in gyro) # Units deg/s
        mx, my, mz = (mag[x] - self.magbias[x] for x in range(3))

        q1, q2, q3, q4 = (self.q[x] for x in range(4))   # short name local variable for readability
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _2q1q3 = 2 * q1 * q3
        _2q3q4 = 2 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return # handle NaN
        norm = 1 / norm                     # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0):
            return                          # handle NaN
        norm = 1 / norm                     # use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2 * q1 * mx
        _2q1my = 2 * q1 * my
        _2q1mz = 2 * q1 * mz
        _2q2mx = 2 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz

        # Gradient descent algorithm corrective step
        s1 = (-_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4)
             + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s2 = (_2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4)
             + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s3 = (-_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
             + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s4 = (_2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4)
              + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        deltat = self.deltat(ts)
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm
        self.heading = self.declination + degrees(atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
            self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3]))
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))

    def printRPY(self):
        print("roll: ", self.roll)
        print("pitch: ", self.pitch)
        print("yaw: ", self.heading)



if __name__ == "__main__":
    ser = serial.Serial('COM5', 115200, timeout=0,
                                parity=serial.PARITY_EVEN, rtscts=1)
    print(ser.is_open)
    print(ser.name)
    start = time.time()
    while(1):
        data = ser.readline().decode('Ascii')
        data = data.split(',')
        if len(data)==11:
            # print(data)
            s1 = Sensor(data)
            time.sleep(.1)
            # s1.displaySensorData()
            # time.sleep(.3)
            # s1.visualization()
            s1.update(data)
            s1.printRPY()
            if time.time() - start > 30:
                ser.close()
                print(ser.is_open)
                break
    print("bitti")




