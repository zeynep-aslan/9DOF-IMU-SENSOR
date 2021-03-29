import serial
import time, datetime
import numpy as np
import matplotlib.pyplot as plt
import threading

class Sensor:

    def __init__(self, data):
        self.data = data
        self.accel_x, self.accel_y, self.accel_z, self.gyro_x, self.gyro_y, self.gyro_z, self.mag_x, self.mag_y, self.mag_z = "","","","","","","","","",
        # t_visual = threading.Thread(target=self.visualization)
        # t_visual.start()

    def getSeperateData(self):
    
        # data = self.data.split(',')
        self.accel_x = data[0]
        self.accel_y = data[1]
        self.accel_z = data[2]

        self.gyro_x = data[3]
        self.gyro_y = data[4]
        self.gyro_z = data[5]

        self.mag_x = data[6]
        self.mag_y = data[7]
        self.mag_z = data[8]

    def displaySensorData(self):

        self.getSeperateData()
        time.sleep(.1)
        print(f"Accelerometer: {self.accel_x} - {self.accel_y} - {self.accel_z}")
        print(f"Gyroscope: {self.gyro_x} - {self.gyro_y} - {self.gyro_z}")
        print(f"Magnetometer: {self.mag_x} - {self.mag_y} - {self.mag_z}")
        print("**************")
        # t_visual = threading.Thread(target=self.visualization)
        # t_visual.start()

    def visualization(self):

        # while(True):
        plt.style.use('ggplot') # matplotlib visual style setting
        time.sleep(1) # wait for mpu9250 sensor to settle

        ii = 1000 # number of points
        t1 = time.time() # for calculating sample rate
        # prepping for visualization
        mpu6050_str = ['accel-x','accel-y','accel-z','gyro-x','gyro-y','gyro-z']
        AK8963_str = ['mag-x','mag-y','mag-z']
        mpu6050_vec,AK8963_vec,t_vec = [],[],[]

        print('recording data')
        for ii in range(0,ii):
            
            try:
                ax,ay,az,wx,wy,wz = self.accel_x, self.accel_y, self.accel_z, self.gyro_x, self.gyro_y, self.gyro_z
                mx,my,mz = self.mag_x, self.mag_y, self.mag_z
            except:
                continue
            t_vec.append(time.time()) # capture timestamp
            AK8963_vec.append([mx,my,mz])
            mpu6050_vec.append([ax,ay,az,wx,wy,wz])

        print('sample rate accel: {} Hz'.format(ii/(time.time()-t1))) # print the sample rate
        t_vec = np.subtract(t_vec,t_vec[0])

        # plot the resulting data in 3-subplots, with each data axis
        fig,axs = plt.subplots(3,1,figsize=(12,7),sharex=True)
        cmap = plt.cm.Set1

        ax = axs[0] # plot accelerometer data
        for zz in range(0,np.shape(mpu6050_vec)[1]-3):
            data_vec = [ii[zz] for ii in mpu6050_vec]
            ax.plot(t_vec,data_vec,label=mpu6050_str[zz],color=cmap(zz))
        ax.legend(bbox_to_anchor=(1.12,0.9))
        ax.set_ylabel('Acceleration [g]',fontsize=12)

        ax2 = axs[1] # plot gyroscope data
        for zz in range(3,np.shape(mpu6050_vec)[1]):
            data_vec = [ii[zz] for ii in mpu6050_vec]
            ax2.plot(t_vec,data_vec,label=mpu6050_str[zz],color=cmap(zz))
        ax2.legend(bbox_to_anchor=(1.12,0.9))
        ax2.set_ylabel('Angular Vel. [dps]',fontsize=12)

        ax3 = axs[2] # plot magnetometer data
        for zz in range(0,np.shape(AK8963_vec)[1]):
            data_vec = [ii[zz] for ii in AK8963_vec]
            ax3.plot(t_vec,data_vec,label=AK8963_str[zz],color=cmap(zz+6))
        ax3.legend(bbox_to_anchor=(1.12,0.9))
        ax3.set_ylabel('Magn. Field [Î¼T]',fontsize=12)
        ax3.set_xlabel('Time [s]',fontsize=14)

        fig.align_ylabels(axs)
        plt.show()
        # if 0xFF == ord('q'):
        #     break


# if '__main__':
ser = serial.Serial('COM5', 115200, timeout=0,
                            parity=serial.PARITY_EVEN, rtscts=1)
print(ser.is_open)
print(ser.name)
# s = ser.read(100)
start = time.time()
while(1):
    data = ser.readline().decode('Ascii')
    data = data.split(',')
    # print(len(data))
    # print(data)

    time.sleep(1)

    # print(len(data))
    # print("data-------", data)
    if len(data)==11:
        # print(data)
        # time.sleep(.1)
        # data = data.split(',')
        # print(data)
        # print(data[0], "--", data[1], "--", data[2], "--")
        s1 = Sensor(data)
        time.sleep(.1)
        # s1.getSeperateData()
        s1.displaySensorData()
        time.sleep(.1)
        # t_visual = threading.Thread(target=s1.visualization)
        # t_visual.start()

        s1.visualization()
        # print(s1.displaySensorData)
        # print(start)
        if time.time() - start > 30:
            ser.close()
            print(ser.is_open)
            break
# print(s)
# ser.close()
# print(ser.is_open)
print("bitti")







