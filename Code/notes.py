# bluetoth eslesen cihazlari gosterir
# import bluetooth

# nearby_devices = bluetooth.discover_devices(lookup_names=True)
# print("Found {} devices.".format(len(nearby_devices)))

# for addr, name in nearby_devices:
#     print("  {} - {}".format(addr, name))


# data=bluetooth.advertise_service()
# print(data)

"""---------------------------------------"""
# import serial

# serialPort = serial.Serial(port='COM4', baudrate=9600, timeout=0, parity=serial.PARITY_EVEN, stopbits=1)
# size = 1024

# while 1:
#     data = serialPort.readline(size)

#     if data:
#         print(data)

# serialPort = serial.Serial(port = "COM4", baudrate=115200,
#                            bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
# serialString = ""                           # Used to hold data coming over UART
# while(1):

#     # Wait until there is data waiting in the serial buffer
#     if(serialPort.in_waiting > 0):

#         # Read data out of the buffer until a carraige return / new line is found
#         serialString = serialPort.readline()

#         # Print the contents of the serial data
#         print(serialString.decode('Ascii'))

#         # Tell the device connected over the serial port that we recevied the data!
#         # The b at the beginning is used to indicate bytes!
#         serialPort.write(b"Thank you for sending data \r\n")
"""---------------------------------------"""


# MPU9250 Simple Visualization Code
# In order for this to run, the mpu9250_i2c file needs to 
# be in the local folder

import serial
# from mpu9250_i2c import *
import time,datetime
import numpy as np
import matplotlib.pyplot as plt

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
        data = ser.readline().decode('Ascii')
        if data != '':
            time.sleep(.1)
            data = data.split(',')
            ax,ay,az,wx,wy,wz = data[0],data[1],data[2],data[3],data[4],data[5]
            mx,my,mz = data[6],data[7],data[8]
        # ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
        # mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
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


