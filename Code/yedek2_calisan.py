import serial
import time

class Sensor:

    def __init__(self, data):
        self.data = data

    def getSeperateData(self):
        data = self.data.split(',')
        # print(data)
        accel_x = data[0]
        accel_y = data[1]
        accel_z = data[2]

        gyro_x = data[3]
        gyro_y = data[4]
        gyro_z = data[5]

        mag_x = data[6]
        mag_x = data[7]
        mag_x = data[8]

        self.displaySensorData(accel_y)

    def displaySensorData(self, accel_y):
        print(accel_y)

    def visualization3D(self):  # farkli yerleri degistir
        # def plot_opaque_cube(x=10, y=20, z=30, dx=40, dy=50, dz=60):
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1, projection='3d')


        xx = np.linspace(x, x+dx, 2)
        yy = np.linspace(y, y+dy, 2)
        zz = np.linspace(z, z+dz, 2)

        xx, yy = np.meshgrid(xx, yy)

        ax.plot_surface(xx, yy, z)
        ax.plot_surface(xx, yy, z+dz)

        yy, zz = np.meshgrid(yy, zz)
        ax.plot_surface(x, yy, zz)
        ax.plot_surface(x+dx, yy, zz)

        xx, zz = np.meshgrid(xx, zz)
        ax.plot_surface(xx, y, zz)
        ax.plot_surface(xx, y+dy, zz)
        # ax.set_xlim3d(-dx, dx*2, 20)
        # ax.set_xlim3d(-dx, dx*2, 20)
        # ax.set_xlim3d(-dx, dx*2, 20)
        plt.title("Cube")
        plt.show()


# if '__main__':
ser = serial.Serial('COM5', 38400, timeout=0,
                            parity=serial.PARITY_EVEN, rtscts=1)
print(ser.is_open)
print(ser.name)
# s = ser.read(100)
start = time.ctime()
while(1):
    data = ser.readline().decode('Ascii')
    # print("data-------", data)
    if data != '':
        # print(data)
        # time.sleep(.1)
        # data = data.split(',')
        # print(data)
        # print(data[0], "--", data[1], "--", data[2], "--")
        s1 = Sensor(data)
        time.sleep(.1)
        s1.getSeperateData()
        # print(s1.displaySensorData)
    if start==10:
        ser.close()
        print(ser.is_open)
        break
# print(s)
ser.close()
print(ser.is_open)








