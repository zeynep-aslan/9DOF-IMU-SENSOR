# terminale pytest yaziyoruz test etmek icin
import numpy as np
import matplotlib.pyplot as plt
from kf import KF
from readFromBluetooth import Sensor

plt.ion()
plt.figure()

real_x = 0.0
meas_variance = 0.1 ** 2  # this is gonna simulate the noise in our measurements
real_v = 0.5

# kf = KF(initial_x=0.0, initial_v=1.0, accel_variance=0.1)  # accel_variance: ivme
kf = KF(initial_x=0.0, initial_v=1.0, accel_variance=0.1)  # accel_variance: ivme

DT = 0.1
NUM_STEPS = 1000
MEAS_EVERY_STEPS = 20

mus = []
covs = []

for step in range(NUM_STEPS):
    covs.append(kf.cov)
    mus.append(kf.mean)

    real_x = real_x + DT * real_v

    kf.predict(dt=DT)
    if step != 0 and step % MEAS_EVERY_STEPS == 0:
        kf.update(meas_value=real_x + np.random.randn() * np.sqrt(meas_variance), 
                 meas_variance=meas_variance)

# for i in covs:
#     if i.any() < 0:
#         print(i)

plt.subplot(2, 1, 1)    
plt.title('Position')
plt.plot([mu[0] for mu in mus], 'r')  # grafik 0 dan basliyor
plt.plot([mu[0] - 2*np.sqrt(cov[0,0]) for mu, cov in zip(mus, covs)], 'r--')  # tirtikli olan asagi dogru, sol taraf en k.deger -100
plt.plot([mu[0] + 2*np.sqrt(cov[0,0]) for mu, cov in zip(mus, covs)], 'r--')  # tirtikli olan yukari dogru


plt.subplot(2, 1, 2)    
plt.title('Velocity')
plt.plot([mu[1] for mu in mus], 'r')
plt.plot([mu[1] - 2*np.sqrt(cov[1,1]) for mu, cov in zip(mus, covs)], 'r--')
plt.plot([mu[1] + 2*np.sqrt(cov[1,1]) for mu, cov in zip(mus, covs)], 'r--')


plt.show()
plt.ginput(1)