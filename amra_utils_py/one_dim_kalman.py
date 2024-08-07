#https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
from filterpy.kalman import KalmanFilter
import numpy as np

np.set_printoptions(precision=3)

refreshrate = 1/30

kf = KalmanFilter(dim_x = 2, dim_z = 1)
kf.x = np.array([1., 0])
kf.F = np.array([[1.0, 1.0], [0.0, 1.0]])
kf.H = np.array([[1.0, 0.0]])
kf.P *= 1000
kf.R = 5

from filterpy.common import Q_discrete_white_noise
kf.Q = Q_discrete_white_noise(dim=2, dt=refreshrate, var=0.13)

sensor_data = [1, 1, 1.5, 2, 3, 4, 5, 4.5, 5, 4, 3, 2, 1]
preds = [1]

for x in range(2, 13):
    z = sensor_data[x]
    pred = kf.predict()
    print(kf.x)
    preds.append(round(kf.x[0], 3))
    kf.update(z)

print(sensor_data)
print(preds)
    
