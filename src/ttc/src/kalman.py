from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

class KalmanFilterHelper:
    def __init__(self, initPos):
        # x is state (x, y, v_x, v_y)
        # z is measurement (x, y)
        self.filter = KalmanFilter(dim_x=4, dim_z=2)

        # Initial state
        self.filter.x = np.array([initPos[0], initPos[1], 0., 0.]) # Should maybe set the position correctly?

        # Measurement transition matrix (maps z to x)
        self.filter.H = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0]])

        # Initial state covariance matrix
        self.filter.P = np.eye(4) * 1000

        # Measurement noise
        self.filter.R = np.eye(2) * 0.05

        # Process noise
        self.filter.Q = Q_discrete_white_noise(dim=4, dt=0.05, var=0.13)


    def newData(self, dt, pos):
        # State transition matrix
        self.filter.F = np.array([[1., 0., dt, 0.],
                                  [0., 1., 0., dt],
                                  [0., 0., 1., 0.],
                                  [0., 0., 0., 1.]])

        self.filter.predict()
        self.filter.update(pos)

        return self.filter.x
