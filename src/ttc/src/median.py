import numpy as np
from filter import Filter
import collections

class Median(Filter):
    def __init__(self, initPos, numSamples: int):
        # x is state (p_x, p_y, v_x, v_y)
        # z is measurement (p_x, p_y)
        self.buf = collections.deque(maxlen=numSamples) # historic positions
        self.buf.append((0, initPos))

    def _updateVel(self):
        velSum = np.array([0, 0])
        first = True
        p = np.array([0, 0])

        for val in self.buf:
            if not first:
                dt = val[0]
                dp = val[1] - p
                velSum = velSum + (dp / dt)
            first = False
            p = val[1]

        self.v = velSum / self.buf.maxlen

    def newData(self, dt, pos):
        # State transition matrix
        self.buf.append((dt, pos))
        self._updateVel()

        return np.array([pos[0], pos[1], self.v[0], self.v[1]])

