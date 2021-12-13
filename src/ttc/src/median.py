import math
import statistics

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
        # x = statistics.median(p[1][0] for p in self.buf)
        # y = statistics.median(y for _, (_, y) in self.buf)
        # time_delta = statistics.median(p[0] for p in self.buf)

        velocities = []

        for p1, p2 in zip(self.buf, list(self.buf)[1:]):
            displacement = p2[1] - p1[1]
            time_delta = p2[0]
            vel = displacement / time_delta
            # velocities.append(math.sqrt(vel.dot(vel)))
            velocities.append(vel)

        med_vel = np.median(velocities, axis=0)
        return *self.buf[-1][1], *med_vel

        # velSum = np.array([0, 0])
        # first = True
        # p = np.array([0, 0])
        #
        # for val in self.buf:
        #     if not first:
        #         dt = val[0]
        #         dp = val[1] - p
        #         velSum = velSum + (dp / dt)
        #     first = False
        #     p = val[1]
        #
        # self.v = velSum / self.buf.maxlen

    def newData(self, dt, pos):
        # State transition matrix
        self.buf.append((dt, pos))

        return self._updateVel()

