import math
import statistics

import numpy as np
from filter import Filter
import collections

class SplitMedian(Filter):
    def __init__(self, initPos, numSamples: int):
        # x is state (p_x, p_y, v_x, v_y)
        # z is measurement (p_x, p_y)
        if(not numSamples % 4 == 0 or numSamples == 0):
            raise Exception("Number of samples must be divisible by 4")
        
        self.buf = collections.deque(maxlen=numSamples) # historic positions
        self.buf.append((0, initPos))

    def _getMedianPos(self, startIdx: int, endIdx: int):
        return np.mean(list(map(lambda x: x[1], list(self.buf)[startIdx:endIdx])), axis=0)

    def _updateVel(self):
        p1 = self._getMedianPos(0, self.buf.maxlen//2 - 1)
        p2 = self._getMedianPos(self.buf.maxlen//2, self.buf.maxlen-1)

        dt = sum(list(map(lambda x: x[0], list(self.buf)[self.buf.maxlen//4:(self.buf.maxlen*3)//4])))

        vel = (p2 - p1) / dt
        return *self.buf[-1][1], *vel

    def newData(self, dt, pos):
        # State transition matrix
        self.buf.append((dt, pos))

        return self._updateVel()

