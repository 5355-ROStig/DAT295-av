from abc import ABC, abstractmethod


class Filter(ABC):
    @abstractmethod
    def __init__(self, initPos):
        ...

    # return type is numpy array (p_x, p_y, v_x, v_y)
    @abstractmethod
    def newData(self, dt, pos):
        ...
