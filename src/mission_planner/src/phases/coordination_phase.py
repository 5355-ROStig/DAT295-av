import rospy

from mapdata.msg import RoadSection

from coordination_strategies import COORDINATION_STRATEGIES
from phases.phase import Phase


class CoordinationPhase(Phase):

    def __init__(self, start_road: RoadSection):
        self.start_road = start_road
        self.strategy = COORDINATION_STRATEGIES[self.start_road.priority_sign]()

    @property
    def name(self):
        return f"Coordination (strategy: {self.strategy.__class__.__name__})"

    def begin(self):
        rospy.sleep(1)

    def run(self):
        pass

    def finish(self):
        pass

    def condition(self):
        return True
