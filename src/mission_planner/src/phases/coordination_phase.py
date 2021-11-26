import rospy

from coordination_strategies import COORDINATION_STRATEGIES
from mission_planner_node import MissionPlannerNode
from phases.phase import Phase


class CoordinationPhase(Phase):

    def __init__(self, mission: MissionPlannerNode):
        super().__init__(mission)
        self.start_road = self.mission.start_road
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
