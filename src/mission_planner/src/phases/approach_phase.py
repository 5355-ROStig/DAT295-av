import rospy

from mission_planner_node import MissionPlannerNode
from phases.phase import Phase


class ApproachPhase(Phase):

    def __init__(self, mission: MissionPlannerNode):
        super().__init__(mission)
        self.start_road = self.mission.start_road
        self.target_line = self.mission.start_line

        if self.start_road.name == 'N':
            ...

        self.condition_exp = ...

    @property
    def name(self):
        return "Approach intersection"

    def begin(self):
        rospy.sleep(1)

    def run(self):
        pass

    def finish(self):
        pass

    def condition(self):
        return True
