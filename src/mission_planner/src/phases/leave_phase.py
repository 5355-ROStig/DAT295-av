import rospy

from phases.phase import Phase


class LeavePhase(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.destination_road = self.mission.destination_road
        self.target_line = self.mission.stop_line

    @property
    def name(self):
        return "Leave intersection"

    def begin(self):
        rospy.sleep(1)

    def run(self):
        pass

    def finish(self):
        pass

    def condition(self):
        return True
