import rospy

from phases.phase import Phase


class CrossingPhase(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.start_road = mission.start_road
        self.destination_road = mission.destination_road
        self.target_line = mission.stop_line

    @property
    def name(self):
        return "Enter and cross intersection"

    def begin(self):
        rospy.sleep(1)

    def run(self):
        pass

    def finish(self):
        pass

    def condition(self):
        return True
