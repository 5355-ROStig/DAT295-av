import rospy

from mapdata.msg import RoadSection, StopLine

from phases.phase import Phase


class ApproachPhase(Phase):

    def __init__(self, start_road: RoadSection, start_line: StopLine):
        self.start_road = start_road
        self.target_line = start_line

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
