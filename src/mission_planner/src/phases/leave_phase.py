import rospy

from mapdata.msg import RoadSection, StopLine

from phases.phase import Phase


class LeavePhase(Phase):

    def __init__(self, destination_road: RoadSection, stop_line: StopLine):
        self.destination_road = destination_road
        self.target_line = stop_line

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
