import rospy

from mapdata.msg import RoadSection, StopLine

from phases.phase import Phase


class CrossingPhase(Phase):

    def __init__(self, start_road: RoadSection, destination_road: RoadSection,
                 stop_line: StopLine):
        self.start_road = start_road
        self.destination_road = destination_road
        self.target_line = stop_line

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
