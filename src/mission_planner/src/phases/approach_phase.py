import rospy
from geometry_msgs.msg import Twist

from phases.phase import Phase


class ApproachPhase(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.start_road = self.mission.start_road
        self.target_line = self.mission.start_line

        if self.start_road.name == 'N':
            self.condition_exp = lambda: self.mission.pos.y >= self.target_line
        elif self.start_road.name == 'S':
            self.condition_exp = lambda: self.mission.pos.y <= self.target_line
        elif self.start_road.name == 'E':
            self.condition_exp = lambda: self.mission.pos.x <= self.target_line
        elif self.start_road.name == 'W':
            self.condition_exp = lambda: self.mission.pos.x >= self.target_line

    @property
    def name(self):
        return "Approach intersection"

    def begin(self):
        pass

    def run(self):
        print(self.mission.pos.y, self.target_line)
        twist = Twist()
        twist.linear.x = 0.35
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.mission.cmd_vel_pub.publish(twist)

    def finish(self):
        pass

    def condition(self):
        return self.condition_exp()
