import rospy
from geometry_msgs.msg import Twist

from phases.phase import Phase


class CrossingPhase(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.start_road = mission.start_road
        self.destination_road = mission.destination_road
        self.target_line = mission.stop_line

        if self.destination_road.name == 'N':
            self.condition_exp = lambda: self.mission.pos.y <= self.target_line
        elif self.destination_road.name == 'S':
            self.condition_exp = lambda: self.mission.pos.y >= self.target_line
        elif self.destination_road.name == 'E':
            self.condition_exp = lambda: self.mission.pos.x >= self.target_line
        elif self.destination_road.name == 'W':
            self.condition_exp = lambda: self.mission.pos.x <= self.target_line

    @property
    def name(self):
        return "Enter and cross intersection"

    def begin(self):
        pass

    def run(self):
        twist = Twist()
        twist.linear.x = 0.35
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.mission.cmd_vel_pub.publish(twist)

    def finish(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.mission.cmd_vel_pub.publish(twist)

    def condition(self):
        return self.condition_exp()
