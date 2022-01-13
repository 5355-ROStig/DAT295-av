import rospy

from geometry_msgs.msg import Twist

from phases.phase import Phase
from std_msgs.msg import Empty


class LeavePhase(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.destination_road = self.mission.destination_road
        self.target_line = self.mission.stop_line

        if self.destination_road.name == 'N':
            self.condition_exp = lambda: self.mission.pos.y <= self.target_line - 500
        elif self.destination_road.name == 'S':
            self.condition_exp = lambda: self.mission.pos.y >= self.target_line + 500
        elif self.destination_road.name == 'E':
            self.condition_exp = lambda: self.mission.pos.x >= self.target_line + 500
        elif self.destination_road.name == 'W':
            self.condition_exp = lambda: self.mission.pos.x <= self.target_line - 500

    @property
    def name(self):
        return "Leave intersection"

    def begin(self):
        self.mission.exit_pub.publish(Empty())
        rospy.loginfo("I have left the intersection")

    def run(self):
        twist = Twist()
        twist.linear.x = self.mission.drive_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.mission.cmd_vel_pub.publish(twist)

    def finish(self):
        rospy.loginfo("Done!")

    def condition(self):
        return self.condition_exp()
