import rospy

from geometry_msgs.msg import Twist

from phases.phase import Phase
from std_msgs.msg import Empty

class LeavePhase(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.destination_road = self.mission.destination_road
        self.target_line = self.mission.stop_line

    @property
    def name(self):
        return "Leave intersection"

    def begin(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.mission.cmd_vel_pub.publish(twist)
        self.mission.exit_pub.publish(Empty())
        rospy.loginfo("I have left!")

    def run(self):
        pass

    def finish(self):
        pass

    def condition(self):
        return True
