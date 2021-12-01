import rospy

from geometry_msgs.msg import Twist

from coordination_strategies import COORDINATION_STRATEGIES
from phases.phase import Phase


class CoordinationPhase(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.start_road = self.mission.start_road
        self.strategy = COORDINATION_STRATEGIES[self.start_road.priority_sign]()

    @property
    def name(self):
        return f"Coordination (strategy: {self.strategy.__class__.__name__})"

    def begin(self):
        self.run()

    def run(self):
        if not self.condition():
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.mission.cmd_vel_pub.publish(twist)
            rospy.loginfo("Waiting for go signal...")

    def finish(self):
        rospy.loginfo("I have permission to enter!")

    def condition(self):
        return self.mission.go
