import rospy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

from phases.phase import Phase


class StopAtIntersection(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.start_road = self.mission.start_road

    @property
    def name(self):
        return f"Stop at Intersection if necessary"

    def begin(self):
        if not self.condition():
            rospy.loginfo("Waiting for go signal...")

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
            self.mission.stopped_pub.publish(Empty())

    def finish(self):
        rospy.loginfo("I have permission to enter!")

    def condition(self):
        return self.mission.go
