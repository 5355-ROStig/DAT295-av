import rospy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

from phases.phase import Phase


class StopSignAtIntersection(Phase):
    stop_duration = rospy.Duration(nsecs=int(5e8))

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.start_road = self.mission.start_road
        self.has_stopped = False
        self.stop_timer = None

    @property
    def name(self):
        return f"Stop at stop sign for at least 0.5 seconds"

    def begin(self):
        if not self.condition():
            rospy.loginfo("Waiting for go signal...")

    def run(self):
        if not self.condition() and not self.has_stopped:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.mission.cmd_vel_pub.publish(twist)
            self.mission.stopped_pub.publish(Empty())
            self.has_stopped = True

    def finish(self):
        rospy.loginfo("I have permission to enter!")

    def condition(self):
        if self.stop_timer is None:
            self.stop_timer = rospy.Time.now()

        stopped_duration = rospy.Time.now() - self.stop_timer
        condition = self.mission.go and stopped_duration >= self.stop_duration

        if condition:
            self.stop_timer = None

        return condition
