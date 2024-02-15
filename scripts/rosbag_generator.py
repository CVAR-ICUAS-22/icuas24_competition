import rospy
from geometry_msgs.msg import PoseStamped

POINTS = [[1.0, 6.0, 3.9], [1.0, 13.5, 3.9], [1.0, 21.0, 3.9], [1.0, 26.0, 3.9], [7.0, 26.0, 3.9],
          [7.0, 21.0, 3.9], [7.0, 13.5, 3.9], [7.0, 6.0, 3.9], [7.0, 1.0, 3.9], [13.0, 1.0, 3.9],
          [13.0, 6.0, 3.9], [13.0, 13.5, 3.9], [13.0, 21.0, 3.9], [13.0, 26.0, 3.9], [19.0, 26.0, 3.9],
          [19.0, 21.0, 3.9], [19.0, 13.5, 3.9], [19.0, 6.0, 3.9]]

class NavigateWaypoints:
    def __init__(self) -> None:
        rospy.init_node("navigate_waypoints", anonymous=True)
        self.waypoints_pub = rospy.Publisher("/red/tracker/input_pose", PoseStamped, queue_size=10)
        self.waypoints_timer = rospy.Timer(rospy.Duration(8), self.publish_waypoints)
        self.counter = 0

    def publish_waypoints(self, event):
        pose = PoseStamped()
        pose.pose.position.x = POINTS[self.counter][0]
        pose.pose.position.y = POINTS[self.counter][1]
        pose.pose.position.z = POINTS[self.counter][2]
        if self.counter >= 14:
            pose.pose.orientation.w = 0.0
            pose.pose.orientation.z = 1.0
        self.waypoints_pub.publish(pose)
        self.counter += 1
        rospy.loginfo("Publishing waypoint %d", self.counter)
        if self.counter == len(POINTS):
            self.waypoints_timer.shutdown()

if __name__ == "__main__":
    navigator = NavigateWaypoints()
    rospy.spin()
