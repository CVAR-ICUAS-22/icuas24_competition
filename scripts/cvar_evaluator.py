#!/usr/bin/env python3
"""
cvar_evaluator.py: Evaluator Node for the ICUAS 24 Competition.
"""
import rospy
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

NAMESPACE = 'red'
DISTANCE_THRESHOLD = 0.2


class Evaluator:
    """Evaluator"""

    def __init__(self):
        rospy.init_node('cvar_evaluator', anonymous=True)
        self.namespace = rospy.get_param('~namespace', '')

        # Suscribers
        rospy.Subscriber('challenge_started', Bool, self.cb_challenge_started)
        rospy.Subscriber('plants_beds', String, self.cb_plant_beds)
        rospy.Subscriber('/fruit_count',
                         Int32, self.cb_fruit_count)
        rospy.Subscriber('odometry', Odometry, self.cb_odom)

        # Initialize internal variables
        self.challenge_start_time = self.get_current_time()

        self.plant_beds_flag = False
        self.challenge_start_flag = False
        self.fruit_count_received = False

        self.initial_position = Point()
        self.last_position = Point()
        self.distance_travelled = 0

        rospy.loginfo('Node started')
        rospy.spin()

    def get_current_time(self):
        """get_current_time: Get the current time."""
        return rospy.Time.now()

    def print_time(self, time_name, input_time):
        """print_time: Print the time."""
        total_seconds = input_time.secs - self.challenge_start_time.secs  # in seconds
        # rospy.loginfo('[Evaluator] %s: %d s', time_name, total_seconds)
        minutes = total_seconds // 60
        seconds = total_seconds % 60
        rospy.loginfo('[Evaluator] %s: %f s -- %d m, %d s',
                      time_name, total_seconds, minutes, seconds)

    def calculate_distance(self, point1, point2):
        """calculate_distance: Calculate the distance between two points."""
        # 3D distance
        return ((point2.x - point1.x)**2 + (point2.y - point1.y)**2 + (point2.z - point1.z)**2)**0.5

    def calculate_planar_distance(self, point1, point2):
        """calculate_planar_distance: Calculate the planar distance between two points."""
        # 2D distance
        return ((point2.x - point1.x)**2 + (point2.y - point1.y)**2)**0.5

    def setup_challenge_started_time(self):
        """setup_challenge_started_time: Setup the challenge_started time."""
        if self.plant_beds_flag and self.challenge_start_flag:
            self.challenge_start_flag = True
            self.challenge_start_time = self.get_current_time()
            rospy.loginfo('[Evaluator] Challenge started, time set to zero')
            self.initial_position = self.last_position
            # rospy.loginfo('Initial position: %f, %f, %f',
            #               self.initial_position.x, self.initial_position.y, self.initial_position.z)

    def cb_challenge_started(self, msg):
        """cb_challenge_started: Callback for the challenge_started topic."""
        if msg.data:
            self.challenge_start_flag = True
            rospy.loginfo('[Evaluator] Challenge started received')
            self.setup_challenge_started_time()
        else:
            rospy.logwarn('Challenge started received False')

    def cb_plant_beds(self, msg):
        """cb_plant_beds: Callback for the plant_beds topic."""
        self.plant_beds_flag = True
        message = f"[Evaluator] Beds location received -> {msg.data}"
        rospy.loginfo(message)
        self.setup_challenge_started_time()

    def cb_fruit_count(self, msg):
        """cb_fruit_count: Callback for the fruit_count topic."""
        self.fruit_count_received = True
        message = f"Fruit count received -> count: {msg.data}"
        self.print_time(message, self.get_current_time())

    def cb_odom(self, msg):
        """cb_odom: Callback for the odometry topic."""
        current_position = msg.pose.pose.position
        if self.challenge_start_flag:
            self.distance_travelled += self.calculate_distance(
                self.last_position, current_position)

        if self.fruit_count_received:
            distance_to_initial_position = self.calculate_planar_distance(
                self.initial_position, current_position)
            if distance_to_initial_position < DISTANCE_THRESHOLD:
                self.print_time(
                    'UAV at start position at time',
                    self.get_current_time())
                self.print_time(
                    'Finished at time', self.get_current_time())
                rospy.loginfo('[Evaluator] Total path traversed: %f',
                              self.distance_travelled)
                self.fruit_count_received = False
        self.last_position = current_position


if __name__ == '__main__':
    Evaluator()
