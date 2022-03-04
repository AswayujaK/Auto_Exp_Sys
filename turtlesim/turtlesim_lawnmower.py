
# script for implementing a lawnmower path
# Maddie Schwarz
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Turtle:
    # Creates a node with name 'turtle_lawnmower'& make it unique(anonymous=True).
    def __init__(self):
        rospy.init_node('turtle_lawnmower', anonymous=True)
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'.self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

#forward motion controller function
    def move(self, direction, distance, speed):
        # direction -- specify 'horiz' or 'vert'
        global current_distance, t0
        isForward = True  # robot only moves forward
        vel_msg = Twist()
        if isForward:
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        # Param for moving along x or y axis
        if direction == 'x':
            # moving just in x-axis
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
        else:
            # moving just in y-axis
            vel_msg.linear.x = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        # Loop to move the turtle in an specified distance
        while current_distance < distance:
            # Publish the velocity
            self.vel_pub.publish(vel_msg)
            # Takes actual time to velocity calculus
            t1 = rospy.Time.now().to_sec()
            # Calculates distancePoseStamped
            current_distance = speed * (t1 - t0)
        # After the loop, stops turtle
        vel_msg.linear.x = 0
        # stop turtle
        self.vel_pub.publish(vel_msg)

#rotation controller function
    def rotate(self, rangle, clockwise):
        pi = 3.1415926535897
        rspeed = 30
        print("Let's rotate your robot")
        # Convert from angles to radians
        angular_speed = rspeed * 2 * pi / 360
        relative_angle = rangle * 2 * pi / 360
        vel_msg = Twist()
        # set linear components to zero (not in use)
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculation
        t0r = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < relative_angle:
            self.vel_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0r)

        # Stop the turtle
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)

#function directing lawnmower path
    def lawn_path(self):
        # Leg 1
        self.move('x', 4, 1)
        self.rotate(90, False)
        # Leg 2
        self.move('x', 2, 1)
        self.rotate(90, False)
        # Leg 3
        self.move('x', 4, 1)
        self.rotate(90, True)
        # Leg 4
        self.move('x', 2, 1)
        self.rotate(90, True)
        # Leg 5
        self.move('x', 4, 1)
        # Leg 6
        self.rotate(135, True)
        self.move('x', 5.5, 1)
        rospy.spin()

if __name__ == '__main__':
    try:
        x = Turtle()
        x.lawn_path()
    except rospy.ROSInterruptException:
        pass
