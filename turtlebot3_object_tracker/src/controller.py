#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
from your_robot_pkg.srv import HumanDetection

class Controller:
    def __init__(self) -> None:
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        self.angular_vel_coef = 1

        # Create a service proxy for the human detection service
        rospy.wait_for_service('human_detection_service')
        self.human_detection_service = rospy.ServiceProxy('human_detection_service', HumanDetection)

        # Create a publisher for the robot "cmd_vel"
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                # Call the human detection service
                detected, confidence = self.human_detection_service()

                if detected and confidence > 0.8:
                    self.cmd_vel_publisher.publish(self.move)
                else:
                    self.cmd_vel_publisher.publish(self.freeze)

        except rospy.exceptions.ROSInterruptException:
            pass

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()

