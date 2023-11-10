#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from ultralytics.yolo.engine.results import Results

# ROS
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from your_robot_pkg.srv import HumanDetection

class ImageProcessor:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        self.image_msg = Image()
        self.image_res = 240, 320, 3
        self.image_np = np.zeros(self.image_res)

        self.camera_subscriber = rospy.Subscriber('/follower/camera/image', Image, self.camera_listener)
        self.model: YOLO = YOLO('/home/jingnew/catkin_ws/src/turtlebot3_object_tracker/yolo/yolov8n.pt')
        self.results: Results = None

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        self.human_detection_server = rospy.Service('human_detection_service', HumanDetection, self.handle_human_detection)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        self.update_view()

    def camera_listener(self, msg: Image):
        self.image_msg = msg

    def handle_human_detection(self, request):
        response = self.detect_human()
        rospy.loginfo("Service called")
        return response

    def detect_human(self):
        detected = False
        confidence = 0.0

        if self.results is not None:
            # "person" 클래스의 index 가져오기
            person_class_index = self.model.names.index("person")

            for pred in self.results.xyxy[0]:
                if int(pred[5]) == person_class_index and float(pred[4]) > 0.8:
                    detected = True
                    confidence = float(pred[4])
                    break

        return detected, confidence

    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0:
                    continue

                self.image_np = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")
                frame = copy.deepcopy(self.image_np)

                self.results = self.model(frame)

                cv2.imshow("robot_view", frame)
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass

if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()

