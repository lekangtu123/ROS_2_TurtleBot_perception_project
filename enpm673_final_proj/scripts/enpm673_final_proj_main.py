#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from findhorizonline import HorizonDetector
import numpy as np
from geometry_msgs.msg import Twist
import torch
from ultralytics import YOLO
import math
import matplotlib.pyplot as plt
import time
import os
from launch_ros.substitutions import FindPackageShare


# Record the start time
start_time = time.time()

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        pkg_share = FindPackageShare(package='enpm673_final_proj').find('enpm673_final_proj')
        model_path = os.path.join(pkg_share, 'models',"sto.pt")


        
        # Initialize the CvBridge and flags
        self.bridge = CvBridge()
        self.horizon_detected = False
        self.stop_signal_detected = False
        self.frame_count = 0
        self.previous_frame = None
        self.wait_counter = 0
        
        # Set up ROS image subscription
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace this with your image topic
            self.image_callback,
            10)
        
        # Set up ROS command publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Load the YOLO model for stop sign detection
        self.stop_sign_model = YOLO(model_path)
        self.class_names = ["0"]
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.feature_params = dict(maxCorners=100, qualityLevel=0.1, minDistance=5, blockSize=5)
        self.fast_movement_count = 0

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.processed_image = self.current_image.copy()
            self.stop_sign_image = self.current_image.copy()
            self.display_image_1 = self.current_image.copy()
            self.display_image_2 = self.current_image.copy()
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))
            return
        
        # Detect horizon line if not detected yet
        if not self.horizon_detected:
            horizon_detector = HorizonDetector(self.current_image)
            self.vanishing_point = horizon_detector.detect_horizon()
            height, self.image_width = self.current_image.shape[:2]
            print(self.vanishing_point)
            self.horizon_detected = True
        
        # Perform optical flow, stop sign detection, and movement control
        self.perform_optical_flow()
        self.detect_stop_sign()
        self.control_movement()
        
        # Display the detected horizon line and vanishing point
        cv2.line(self.display_image_1, (0, self.vanishing_point[1]), (self.image_width, self.vanishing_point[1]), (0, 0, 255), 3)
        cv2.circle(self.display_image_1, self.vanishing_point, 3, (0, 255, 0), thickness=3)
        cv2.imshow("Image", self.display_image_1)
        cv2.waitKey(1)

    def detect_stop_sign(self):
        # Detect stop signs using the YOLO model
        results = self.stop_sign_model(self.stop_sign_image, stream=True)
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0]
                if confidence > 0.8:
                    # Draw bounding box and label for detected stop sign
                    cv2.rectangle(self.stop_sign_image, (x1, y1), (x2, y2), (255, 0, 255), 3)
                    cls = int(box.cls[0])
                    print("Class name -->", self.class_names[cls])
                    cv2.putText(self.stop_sign_image, "Stop sign", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    self.stop_signal_detected = True
                else:
                    self.stop_signal_detected = False
            cv2.imshow("yolo", self.stop_sign_image)
            cv2.waitKey(1)

    def control_movement(self):
        height, width, channels = self.current_image.shape
        
        # Define source and destination points for homography
        src_points = np.array([[0, 640], [270, 272], [375, 275], [640, 480]])
        dst_points = np.array([[0, 480], [0, 0], [640, 0], [640, 480]])
        
        # Compute homography and apply perspective warp
        homography_matrix, status = cv2.findHomography(src_points, dst_points)
        warped_image = cv2.warpPerspective(self.current_image, homography_matrix, (width, height))
        
        # Convert to grayscale and apply threshold
        gray_image = cv2.cvtColor(warped_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 253, 255, cv2.THRESH_BINARY)
        
        # Find contours and determine the nearest centroid
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        nearest_centroid_x, nearest_centroid_y = 0, 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:
                M = cv2.moments(contour)
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                if nearest_centroid_y < centroid_y:
                    nearest_centroid_y = centroid_y
                    nearest_centroid_x = centroid_x
                cv2.circle(warped_image, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
        
        print("Centroid", (nearest_centroid_x, nearest_centroid_y))
        cv2.imshow("Detected Centroid", warped_image)
        cv2.waitKey(1)
        
        # Generate and publish movement commands
        velocity_command = Twist()
        tolerance = 20
        if not self.stop_signal_detected:
            if width // 2 - tolerance < nearest_centroid_x < width // 2 + tolerance:
                print("Going Forward")
                velocity_command.angular.z = 0.0
                velocity_command.linear.x = 0.06
            elif nearest_centroid_x < width // 2:
                print("Rotating Anti-Clockwise")
                velocity_command.linear.x = 0.003
                velocity_command.angular.z = 0.05
            else:
                print("Rotating Clockwise")
                velocity_command.linear.x = 0.003
                velocity_command.angular.z = -0.05
        else:
            velocity_command.linear.x = 0.0
            velocity_command.angular.z = 0.0
        self.publisher.publish(velocity_command)

    def perform_optical_flow(self):
        if self.frame_count == 1:
            self.previous_frame = self.processed_image.copy()

        frame_height = self.current_image.shape[1]
        current_gray = cv2.cvtColor(self.processed_image, cv2.COLOR_BGR2GRAY)
        previous_gray = cv2.cvtColor(self.previous_frame, cv2.COLOR_BGR2GRAY)
        previous_gray = previous_gray[self.vanishing_point[1]:frame_height, :]
        current_gray = current_gray[self.vanishing_point[1]:frame_height, :]
        
        feature_params = dict(maxCorners=500, qualityLevel=0.01, minDistance=5, blockSize=7)
        initial_points = cv2.goodFeaturesToTrack(current_gray, **feature_params)
        mask = np.zeros_like(self.processed_image)

        if initial_points is not None:
            for point in initial_points:
                x, y = point.ravel()
                cv2.circle(self.display_image_1, (int(x), int(y)), 5, (255, 0, 0), -1)

        if self.frame_count >= 2 and initial_points is not None and initial_points.size > 0:
            new_points, status, err = cv2.calcOpticalFlowPyrLK(previous_gray, current_gray, initial_points, None, **self.lk_params)
            if new_points is not None and status.sum() > 0:
                good_new = new_points[status == 1]
                good_old = initial_points[status == 1]

                for new, old in zip(good_new, good_old):
                    velocity = np.linalg.norm(new - old)
                    if velocity > 25:
                        self.fast_movement_count += 1
                        if self.fast_movement_count > 10:
                            print(f"Bottle moving too fast: {velocity} px/frame")
                            velocity_command = Twist()
                            velocity_command.linear.x = 0.0
                            velocity_command.angular.z = 0.0
                            self.publisher.publish(velocity_command)
                            self.fast_movement_count = 0
                            self.wait_counter = 0
                    if self.wait_counter < 50:
                        self.stop_signal_detected = True
                        self.wait_counter += 1
                        break
                    else:
                        self.stop_signal_detected = False
                    x_new, y_new = new.ravel()
                    x_old, y_old = old.ravel()
                    mask = cv2.line(mask, (int(x_new), int(y_new) + self.vanishing_point[1]), (int(x_old), int(y_old) + self.vanishing_point[1]), (0, 255, 0), 2)
            else:
                print("Optical flow calculation failed or no valid points found.")
        else:
            print("No initial points to track or frame count is too low.")

        self.display_image_1 = cv2.add(self.processed_image, mask)
        cv2.imshow("Optical Flow", self.display_image_1)
        cv2.waitKey(1)
        self.previous_frame = self.processed_image.copy()
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
