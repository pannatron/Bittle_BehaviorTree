#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge

class KalmanFilter:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0], [0,1,0,0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]], np.float32)
        self.kalman.processNoiseCov = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]], np.float32) * 0.03

    def predict(self):
        return self.kalman.predict()

    def correct(self, x, y):
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        return self.kalman.correct(measurement)

def enhance_image(image, blur_ksize, thresh_block_size, thresh_C):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)
    # Apply adaptive thresholding
    enhanced = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, thresh_block_size, thresh_C)
    return enhanced

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image,
            '/ipm/image',
            self.image_callback,
            2)
        self.publisher_ = self.create_publisher(Image, '/aruco_detected_image', 10)
        self.angle_publisher_ = self.create_publisher(Float32, '/aruco_angle', 10)  # Publisher for angle
        self.direction_publisher_ = self.create_publisher(Bool, '/aruco_direction', 10)  # Publisher for direction
        self.distance_publisher_ = self.create_publisher(Float32, '/aruco_distance', 10)  # Publisher for distance
        self.br = CvBridge()
        
        # Change the dictionary to a different one, e.g., DICT_4X4_50
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.kalman_filters = {}
        self.last_detected_corners = {}
        self.last_detected_directions = {}

        cv2.namedWindow('Detected Aruco')
        cv2.createTrackbar('adaptiveThreshConstant', 'Detected Aruco', 7, 50, self.nothing)
        cv2.createTrackbar('minMarkerPerimeterRate', 'Detected Aruco', 3, 100, self.nothing)
        cv2.createTrackbar('maxMarkerPerimeterRate', 'Detected Aruco', 40, 100, self.nothing)
        cv2.createTrackbar('blur_ksize', 'Detected Aruco', 5, 50, self.nothing)
        cv2.createTrackbar('thresh_block_size', 'Detected Aruco', 11, 50, self.nothing)
        cv2.createTrackbar('thresh_C', 'Detected Aruco', 2, 50, self.nothing)

        self.get_logger().info('Aruco Detector Node has been started.')

    def nothing(self, x):
        pass

    def calculate_vector_angle(self, vec1, vec2):
        dot_product = np.dot(vec1, vec2)
        magnitude_product = np.linalg.norm(vec1) * np.linalg.norm(vec2)
        angle = np.arccos(dot_product / magnitude_product) * 180 / np.pi
        cross_product = np.cross(vec1, vec2)
        if cross_product < 0:
            angle = -angle
        return angle

    def image_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg)
        
        # Update preprocessing parameters from Trackbars
        blur_ksize = max(cv2.getTrackbarPos('blur_ksize', 'Detected Aruco') | 1, 1)  # Ensure ksize is odd and >= 1
        thresh_block_size = max(cv2.getTrackbarPos('thresh_block_size', 'Detected Aruco') | 1, 3)  # Ensure block size is odd and >= 3
        thresh_C = cv2.getTrackbarPos('thresh_C', 'Detected Aruco')

        enhanced_frame = enhance_image(current_frame, blur_ksize, thresh_block_size, thresh_C)  # Apply filters to reduce wave effects
        
        # Update ArUco parameters from Trackbars
        self.aruco_params.adaptiveThreshConstant = max(cv2.getTrackbarPos('adaptiveThreshConstant', 'Detected Aruco'), 1)
        self.aruco_params.minMarkerPerimeterRate = max(cv2.getTrackbarPos('minMarkerPerimeterRate', 'Detected Aruco') / 100.0, 0.01)
        self.aruco_params.maxMarkerPerimeterRate = max(cv2.getTrackbarPos('maxMarkerPerimeterRate', 'Detected Aruco') / 10.0, 0.1)

        corners, ids, rejected = cv2.aruco.detectMarkers(enhanced_frame, self.aruco_dict, parameters=self.aruco_params)

        detected_ids = set()
        marker_positions = {}
        marker_directions = {}
        angle_diff = None  # Initialize angle_diff to None
        distance_between_markers = None  # Initialize distance to None
        
        if ids is not None:
            for corner, id in zip(corners, ids):
                id = int(id[0])
                if id not in [1, 2]:  # Filter to only show ID 1 and ID 2
                    continue
                
                detected_ids.add(id)

                if id not in self.kalman_filters:
                    self.kalman_filters[id] = KalmanFilter()
                
                # Use original corner positions for drawing
                original_corners = corner[0].copy()

                for i in range(4):  # Update each corner with Kalman filter
                    self.kalman_filters[id].correct(corner[0][i][0], corner[0][i][1])
                    prediction = self.kalman_filters[id].predict()
                    corner[0][i][0] = prediction[0]
                    corner[0][i][1] = prediction[1]

                center = np.mean(corner[0], axis=0)
                marker_positions[id] = center

                # Use Kalman filtered corner for direction calculation
                direction_vector = corner[0][1] - corner[0][0]
                direction_angle = np.arctan2(direction_vector[1], direction_vector[0]) * 180 / np.pi
                marker_directions[id] = direction_angle

                predicted_center = (int(center[0]), int(center[1]))
                cv2.circle(current_frame, predicted_center, 5, (0, 255, 0), -1)
                cv2.putText(current_frame, f"ID: {id}", predicted_center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.polylines(current_frame, [np.int32(original_corners)], True, (0, 255, 0), 2)  # Draw the bounding box

                # Draw the corners with different colors
                corner_colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
                for i, color in enumerate(corner_colors):
                    cv2.circle(current_frame, (int(original_corners[i][0]), int(original_corners[i][1])), 5, color, -1)
                    cv2.putText(current_frame, f"{i+1}", (int(original_corners[i][0]), int(original_corners[i][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
                
                # Save the last detected corners and directions
                self.last_detected_corners[id] = original_corners
                self.last_detected_directions[id] = direction_angle
        
        for id, kf in self.kalman_filters.items():
            if id not in detected_ids and id in [1, 2]:  # Filter to only show ID 1 and ID 2
                # Use last detected corners and directions if available
                if id in self.last_detected_corners:
                    corner = self.last_detected_corners[id]
                    center = np.mean(corner, axis=0)
                    marker_positions[id] = center
                    marker_directions[id] = self.last_detected_directions[id]

                    predicted_center = (int(center[0]), int(center[1]))
                    cv2.circle(current_frame, predicted_center, 5, (255, 0, 0), -1)  # Use different color to indicate predicted values
                    cv2.putText(current_frame, f"ID: {id} (pred)", predicted_center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    cv2.polylines(current_frame, [np.int32(corner)], True, (255, 0, 0), 2)  # Draw the bounding box

                    # Draw the corners with different colors
                    corner_colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
                    for i, color in enumerate(corner_colors):
                        cv2.circle(current_frame, (int(corner[i][0]), int(corner[i][1])), 5, color, -1)
                        cv2.putText(current_frame, f"{i+1}", (int(corner[i][0]), int(corner[i][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
                else:
                    # If no previous data is available, predict using Kalman filter
                    prediction = kf.predict()
                    predicted_center = (int(prediction[0]), int(prediction[1]))
                    cv2.circle(current_frame, predicted_center, 5, (0, 255, 0), -1)
                    cv2.putText(current_frame, f"ID: {id}", predicted_center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    marker_positions[id] = predicted_center

        # Check direction from ID1 to ID2
        if 1 in marker_directions and 2 in marker_directions:
            angle_diff = (marker_directions[2] - marker_directions[1]) % 360
            if angle_diff > 180:
                angle_diff -= 360

            # Check if the first corner of ID1 is the closest to the robot
            if 1 in marker_positions and 2 in marker_positions:
                distance_to_robot = np.linalg.norm(marker_positions[1] - marker_positions[2])
                
                # Get distances from ID1's corners to the center of ID2
                corner_distances = [np.linalg.norm(marker_positions[2] - corner) for corner in self.last_detected_corners[1]]
                center1 = (self.last_detected_corners[1][0] + self.last_detected_corners[1][1]) / 2
                center2 = (self.last_detected_corners[1][2] + self.last_detected_corners[1][3]) / 2
                
                distance1 = np.linalg.norm(marker_positions[2] - center1)
                distance2 = np.linalg.norm(marker_positions[2] - center2)
                
                vector1 = center1 - marker_positions[2]
                vector2 = center2 - marker_positions[2]
                vector_to_id2 = self.last_detected_corners[1][0] - marker_positions[2]
                
                # Calculate vector angles using new method
                vec1 = self.last_detected_corners[1][0] - self.last_detected_corners[2][1]
                vec2 = self.last_detected_corners[1][2] - self.last_detected_corners[2][3]
                
                angle1 = self.calculate_vector_angle(vec1, vec2)
                angle2 = self.calculate_vector_angle(vector2, vector_to_id2)
                
                direction_msg = Bool()
                angle_msg = Float32()
                distance_msg = Float32()
                distance_between_markers = float(np.linalg.norm(marker_positions[1] - marker_positions[2]))  # Calculate distance
                distance_msg.data = distance_between_markers
                self.distance_publisher_.publish(distance_msg)
                cv2.putText(current_frame, f"Distance: {distance_between_markers:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

                cv2.line(current_frame, tuple(np.int32(self.last_detected_corners[1][0])), tuple(np.int32(self.last_detected_corners[2][1])), (0, 255, 0), 2)
                if distance1 < distance2:
                    cv2.line(current_frame, tuple(np.int32(marker_positions[2])), tuple(np.int32(center1)), (0, 255, 0), 2)
                    cv2.putText(current_frame, f"Angle Diff: {float(angle1):.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    self.get_logger().info(f"correct direction with angle {angle1:.2f}")
                    angle_msg.data = angle1
                    direction_msg.data = True
                else:
                    cv2.line(current_frame, tuple(np.int32(marker_positions[2])), tuple(np.int32(center2)), (0, 0, 255), 2)
                    cv2.putText(current_frame, f"Angle Diff: {float(angle2):.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    self.get_logger().info(f"ID1 is not in the correct direction with angle {angle2:.2f}")
                    angle_msg.data = angle2
                    direction_msg.data = False
                
                self.angle_publisher_.publish(angle_msg)
                self.direction_publisher_.publish(direction_msg)

        self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))
        cv2.imshow("Detected Aruco", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
