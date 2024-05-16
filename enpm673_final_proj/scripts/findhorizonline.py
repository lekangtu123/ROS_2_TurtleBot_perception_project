import cv2
import numpy as np
import math
import random

class HorizonDetector:
    def __init__(self, frame):
        self.frame = frame
        self.iterations = 400
        self.threshold = 13
        self.inlier_ratio = 0.93
        self.frame_copy = frame.copy()
        self.frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.edges = cv2.Canny(self.gray_frame, 60, 150, apertureSize=3)
        self.detected_lines = cv2.HoughLines(self.edges, 1, np.pi / 180, 130)

    def calculate_intersection(self, line1, line2):
        """Find intersection point of two lines."""
        rho1, theta1 = line1[0]
        rho2, theta2 = line2[0]
        A = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
        b = np.array([rho1, rho2])
        det_A = np.linalg.det(A)
        if det_A != 0:
            x, y = np.linalg.solve(A, b).ravel()
            x, y = int(np.round(x)), int(np.round(y))
            return x, y
        else:
            return None

    def calculate_distance_to_line(self, point, line):
        """Calculate the perpendicular distance from a point to a line."""
        x0, y0 = point
        rho, theta = line[0]
        m = (-1 * (np.cos(theta))) / np.sin(theta)
        c = rho / np.sin(theta)
        x = (x0 + m * y0 - m * c) / (1 + m ** 2)
        y = (m * x0 + (m ** 2) * y0 - (m ** 2) * c) / (1 + m ** 2) + c
        dist = math.sqrt((x - x0) ** 2 + (y - y0) ** 2)
        return dist

    def ransac_algorithm(self, lines, iterations, threshold, inlier_ratio):
        """Apply the RANSAC algorithm to find the vanishing point."""
        best_inlier_ratio = 0.
        vanishing_point = (0, 0)
        for _ in range(iterations):
            # Randomly select two lines
            selected_lines = random.sample(lines, 2)
            line1, line2 = selected_lines
            intersection_point = self.calculate_intersection(line1, line2)
            if intersection_point is not None:
                inlier_count = 0
                for line in lines:
                    dist = self.calculate_distance_to_line(intersection_point, line)
                    if dist < threshold:
                        inlier_count += 1
                inlier_ratio_current = inlier_count / float(len(lines))
                if inlier_ratio_current > best_inlier_ratio:
                    best_inlier_ratio = inlier_ratio_current
                    vanishing_point = intersection_point
                if inlier_count > len(lines) * inlier_ratio:
                    break
        return vanishing_point

    def detect_horizon(self):
        """Detect the horizon line in the frame."""
        if self.detected_lines is not None:
            lines = [line for line in self.detected_lines]
            valid_lines = []
            for line in lines:
                rho, theta = line[0]
                # Filter lines based on angle to select near-horizontal lines
                if (0.4 < theta < 1.4) or (1.7 < theta < 2.8):
                    valid_lines.append(line)

            if valid_lines:
                vanishing_point = self.ransac_algorithm(valid_lines, self.iterations, self.threshold, self.inlier_ratio)
                return vanishing_point
            else:
                print("No valid lines found for RANSAC.")
        else:
            print("No lines detected.")
        return None
