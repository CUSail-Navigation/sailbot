import cv2
import numpy as np

class BuoyDetectorCV:
    def __init__(self, hsv_lower=None, hsv_upper=None, detection_threshold=100):
        # Initialize parameters with defaults or provided values
        self.hsv_lower = np.array(hsv_lower if hsv_lower else [0, 127, 63])
        self.hsv_upper = np.array(hsv_upper if hsv_upper else [100, 255, 191])
        self.detection_threshold = detection_threshold
        self.show_frames = False
    
    def update_parameters(self, hsv_lower=None, hsv_upper=None, detection_threshold=None):
        """Update parameters dynamically."""
        if hsv_lower:
            self.hsv_lower = np.array(hsv_lower)
        if hsv_upper:
            self.hsv_upper = np.array(hsv_upper)
        if detection_threshold is not None:
            self.detection_threshold = detection_threshold

    def increase_contrast(self, frame):
        """Enhance frame contrast.""" 
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_clahe = clahe.apply(l)
        lab_clahe = cv2.merge((l_clahe, a, b))
        return cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

    def reduce_glare(self, frame):
        """Reduce glare from the frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, glare_mask = cv2.threshold(blurred, 220, 255, cv2.THRESH_BINARY)
        glare_mask_inv = cv2.bitwise_not(glare_mask)
        return cv2.bitwise_and(frame, frame, mask=glare_mask_inv)

    def process_frame(self, frame):
        """Process a single frame to detect the buoy."""
        frame = self.reduce_glare(frame)
        frame = self.increase_contrast(frame)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > self.detection_threshold:
                x, y, w, h = cv2.boundingRect(largest_contour)
                buoy_center = (x + w // 2, y + h // 2)
                if self.show_frames:
                    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.circle(frame, buoy_center, 5, (255, 0, 0), -1)
                    cv2.imshow("Camera", frame)
                    cv2.waitKey(1)
                return buoy_center, frame
        if self.show_frames:
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            cv2.imshow("Camera", frame)
            cv2.waitKey(1) 
        return None, frame
