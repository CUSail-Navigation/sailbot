import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from buoy_detection import BuoyDetectorCV
from std_msgs.msg import Int32
import cv2

class BuoyDetectorNode(Node):
    def __init__(self):
        super().__init__('buoy_detector_node')

        self.declare_parameter('video_source', 0)
        self.declare_parameter('hsv_lower', [0, 120, 180])
        self.declare_parameter('hsv_upper', [10, 160, 255])
        self.declare_parameter('detection_threshold', 100)
        self.declare_parameter('timer_period', 0.1)  # Timer period for frame processing
        
        
        self.CENTER = 300 # TODO: Actually find the center and margin of error
        self.MARGIN = 30
        self.angle = 0

        self.video_source = self.get_parameter('video_source').value
        hsv_lower = self.get_parameter('hsv_lower').value
        hsv_upper = self.get_parameter('hsv_upper').value
        detection_threshold = self.get_parameter('detection_threshold').value
        self.timer_period = self.get_parameter('timer_period').value

        self.detector = BuoyDetectorCV(hsv_lower, hsv_upper, detection_threshold)

        self.displacement_publisher = self.create_publisher(Int32, '/buoy_displacement', 10)
        self.position_publisher = self.create_publisher(Int32, '/buoy_position', 10)

        # OpenCV video capture
        self.cap = cv2.VideoCapture(self.video_source)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open video source {self.video_source}')
            raise RuntimeError(f'Cannot open video source {self.video_source}')

        self.timer = self.create_timer(self.timer_period, self.process_frame)

        self.subscription = self.create_subscription(
            Int32,
            'buoy_angle',
            self.buoy_angle_callback,
            10)

        self.get_logger().info('Buoy Detector Node Initialized')

    def process_frame(self):

        """Process a single frame and publish the buoy position."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read frame from video source')
            return

        buoy_center, _ = self.detector.process_frame(frame)

        displacement = 0
        if not buoy_center:
            return
        elif buoy_center[0] > self.CENTER + self.MARGIN:
            displacement = 1
        elif buoy_center[0] < self.CENTER - self.MARGIN:
            displacement = -1
        if displacement:
            self.displacement_publisher.publish(displacement)
            self.get_logger().info(f'Detected buoy displacement: {displacement}')
        else:
            point_msg = Point()
            point_msg.x, point_msg.y = buoy_center[0], buoy_center[1]
            point_msg.z = abs(point_msg.x - self.CENTER) / math. atan(self.angle)
            self.position_publisher.publish(point_msg)
            self.get_logger().info(f'Detected buoy at: ({point_msg.x, point_msg.y, point_msg.z})')
    
    def buoy_angle_callback(self, msg):
        self.angle = abs(90 - msg.data)

    def update_detector_parameters(self):
        """Update the CV detector parameters dynamically."""
        hsv_lower = self.get_parameter('hsv_lower').value
        hsv_upper = self.get_parameter('hsv_upper').value
        detection_threshold = self.get_parameter('detection_threshold').value

        self.detector.update_parameters(hsv_lower, hsv_upper, detection_threshold)
        self.get_logger().info(f"Updated parameters: HSV Lower: {hsv_lower}, HSV Upper: {hsv_upper}, Detection Threshold: {detection_threshold}")

    def destroy_node(self):
        """Release the video capture and destroy the node."""
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
