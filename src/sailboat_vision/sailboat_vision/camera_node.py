import cv2
import math
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Int32, Float32
from std_msgs.msg import Int32MultiArray

from sailboat_vision.buoy_detection import BuoyDetectorCV

class BuoyDetectorNode(Node):
    def __init__(self):
        super().__init__('buoy_detector_node')

        self.declare_parameter('hsv_lower', [0, 127, 63])
        self.declare_parameter('hsv_upper', [20, 255, 255] )
        self.declare_parameter('detection_threshold', 100)
        self.declare_parameter('timer_period', 0.1)  # Timer period for frame processing
        self.declare_parameter('show_frames', False)
        
        self.hsv_lower = self.get_parameter('hsv_lower').value
        self.hsv_upper = self.get_parameter('hsv_upper').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.timer_period = self.get_parameter('timer_period').value
        self.show_frames = self.get_parameter('show_frames').value
        
        # Subscriptions for dynamic parameter tuning
        self.hsv_lower_sub = self.create_subscription(
            Int32MultiArray,
            'update_hsv_lower',
            self.hsv_lower_callback,
            10)

        self.hsv_upper_sub = self.create_subscription(
            Int32MultiArray,
            'update_hsv_upper',
            self.hsv_upper_callback,
            10)

        self.detection_threshold_sub = self.create_subscription(
            Int32,
            'update_detection_threshold',
            self.detection_threshold_callback,
            10)

        self.position_publisher = self.create_publisher(Point, 'buoy_position', 10)
        # self.buoy_distance_publisher = self.create_publisher(Float32, 'buoy_distance', 10)

        self.timer = self.create_timer(self.timer_period, self.process_frame)

        # Initialize camera
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.detector = BuoyDetectorCV(self.hsv_lower, self.hsv_upper, self.detection_threshold, self.show_frames)
        self.current_mode = "manual"  # Default mode
        self.current_mode_sub = self.create_subscription(String,'current_mode',self.mode_callback,10)

        self.get_logger().info('Buoy Detector Node Initialized')

    def process_frame(self):
        """Check if a buoy is in the camera frame and calculate its position"""
        if self.current_mode != 'search': 
            return

        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            return

        # Check color frame for buoy
        color_image = np.asanyarray(color_frame.get_data())
        buoy_center, _ = self.detector.process_frame(color_image)
        if not buoy_center:
            self.get_logger().info(f'No buoy detected')
            return

        # Calculate and publish buoy position relative to boat
        depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        depth = aligned_depth_frame.get_distance(buoy_center[0], buoy_center[1])
        point_msg = Point()
        point_msg.x, point_msg.z, point_msg.y = rs.rs2_deproject_pixel_to_point(depth_intrinsics, buoy_center, depth)
        self.position_publisher.publish(point_msg)
        self.get_logger().info(f'Detected buoy at: ({point_msg.x, point_msg.y, point_msg.z})')

        # Calculate distance to buoy
        # buoy_distance = math.sqrt(point_msg.x**2 + point_msg.y**2)
        # buoy_distance_msg = Float32()
        # buoy_distance_msg.data = buoy_distance
        # self.buoy_distance_publisher.publish(buoy_distance_msg)
        # self.get_logger().info(f'Buoy distance: {buoy_distance_msg.data}')


    def update_detector_params(self):
        """Update the CV detector parameters dynamically."""
        self.detector.update_parameters(self.hsv_lower, self.hsv_upper, self.detection_threshold)
        self.get_logger().info(f"Updated parameters: HSV Lower: {self.hsv_lower}, HSV Upper: {self.hsv_upper}, Detection Threshold: {self.detection_threshold}")

    def destroy_node(self):
        """Release the video capture and destroy the node."""
        self.pipeline.stop()
        super().destroy_node()

    def hsv_lower_callback(self, msg: Int32MultiArray):
        if len(msg.data) == 3:
            self.hsv_lower = msg.data
            self.update_detector_params()
        else:
            self.get_logger().warn('HSV Lower must have 3 elements (H, S, V)')

    def hsv_upper_callback(self, msg: Int32MultiArray):
        if len(msg.data) == 3:
            self.hsv_upper = msg.data
            self.update_detector_params()
        else:
            self.get_logger().warn('HSV Upper must have 3 elements (H, S, V)')

    def detection_threshold_callback(self, msg: Int32):
        self.detection_threshold = msg.data
        self.update_detector_params()
        
    def mode_callback(self, msg) :
        new_mode = msg.data
        if self.current_mode == 'search' and new_mode != 'search':
            self.get_logger().info(f"Exiting search mode due to mode change to '{new_mode}'.")

        self.current_mode = new_mode

        if self.current_mode == 'search':
            self.get_logger().info("Search mode activated.")
           

def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
