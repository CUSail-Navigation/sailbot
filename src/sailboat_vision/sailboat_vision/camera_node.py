import cv2
import math
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32

from sailboat_vision.buoy_detection import BuoyDetectorCV

class BuoyDetectorNode(Node):
    def __init__(self):
        super().__init__('buoy_detector_node')

        self.declare_parameter('hsv_lower', [0, 120, 180])
        self.declare_parameter('hsv_upper', [10, 160, 255])
        self.declare_parameter('detection_threshold', 100)
        self.declare_parameter('timer_period', 0.1)  # Timer period for frame processing
        self.declare_parameter('servo_angle_step', 5) # How much to shift servo by each time
        
        hsv_lower = self.get_parameter('hsv_lower').value
        hsv_upper = self.get_parameter('hsv_upper').value
        detection_threshold = self.get_parameter('detection_threshold').value
        self.timer_period = self.get_parameter('timer_period').value
        self.servo_angle_step = self.get_parameter('servo_angle_step').value

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.detector = BuoyDetectorCV(hsv_lower, hsv_upper, detection_threshold)
        
        self.CENTER = 320
        self.MARGIN = 30 # TODO: Find best value for margin of error
        self.angle = 0
        self.servo_angle = 90

        self.position_publisher = self.create_publisher(Point, '/buoy_position', 10)
        self.servo_angle_publisher = self.create_publisher(Int32, '/servo_angle', 10)

        self.timer = self.create_timer(self.timer_period, self.process_frame)

        self.subscription = self.create_subscription(
            Int32,
            'buoy_angle',
            self.buoy_angle_callback,
            10)

        self.get_logger().info('Buoy Detector Node Initialized')

    def process_frame(self):

        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        buoy_center, _ = self.detector.process_frame(color_image)

        if not buoy_center:
            self.get_logger().info(f'No buoy detected')
            return
 
        displacement = abs(buoy_center[0] - self.CENTER)
        if displacement > self.MARGIN:
            if buoy_center[0] > self.CENTER + self.MARGIN:
                servo_angle = max(0, self.servo_angle - self.servo_angle_step)
            #move servo left (max of 180)
            elif buoy_center[0] < self.CENTER - self.MARGIN:
                servo_angle = min(180, self.servo_angle + self.servo_angle_step)
            
            angle_msg = Int32()
            angle_msg.data = servo_angle
            self.servo_angle_publisher.publish(angle_msg)
            self.get_logger().info(f'Detected buoy displacement: {displacement}')
            self.get_logger().info(f'Servo angle: {servo_angle}')
        else:
            depth = depth_image[buoy_center[0], buoy_center[1]] / 1000
            point_msg = Point()
            point_msg.x = depth * math.sin(math.radians(self.angle - 90))
            point_msg.y = depth * math.cos(math.radians(self.angle - 90))
            point_msg.z = float(buoy_center[1])
            self.position_publisher.publish(point_msg)
            self.get_logger().info(f'Detected buoy at: ({point_msg.x, point_msg.y, point_msg.z})')
    
    def buoy_angle_callback(self, msg):
        self.angle = msg.data

    def update_detector_parameters(self):
        """Update the CV detector parameters dynamically."""
        hsv_lower = self.get_parameter('hsv_lower').value
        hsv_upper = self.get_parameter('hsv_upper').value
        detection_threshold = self.get_parameter('detection_threshold').value

        self.detector.update_parameters(hsv_lower, hsv_upper, detection_threshold)
        self.get_logger().info(f"Updated parameters: HSV Lower: {hsv_lower}, HSV Upper: {hsv_upper}, Detection Threshold: {detection_threshold}")

    def destroy_node(self):
        """Release the video capture and destroy the node."""
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
