import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sailboat_interfaces.msg import SailTail  # Replace with the actual package name
import serial

class Radio(Node):
    def __init__(self):
        super().__init__('radio')
        
        # Create publisher for SailTail.msg
        self.publisher_ = self.create_publisher(SailTail, 'servo', 10)
        
        # Setup the serial connection (Adjust to match your serial port and baudrate)
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)

        # Timer callback to check and read serial data periodically
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check every 100ms

    def timer_callback(self):
        try:
            if self.serial_port.in_waiting > 0:
                # Read the incoming message from XBee
                raw_data = self.serial_port.readline().decode('utf-8').strip()
                # Split the data (assuming the format is 'a b')
                data_parts = raw_data.split(' ')
                
                if len(data_parts) == 2:
                    a = int(data_parts[0])
                    b = int(data_parts[1])
                    
                    # Convert the integer values to float32 for SailTail message
                    msg = SailTail()
                    msg.sail = float(a)
                    msg.tail = float(b)
                    
                    # Publish the SailTail message
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published sail: {msg.sail}, tail: {msg.tail}")
        except Exception as e:
            self.get_logger().error(f"Error reading radio data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Radio()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.serial_port.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
