import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sailboat_interface.msg import SailTail  # Replace with the actual package name
import serial

class Radio(Node):
    def __init__(self):
        super().__init__('radio')
        
        # Create publisher for SailTail.msg
        #self.publisher_ = self.create_publisher(SailTail, 'servo', 10)
        self.publisher_rudder = self.create_publisher(Int32, 'radio-rudder', 10)
        self.publisher_sail = self.create_publisher(Int32, 'radio-sail', 10)

        # Setup the serial connection (Adjust to match your serial port and baudrate)
        self.serial_port = serial.Serial('/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30DQXAS-if00-port0', baudrate=9600, timeout=1)

        # Timer callback to check and read serial data periodically
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check every 100ms
        self.get_logger().info(f"Starting up radio module")

    def timer_callback(self):
        try:
            # Read the incoming message from XBee
            if self.serial_port.in_waiting > 0:
                raw_data = self.serial_port.readline().decode('utf-8').strip()
                # Split the data (assuming the format is 'a b')
                data_parts = raw_data.split(' ')
                
                if len(data_parts) == 2:
                    a = int(data_parts[0])
                    b = int(data_parts[1])

                    if(a >= 90):
                        a = 90
                    elif (a <= -90):
                        a = -90
                    
                    if(b >= 30):
                        b = 30
                    elif (b <= -30):
                        b = -30
                    
                    
                    # Convert the integer values to float32 for SailTail message
                    msg_sail = Int32()
                    msg_tail = Int32()
                    msg_sail.data = a
                    msg_tail.data = b
                    
                    # Publish the SailTail message
                    self.publisher_sail.publish(msg_sail)
                    self.publisher_rudder.publish(msg_tail)
                    self.get_logger().info(f"Published sail: {msg_sail.data}, tail: {msg_tail.data}")
                else:
                    self.get_logger().info(f"Invalid radio message: {raw_data}")
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
