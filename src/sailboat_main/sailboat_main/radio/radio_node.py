import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from sailboat_main.radio.radio_fake import FakeRadio
from sailboat_main.radio.radio_real import RadioHardware

class Radio(Node):
    def __init__(self):
            super().__init__('radio')

            # Declare "simulated" parameter
            self.declare_parameter('simulated', False)
            self.simulated = self.get_parameter('simulated').value  # gets from config file

            # Declare "source" parameter. Determines whether to use XBee or Webserver comms
            self.declare_parameter('source', 'webserver')
            self.source = self.get_parameter('source').value

            # Create publisher for SailTail.msg
            self.publisher_rudder = self.create_publisher(Int32, 'radio_rudder', 10)
            self.publisher_sail = self.create_publisher(Int32, 'radio_sail', 10)

            # If we are using the webserver, the webserver will automatically publish the data 
            # to radio_rudder and radio_sail so we don't need to do anything here.
            if self.source != 'webserver':            
                # Depending on simulation flag, instantiate RadioHardware or FakeRadio
                if self.simulated:
                    self.radio_interface = FakeRadio()
                else:
                    self.radio_interface = RadioHardware(self.get_logger())
                
                # Timer callback to check and read data periodically
                self.timer = self.create_timer(0.1, self.timer_callback)  # Check every 100ms
                self.get_logger().info(f"Starting up radio module in {'simulated' if self.simulated else 'hardware'} mode")

    def timer_callback(self):
        a, b = self.radio_interface.read_data()
        
        if a is not None and b is not None:
            # Convert the integer values to Float32 for SailTail message
            msg_sail = Int32()
            msg_rudder = Int32()
            msg_sail.data = a
            msg_rudder.data = b
            
            # Publish the SailTail message
            self.publisher_sail.publish(msg_sail)
            self.publisher_rudder.publish(msg_rudder)
            self.get_logger().info(f"Published sail: {msg_sail.data}, rudder: {msg_rudder.data}")

    def destroy_node(self):
        # Call the cleanup method for the hardware interface
        self.radio_interface.close()
        super().destroy_node()

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
