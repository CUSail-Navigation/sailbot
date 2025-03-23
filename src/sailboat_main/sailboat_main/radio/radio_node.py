import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from sailboat_main.radio.radio_fake import FakeRadio
from sailboat_main.radio.radio_real import RadioHardware

class Radio(Node):
    """
    ROS 2 Node that implements radio communication from the boat to a base station. 
    The node reads the data from the radio interface and publishes it to control the sail
    and rudder. The node also subscribes to "interesting" publishers that contain useful
    telemetry and sends it via the radio interface.

    The control period (how often control data is scanned for), and the 
    telemetry period (how often telemetry data is sent) are configurable via the 
    config file.
    """
    def __init__(self):
            super().__init__('radio')

            # Declare "simulated" parameter
            self.declare_parameter('simulated', False)
            self.simulated = self.get_parameter('simulated').value  # gets from config file
            self.control_period = self.get_parameter('control_period').value
            self.timer_period = self.get_parameter('timer_period').value
            
            # Create publisher for SailTail.msg
            self.publisher_rudder = self.create_publisher(Int32, 'radio_rudder', 10)
            self.publisher_sail = self.create_publisher(Int32, 'radio_sail', 10)

            # Depending on simulation flag, instantiate RadioHardware or FakeRadio
            if self.simulated:
                self.radio_interface = FakeRadio()
            else:
                self.radio_interface = RadioHardware(self.get_logger())

            # Timer callbacks to read and send data periodically                
            self.control_timer = self.create_timer(self.control_period, self.control_callback)  # TODO: are these times acceptable? Check every 100ms
            self.telemetry_timer = self.create_timer(self.timer_period, self.telemetry_callback)  # Check every 500ms

            # TODO: Add subscribers that read from "interesting" publishers. 
            # Select these publishers based on what the webserver displays or 
            # what you think is needed.

            self.get_logger().info(f"Starting up radio module in {'simulated' if self.simulated else 'hardware'} mode")

    # TODO: Add subscriber callback functions that will be executed whenever an "interesting" topic has new data. 
    # These functions should update a class attribute that contains the "most recent data"

    def control_callback(self):
        """
        Callback function for the controlling the sail and rudder. Reads the data 
        from the radio interface and publishes it to the SailTail topic.
        """
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

    def telemetry_callback(self):
        """
        Callback function for the telemetry. Reads the data from "interesting" 
        publishers and sends it to the radio interface.
        """

        # TODO: Implement this method: it should
        # 1) format all data into some kind of packet 
        # 2) send this packet via the radio
        # 2b) a "send" function will need to be implememented in radio_real.py and radio_fake.py
        pass

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
