import serial

class RadioHardware:
    def __init__(self, logger):
        # Setup the serial connection (Adjust to match your serial port and baudrate)
        self.serial_port = serial.Serial('/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30DQXAS-if00-port0', baudrate=9600, timeout=1)
        self.logger = logger

    def read_data(self):
        try:
            # Read the incoming message from XBee
            if self.serial_port.in_waiting > 0:
                raw_data = self.serial_port.readline().decode('utf-8').strip()
                # Split the data (assuming the format is 'a b')
                data_parts = raw_data.split(' ')
                
                if len(data_parts) == 2:
                    a = int(data_parts[0])
                    b = int(data_parts[1])

                    # Constrain the values within valid ranges
                    a = max(min(a, 90), -90)
                    b = max(min(b, 30), -30)
                    
                    return a, b
                else:
                    self.logger.info(f"Invalid radio message: {raw_data}")
        except Exception as e:
            self.logger.error(f"Error reading radio data: {e}")
        return None, None

    def close(self):
        self.serial_port.close()