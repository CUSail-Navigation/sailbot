class SailServoFake:
    """
    Simulates servo communication for testing without real hardware.
    Prints the commands to the console instead of sending them to a physical device.
    """
    def __init__(self):
        print("Simulated SailServo initialized.")

    def send_command(self, sail, tail):
        """
        Simulate sending a command to the servo.
        :param sail: Sail position
        :param tail: Tail position
        """
        print(f'[SIMULATION] Sail: {sail}, Tail: {tail}')

    def close(self):
        """
        Simulate closing the connection (no-op for simulated servo).
        """
        print('Simulated SailServo closed.')
