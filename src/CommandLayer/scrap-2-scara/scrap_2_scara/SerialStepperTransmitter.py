import rclpy
from rclpy.node import Node
import serial
import time

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_controller')

        # Change to your Arduino port
        port = "/dev/ttyACM0"   # or COM3 on Windows
        baudrate = 115200

        # Open serial connection
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # wait for Arduino to reset
        self.get_logger().info(f"Serial connection established on {port}")

        # Example sequence of step positions
        self.positions = [0, 200, 400, 200, 0, -200]
        self.index = 0

        # Create a timer to send positions every 3 seconds
        self.timer = self.create_timer(3.0, self.send_position)

    def send_position(self):
        if self.index < len(self.positions):
            pos = self.positions[self.index]
            msg = f"{pos}\n"
            self.ser.write(msg.encode('utf-8'))
            self.get_logger().info(f"Sent target position: {pos}")
            self.index += 1
        else:
            self.get_logger().info("Finished sending all positions.")
            self.timer.cancel()

    def destroy_node(self):
        self.ser.close()
        self.get_logger().info("Serial connection closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
