"""
File:        SerialStepperTransmitter.py
Package:     Scrap-2-SCARA
Author:      Alexandre Allonas
Date:        
    Creation:       21/08/2025
    Last Update:    15/09/2025

Description:
    This node transmits the joint positions to the arduino over serial

Subscriptions:
    - stepper_targets (std_msgs/Int32MultiArray): Target positions for the stepper motors

Publications:
    - stepper_positions (std_msgs/Int32MultiArray): Current positions of the stepper motors

Services:
    - /

Parameters:
    - /

Notes:
    /
"""
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct
import threading
import time

class StepperControlNode(Node):
    def __init__(self):
        super().__init__('stepper_control_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Adjust for your system
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to Arduino on {serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            return
        
        # ROS2 publishers and subscribers
        self.target_subscriber = self.create_subscription(
            Int32MultiArray,
            'stepper_targets',
            self.target_callback,
            10
        )
        
        self.position_publisher = self.create_publisher(
            Int32MultiArray,
            'stepper_positions',
            10
        )
        
        # Communication protocol constants
        self.HEADER = 0xAA
        self.FOOTER = 0x55
        self.PACKET_SIZE = 18  # Header(1) + 4*INT32(16) + Footer(1)
        
        # Current target positions
        self.target_positions = [0, 0, 0, 0]
        
        # Start serial reading thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_reader_thread)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        # Timer for sending commands
        self.command_timer = self.create_timer(0.1, self.send_command)  # 10Hz
        
        self.get_logger().info('Stepper Control Node initialized')
    
    def target_callback(self, msg):
        """Callback for receiving target positions"""
        if len(msg.data) >= 4:
            self.target_positions = list(msg.data[:4])  # Take first 4 values
            self.get_logger().info(f'Received targets: {self.target_positions}')
        else:
            self.get_logger().warn('Received target array with less than 4 elements')
    
    def send_command(self):
        """Send target positions to Arduino"""
        try:
            # Create packet: Header + 4*INT32 + Footer
            packet = bytearray()
            packet.append(self.HEADER)
            
            # Pack 4 INT32 values (little-endian)
            for pos in self.target_positions:
                # Ensure position is within INT32 range
                pos = max(-2147483648, min(2147483647, int(pos)))
                packet.extend(struct.pack('<i', pos))  # '<i' = little-endian signed 32-bit
            
            packet.append(self.FOOTER)
            
            # Send packet
            self.serial_conn.write(packet)
            self.serial_conn.flush()
            
        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')
    
    def serial_reader_thread(self):
        """Thread function for reading serial data"""
        buffer = bytearray()
        
        while self.running and rclpy.ok():
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(data)
                    
                    # Look for complete packets
                    while len(buffer) >= self.PACKET_SIZE:
                        # Find header
                        header_index = buffer.find(self.HEADER)
                        if header_index == -1:
                            # No header found, clear buffer
                            buffer.clear()
                            break
                        
                        # Remove data before header
                        if header_index > 0:
                            buffer = buffer[header_index:]
                        
                        # Check if we have a complete packet
                        if len(buffer) >= self.PACKET_SIZE:
                            if buffer[self.PACKET_SIZE - 1] == self.FOOTER:
                                # Valid packet found
                                packet = buffer[:self.PACKET_SIZE]
                                self.process_position_packet(packet)
                                buffer = buffer[self.PACKET_SIZE:]
                            else:
                                # Invalid packet, remove header and continue
                                buffer = buffer[1:]
                        else:
                            break
                
                time.sleep(0.001)  # Small delay to prevent excessive CPU usage
                
            except Exception as e:
                self.get_logger().error(f'Error in serial reader: {e}')
                time.sleep(0.1)
    
    def process_position_packet(self, packet):
        """Process received position packet from Arduino"""
        try:
            # Extract 4 INT32 values (skip header and footer)
            positions = []
            for i in range(4):
                start_idx = 1 + i * 4  # Skip header byte
                pos_bytes = packet[start_idx:start_idx + 4]
                pos = struct.unpack('<i', pos_bytes)[0]  # little-endian signed 32-bit
                positions.append(pos)
            
            # Publish positions
            msg = Int32MultiArray()
            msg.data = positions
            self.position_publisher.publish(msg)
            
            # Log occasionally (not every packet to avoid spam)
            if hasattr(self, '_last_log_time'):
                if time.time() - self._last_log_time > 1.0:  # Log every second
                    self.get_logger().info(f'Current positions: {positions}')
                    self._last_log_time = time.time()
            else:
                self._last_log_time = time.time()
                
        except Exception as e:
            self.get_logger().error(f'Error processing position packet: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if hasattr(self, 'serial_thread'):
            self.serial_thread.join(timeout=1.0)
        if hasattr(self, 'serial_conn'):
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StepperControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()