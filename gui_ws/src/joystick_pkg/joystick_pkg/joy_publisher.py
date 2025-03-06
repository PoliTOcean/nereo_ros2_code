import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import os
import struct

class JoystickPublisher(Node):
    def __init__(self, joystick_path='/dev/input/js0'):
        super().__init__('joystick_publisher')
        
        try:
            self.joystick_fd = os.open(joystick_path, os.O_RDONLY | os.O_NONBLOCK)
            self.publisher = self.create_publisher(Joy, '/joy', 10)
            self.timer = self.create_timer(0.01, self.publish_joystick_data)
            
            # Preallocate with reasonable defaults
            self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            
            self.get_logger().info(f"Connected to joystick: {joystick_path}")
        
        except Exception as e:
            self.get_logger().error(f'Joystick error: {e}')

    def publish_joystick_data(self):
        try:
            while True:
                try:
                    event = os.read(self.joystick_fd, 8)
                    self.get_logger().info(f'Joystick event: {event}')
                except BlockingIOError:
                    break

                # Joystick event structure: time, value, type, number
                _, value, event_type, number = struct.unpack('IhBB', event)
                
                # Axis event
                if event_type & 0x02 and number < len(self.axes):
                    self.axes[number] = value / 32767.0
                
                # Button event
                if event_type & 0x01 and number < len(self.buttons):
                    self.buttons[number] = value

            # Create Joy message with current state
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.header.frame_id = 'joystick'
            joy_msg.axes = self.axes
            joy_msg.buttons = self.buttons
            
            self.publisher.publish(joy_msg)
            
        except Exception as e:
            self.get_logger().error(f'Joystick read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()
    rclpy.spin(joystick_publisher)
    
    joystick_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
