import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nereo_interfaces.msg import CommandVelocity
import time

from .ControllerClass import Controller

MAX_STEPS = 10
COOLDOWN_TIME = 0.1  # seconds
DEADZONE = 0.04

class JoyToCmdVelNode(Node):
    def __init__(self) -> None:
        super().__init__('joy_to_cmd_vel')

        self.controller = Controller()

        self.pitch = 0.0
        self.roll = 0.0

        self.deadzone = DEADZONE

        # Add cooldown timers
        self.last_pitch_update = 0.0
        self.last_roll_update = 0.0
        
        self.cmd_vel_publisher = self.create_publisher(
            CommandVelocity,
            '/nereo_cmd_vel',
            10
        )
        
        self.timer = self.create_timer(1/20, self.publish_cmd_vel)
        
        self.get_logger().info('JoyToCmdVelNode started')


    def update_pitch_roll(self, pad: list[int]) -> None:
        current_time = time.time()
        
        # Update pitch with cooldown
        if current_time - self.last_pitch_update >= COOLDOWN_TIME:
            if pad[3] != 0 and self.pitch < 1.0 - self.deadzone:
                self.pitch += pad[3]/MAX_STEPS
                self.last_pitch_update = current_time
            if pad[2] != 0 and self.pitch > -1.0 + self.deadzone:
                self.pitch -= pad[2]/MAX_STEPS
                self.last_pitch_update = current_time
                
        # Update roll with cooldown
        if current_time - self.last_roll_update >= COOLDOWN_TIME:
            if pad[1] != 0 and self.roll < 1.0 - self.deadzone:
                self.roll += pad[1]/MAX_STEPS
                self.last_roll_update = current_time
            if pad[0] != 0 and self.roll > -1.0 + self.deadzone:
                self.roll -= pad[0]/MAX_STEPS
                self.last_roll_update = current_time
            
        self.pitch = self.handle_deadzone(self.pitch)
        self.roll = self.handle_deadzone(self.roll)

    def get_joystick_data(self) -> dict[str, list[float]]:
        data = self.controller.read()
        self.get_logger().info(f'Joystick data: {data}')
        self.update_pitch_roll(data['pad'])
        self.get_logger().info(f'Pitch: {self.pitch}, Roll: {self.roll}')
        return data
        
    def publish_cmd_vel(self) -> None:
        data = self.get_joystick_data()
        cmd_vel_msg = CommandVelocity()
        cmd_vel_msg.cmd_vel[0] = - self.handle_deadzone(data['leftJoy'][1] / 32768)
        cmd_vel_msg.cmd_vel[1] = self.handle_deadzone(data['leftJoy'][0] / 32768)
        cmd_vel_msg.cmd_vel[2] = self.handle_deadzone(data['rightJoy'][1] / 32768)
        cmd_vel_msg.cmd_vel[3] = self.roll
        cmd_vel_msg.cmd_vel[4] = self.pitch
        cmd_vel_msg.cmd_vel[5] = self.handle_deadzone(data['rightJoy'][0] / 32768)
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        #self.get_logger().info(f'Published: {cmd_vel_msg}')

    def handle_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVelNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
