import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nereo_interfaces.msg import CommandVelocity

class JoyToCmdVelNode(Node):
    def init(self):
        super().init('joy_to_cmdvel_node')
        
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
     
        self.cmd_vel_publisher = self.create_publisher(
            CommandVelocity,
            '/nereo_cmd_vel',
            10
        )
        
        self.get_logger().info('JoyToCmdVelNode started')

    def joy_callback(self, joy_msg):
        cmd_vel_msg = CommandVelocity()

        if len(joy_msg.axes) >= 4:
            cmd_vel_msg[0] = joy_msg.axes[0]
            cmd_vel_msg[1] = joy_msg.axes[1]
            cmd_vel_msg[2] = joy_msg.axes[3]
            cmd_vel_msg[3] = 0
            cmd_vel_msg[4] = 0
            cmd_vel_msg[5] = joy_msg.axes[2]
           
        else:
            self.get_logger().warning('Joy message does not have enough axes.')

        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Published: {cmd_vel_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVelNode('joy_to_cmdvel_node')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
