from std_srvs.srv import SetBool
from rclpy.node import Node
import rclpy
from time import sleep


class ROVArmDisarmServiceClient(Node):
    def __init__(self):
        self.max_try = 3
        self.service_available = False
        super().__init__('rov_arm_disarm_service_client')
        self.cli = self.create_client(SetBool, '/set_rov_arm_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0) and self.max_try > 0:
            self.max_try -= 1
            self.get_logger().info('service not available, waiting again...')
        
        if self.max_try == 0:
            self.get_logger().error('Service not available')
            self.service_available = False
            return

        self.service_available = True
        print('Service available')

    def call_service(self, arm_status: bool):
        if self.service_available is False:
            self.get_logger().error('Service not available')
            return
        
        request = SetBool.Request()
        request.data = arm_status
        
        future = self.cli.call_async(request)
        
        #rclpy.spin_until_future_complete(self, future)
        #sleep(1)
        
        if future is not None:
            self.get_logger().warn(f"Value of future.done() = {future.done()}")
            self.get_logger().info(f'Successfully called service: arm_status={arm_status}')
        else:
            self.get_logger().error('Service call failed')

    def get_current_value(self):
        """
        Function to get the current value of the arm status without changing it or calling the service
        """
        if self.service_available is False:
            self.get_logger().error('Service not available')
            return
        
        request = SetBool.Request()
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Current value of arm status: {future.result().data}')
            return future.result().data
        else:
            self.get_logger().error('Service call failed')
            return None

