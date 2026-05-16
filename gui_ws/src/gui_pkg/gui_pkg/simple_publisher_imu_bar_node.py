import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, FluidPressure
import math
import random
from . import PoliciesUtils

class SensorsRandomNode(Node):
    def __init__(self):
        super().__init__('sensors_random_node')
        
        # Publisher dell'IMU
        self.imu_publisher = self.create_publisher(Imu, 'imu_data', 10)
        
        # Publisher del Barometro
        self.baro_publisher = self.create_publisher(
            FluidPressure, 
            'barometer_pressure', 
            PoliciesUtils.sensor_qos
        )
        
        # Timer unico che gira a 10 Hz (~100ms) per aggiornare entrambi i sensori
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.count = 0.0

    def timer_callback(self):
        self.count += 0.05
        
        # ==========================================
        # 1. SIMULAZIONE DATI IMU
        # ==========================================
        imu_msg = Imu()
        
        # Generiamo oscillazioni sinusoidali per simulare il Roll/Pitch/Yaw del ROV
        roll = math.sin(self.count) * 0.2         # ~11 gradi
        pitch = math.cos(self.count * 0.8) * 0.15  # ~8 gradi
        yaw = (self.count * 0.2) % (2 * math.pi)
        
        # Conversione base Eulero -> Quaternione (Z-Y-X)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Pubblica l'assetto
        self.imu_publisher.publish(imu_msg)

        # ==========================================
        # 2. SIMULAZIONE DATI BAROMETRO (PROFONDITÀ)
        # ==========================================
        baro_msg = FluidPressure()
        
        # depth = (fluid_pressure - 101325) / 9806.65
        target_depth = 5.0 + math.sin(self.count * 0.5) * 1.5 # Varia tra 3.5m e 6.5m
        
        simulated_pressure = (target_depth * 9806.65) + 101325.0
        
        # Rumore bianco
        baro_msg.fluid_pressure = simulated_pressure + random.uniform(-50.0, 50.0)

        # Pubblica la pressione barometrica
        self.baro_publisher.publish(baro_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorsRandomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()