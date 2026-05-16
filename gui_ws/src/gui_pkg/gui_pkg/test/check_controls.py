#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, FluidPressure, Joy
from std_srvs.srv import SetBool
import math
import random

class SimulatoreGlobaleNereo(Node):
    def __init__(self):
        super().__init__('simulatore_globale_nereo')
        
        # 1. PUBLISHER TELEMETRIA E JOYSTICK
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.baro_pub = self.create_publisher(FluidPressure, 'barometer_pressure', 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        
        # 2. SERVER DI ARM/DISARM
        self.arm_srv = self.create_service(SetBool, '/set_rov_arm_mode', self.handle_arm_request)
        
        # Stato locale simulato del ROV
        self.is_armed = False
        self.count = 0.0
        
        # Timer a 10 Hz per telemetria e Heartbeat Joystick
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("=== SIMULATORE GLOBALE NEREO ATTIVO ===")
        self.get_logger().info("In ascolto sul servizio /set_rov_arm_mode...")
        self.get_logger().info("Pubblicazione dati IMU, Profondità e Joystick (Heartbeat) attiva.")

    def handle_arm_request(self, request, response):
        """Risponde istantaneamente alle richieste di ARM/DISARM della GUI"""
        self.is_armed = request.data
        stato_str = "ARMED" if self.is_armed else "DISARMED"
        
        self.get_logger().info(f"[Ricevuto Comando] Richiesta cambio stato ROV a: {stato_str}")
        
        response.success = True
        response.message = f"ROV passato con successo a stato {stato_str}"
        return response

    def timer_callback(self):
        self.count += 0.05
        
        # A. Pubblica Heartbeat Joystick per tenere la spia della GUI VERDE
        joy_msg = Joy()
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        self.joy_pub.publish(joy_msg)
        
        # B. Pubblica IMU (Orizzonte artificiale)
        imu_msg = Imu()
        roll = math.sin(self.count) * 0.15
        pitch = math.cos(self.count * 0.7) * 0.1
        yaw = (self.count * 0.1) % (2 * math.pi)
        
        cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
        cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
        cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
        
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
        self.imu_pub.publish(imu_msg)

        # C. Pubblica Barometro
        baro_msg = FluidPressure()
        depth = 3.5 + math.sin(self.count * 0.3) * 0.8
        baro_msg.fluid_pressure = (depth * 9806.65) + 101325.0 + random.uniform(-20.0, 20.0)
        self.baro_pub.publish(baro_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatoreGlobaleNereo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()