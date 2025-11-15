#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class SolturaGancho(Node):
    def __init__(self):
        super().__init__('soltura_gancho')
        self.subscription = self.create_subscription(Bool, 'centralizado', self.liberar, 10)
        self.gancho_liberado = False

        self.get_logger().info('Sistema de soltura de gancho inicializado.')

    def liberar(self, msg):
        if msg.data and not self.gancho_liberado:
            self.get_logger().info('Drone centralizado. Liberando gancho')
            self.gancho_liberado = True

def main(args=None):
    rclpy.init(args=args)
    node = SolturaGancho()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
