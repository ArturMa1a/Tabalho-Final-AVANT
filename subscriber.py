import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class cameraSubscriber(Node):
    def __init__(self):
        super().__init__('drone_color_follower')
        self.subscription = self.create_subscription(
            Image,
            'camera/imagem',
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        self.centralizado = self.create_publisher(
            Bool,
            'drone/centralizado',
            10
        )

        self.bridge = CvBridge()
        self.get_logger().info('Drone iniciado.')

        self.yawIncr = 0.002
        self.zIncr = 0.002
        self.velocidade = 0.20
        self.min_contour_area = 200
        self.paradaVermelho = 3000  
        self.toleranciaCentro = 10
        self.xVelMax = 1.0
        self.zVelMax = 0.5
        self.zVelAngMax = 1.0

 
    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 50, 150])
        upper_blue = np.array([130, 255, 255])
        lower_red =  np.array([170, 0, 0])
        upper_red =  np.array([255, 70, 70])
        mascaraAzul = cv2.inRange(hsv, lower_blue, upper_blue)
        mascaraVermelha = cv2.inRange(hsv, lower_red, upper_red)

        kernel = np.ones((5, 5), np.uint8)
        mascaraAzul = cv2.morphologyEx(mascaraAzul, cv2.MORPH_OPEN, kernel)
        mascaraAzul = cv2.morphologyEx(mascaraAzul, cv2.MORPH_CLOSE, kernel)
        mascaraVermelha = cv2.morphologyEx(mascaraVermelha, cv2.MORPH_OPEN, kernel)
        mascaraVermelha = cv2.morphologyEx(mascaraVermelha, cv2.MORPH_CLOSE, kernel)

        contornoAzul, _ = cv2.findContours(mascaraAzul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contornoVermelho, _ = cv2.findContours(mascaraVermelha, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        centralizado_msg = Bool()
        centralizado_msg.data = False

        if contornoVermelho:
            largest2 = max(contornoVermelho, key=cv2.contourArea)
            area_r = cv2.contourArea(largest2)
            if area_r >= self.min_contour_area:
                M = cv2.moments(largest2)
                if M.get('m00', 0) != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    img_cx = frame.shape[1] / 2
                    img_cy = frame.shape[0] / 2
                    dx = cx - img_cx
                    dz = cy - img_cy

                    if area_r >= self.paradaVermelho and abs(dx) < self.toleranciaCentro and abs(dz) < self.toleranciaCentro:
                        twist.linear.x = 0.0
                        twist.linear.z = -self.zIncr * dz
                        twist.angular.z = -self.yawIncr * dx
                        self.get_logger().info('Mangueira alcançada. Centralizando')

                        centralizado_msg.data = True
                    else:
                        twist.linear.x = min(self.velocidade, self.xVelMax)
                        twist.linear.z = - self.zIncr * dz
                        twist.angular.z = - self.yawIncr * dx
                        self.get_logger().info('Mangueira detectada')
                else:
                    twist.linear.x = 0.0
                    twist.linear.z = 0.0
                    twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.linear.z = 0.0
                twist.angular.z = 0.0

        elif contornoAzul:
            largest1 = max(contornoAzul, key=cv2.contourArea)
            area_b = cv2.contourArea(largest1)
            if area_b >= self.min_contour_area:
                M = cv2.moments(largest1)
                if M.get('m00', 0) != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    img_cx = frame.shape[1] / 2
                    img_cy = frame.shape[0] / 2
                    dx = cx - img_cx
                    dz = cy - img_cy

                    twist.angular.z = -self.yawIncr * dx
                    twist.linear.z = -self.zIncr * dz
                    twist.linear.x = self.velocidade
                    self.get_logger().info('Seguindo linha azul')
                else:
                    twist.linear.x = 0.0
                    twist.linear.z = 0.0
                    twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.linear.z = 0.0
                twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn('Nenhum alvo detectado')

        twist.linear.x = float(np.clip(twist.linear.x, -self.xVelMax, self.xVelMax))
        twist.linear.z = float(np.clip(twist.linear.z, -self.zVelMax, self.zVelMax))
        twist.angular.z = float(np.clip(twist.angular.z, -self.zVelAngMax, self.zVelAngMax))

        self.publisher_.publish(twist)
        self.centralizado.publish(centralizado_msg)

def main(args=None):
    rclpy.init(args=args)
    node = cameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
