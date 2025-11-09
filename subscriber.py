import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class DroneColorFollower(Node):
    def __init__(self):
        super().__init__('drone_color_follower')
        self.subscription = self.create_subscription(
            Image,
            'camera/imagem',
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Twist
  '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Drone Color Follower...')

 
   def image_callback(self, msg):
        OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 50, 150])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist ()

       if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])  # centro x do objeto
                cy = int(M['m01'] / M['m00'])  # centro y do objeto
                img_cx = frame.shape[1] / 2
                img_cy = frame.shape[0] / 2
                error_x = cx - img_cx
                twist.angular.z = -0.002 * error_x  
                error_y = cy - img_cy
                twist.linear.z = -0.002 * error_y  
                twist.linear.x = 0.2
                self.get_logger().info(
                    f'Alvo detectado | erro_x={error_x:.1f} erro_y={error_y:.1f}'

                )
        else:
            twist.linear.x = 0.0
            twist.linear.z = 0.0
            twist..angular.z = 0.0
            self.get_logger().warn('Nenhum alvo detectado — parado.')
            self.publisher_.publish(twist)
            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DroneColorFollower()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
