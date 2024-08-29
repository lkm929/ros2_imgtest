import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Subscriber(Node):
    def __init__(self,name):
        super().__init__(name)
        print("11")
        self.subscription = self.create_subscription(
            Image,
            'imgtopic',
            self.img_callback,
            10)
        print(22)
        self.subscription
        self.bridge = CvBridge()
        print(33)
        
    def img_callback(self,ros_img_msg):
        print("Received image")
        try:
            # Convert ROS Image message to OpenCV format
            cv_img = self.bridge.imgmsg_to_cv2(ros_img_msg, desired_encoding='bgr8')
            cv2.imshow("cv_img", cv_img)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')


       


def main(args=None):
    rclpy.init(args=args)
    node = Subscriber("getimg")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    #node.destroy_node()



if __name__ == '__main__':
    main()
