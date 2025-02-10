import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# 이미지 메시지 데이터 -> Array 형태로 변환
bridge = CvBridge()

class ImageExample(Node):
    
    def __init__(self):
        super().__init__('image_example')
        self.img_subscription = self.create_subscription(
                    Image,
                    '/intel_realsense_r200_depth/image_raw',
                    self.listener_callback,
                    10
                )
        self.img_subscription # prevent unused variable warning

        self.img_publisher = self.create_publisher(
            Image,
            '/binarized_img',
            10
        )

    def listener_callback(self, msg):
        self.img = bridge.imgmsg_to_cv2(msg, msg.encoding)
        gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, binarized_img = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY)
            
        new_msg = bridge.cv2_to_imgmsg(binarized_img)    

        self.img_publisher.publish(new_msg)
        self.get_logger().info("Publishing processed image")

def main(args=None):
    rclpy.init(args=args)

    image_example = ImageExample()

    rclpy.spin(image_example)
    image_example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
