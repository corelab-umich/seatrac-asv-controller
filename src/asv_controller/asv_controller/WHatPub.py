import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2  # OpenCV library

class WHatPublisher(Node):
    def __init__(self):
        super().__init__('w_hat_pub')
        
        # Create a publisher for the image topic
        self.publisher_ = self.create_publisher(Image, 'w_hat', 10)
        
        # Set up the CvBridge
        self.bridge = CvBridge()
        
        # Define the path to your image
        self.image_path = '/root/images/w_hat.png'
        
        # Timer to periodically publish the image
        self.timer = self.create_timer(1.0, self.publish_image)  # Publish every second

    def publish_image(self):
        # Load the image using OpenCV
        cv_image = cv2.imread(self.image_path)
        
        # Check if the image was loaded successfully
        if cv_image is None:
            self.get_logger().error(f"Failed to load image from {self.image_path}")
            return
        
        # Convert the OpenCV image to a ROS Image message
        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        
        # Publish the image message
        self.publisher_.publish(image_message)
        self.get_logger().info('Image published')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = WHatPublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
