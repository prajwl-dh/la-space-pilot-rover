import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class imagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'videoFrames', 30)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.publisher.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info('Publishing video frames')
        
def main(args=None):
    rclpy.init(args=args)
    image_publisher = imagePublisher()
    
    rclpy.spin(image_publisher)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
