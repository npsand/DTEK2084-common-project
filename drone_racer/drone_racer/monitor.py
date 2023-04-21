import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from racer_interfaces.msg import Rectangle


class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.image_sub = self.create_subscription(Image, '/drone1/image_raw', self.image_callback, qos_profile=qos_policy)
        self.raw_image = None

        self.rect_sub  = self.create_subscription(Rectangle, '/drone1/gate_rectangle', self.rect_callback, qos_profile=qos_policy)
        self.rect = None

        self.bridge = CvBridge()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def image_callback(self, msg):
        self.raw_image = msg

    def rect_callback(self, msg):
        self.rect = msg

    def timer_callback(self):
        if self.raw_image is None:
            self.get_logger().info('no image')   
        else:
            img = self.bridge.imgmsg_to_cv2(self.raw_image, 'rgb8')
            print(f'image shape: {img.shape}')
            if self.rect is not None:
                x = self.rect.x
                y = self.rect.y
                w = self.rect.w
                h = self.rect.h
                #self.get_logger().info('x: %d, y: %d, w: %d, h: %d' % (x, y, w, h))
                img = cv2.rectangle(img, (x, y),(x + w, y + h), color=(255, 0, 0), thickness=3)
            cv2.imshow('Monitor', img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    monitor = Monitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()