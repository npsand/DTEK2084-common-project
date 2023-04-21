import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from racer_interfaces.msg import Rectangle
from .get_gate import get_closest_gate


class GateFinder(Node):
    def __init__(self):
        super().__init__('gate_finder')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(Image, '/drone1/image_raw', self.image_callback, qos_profile=qos_policy)
        self.publisher_ = self.create_publisher(Rectangle, '/drone1/gate_rectangle', 10)

        self.bridge = CvBridge()

        self.raw_image = None

        timer_period = 0.03  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def image_callback(self, msg):
        self.raw_image = msg

    def timer_callback(self):
        if self.raw_image is None:
            self.get_logger().info('no image')   
        else:
            img = self.bridge.imgmsg_to_cv2(self.raw_image, 'rgb8')
            x, y, w, h = get_closest_gate(img)
            rect_msg = Rectangle()
            rect_msg.x = x
            rect_msg.y = y
            rect_msg.w = w
            rect_msg.h = h
            self.publisher_.publish(rect_msg)

def main(args=None):
    rclpy.init(args=args)
    gate_finder = GateFinder()
    rclpy.spin(gate_finder)
    gate_finder.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()