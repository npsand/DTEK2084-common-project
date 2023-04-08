import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from tello_msgs.srv import TelloAction
from .get_gate import get_closest_gate


class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(Image, '/drone1/image_raw', self.image_callback, qos_profile=qos_policy)
        self.publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 10)

        self.tello_action_client = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.tello_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for tello action service')
        self.req = TelloAction.Request()

        self.bridge = CvBridge()

        self.raw_image = None


        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def image_callback(self, msg):
        self.raw_image = msg

    def timer_callback(self):
        if self.raw_image is None:
            self.get_logger().info('no image')   
        else:
            img = self.bridge.imgmsg_to_cv2(self.raw_image, 'rgb8')
            x, y, w, h = get_closest_gate(img)
            #self.get_logger().info('x: %d, y: %d, w: %d, h: %d' % (x, y, w, h))
            img = cv2.rectangle(img, (x, y),(x + w, y + h), color=(255, 0, 0), thickness=3)
            cv2.imshow('Monitor', img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()