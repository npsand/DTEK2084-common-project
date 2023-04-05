import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from tello_msgs.srv import TelloAction
from .get_gate import get_closest_gate


class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile=qos_policy)
        self.publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', qos_profile=qos_policy)

        self.path_client = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for next waypoint service')
        self.req = TelloAction.Request()

        self.waiting_for_waypoint = True




        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def image_callback(self, msg): 

        img = msg.data
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        x, y, w, h = get_closest_gate(img)

        self.get_logger().info('x: %d, y: %d, w: %d, h: %d' % (x, y, w, h))


def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()