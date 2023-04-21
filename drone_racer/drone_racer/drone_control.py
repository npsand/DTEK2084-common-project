import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction
from racer_interfaces.msg import Rectangle


class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(Rectangle, '/drone1/gate_rectangle', self.rect_callback, qos_profile=qos_policy)
        self.vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)

        self.tello_action_client = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.tello_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for tello action service')
        self.req = TelloAction.Request()

        self.middle_x = int(960/2)
        self.middle_y = int(720/2)

        self.last_rect = Rectangle()
        self.last_rect.x = self.middle_x
        self.last_rect.y = self.middle_y
        self.last_rect.w = 0
        self.last_rect.h = 0
 
    def rect_callback(self, rect_msg):
        self.adjust_horizontal(rect_msg)

    def adjust_horizontal(self, rect_msg):
        vel_msg = Twist()
        rect_mid_x = rect_msg.x + rect_msg.w/2
        if abs(self.middle_x - rect_mid_x) < 20:
            return
        #self.get_logger().info('mmmm %d' %(self.middle_x - rect_mid_x))
        vel_msg.angular.z = (self.middle_x - rect_mid_x)/5000
        vel_msg.linear.x = 0.05
        self.get_logger().info('publish') 
        self.vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()