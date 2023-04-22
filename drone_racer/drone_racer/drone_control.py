import rclpy
from rclpy.node import Node
import numpy as np
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
        self.req.cmd = 'takeoff'
        self.future = self.tello_action_client.call_async(self.req)

        self.camera_width = 480
        self.camera_height = 360

        self.middle_x = int(self.camera_width/2)
        self.middle_y = int(self.camera_height/2)

        self.aspect_ratios = []

        self.go_through_counter = 0

        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.05
 
    def rect_callback(self, rect_msg):
        gate_size = np.maximum(rect_msg.w, rect_msg.h)
        aspect_ratio = rect_msg.w/rect_msg.h

        # At gate?
        if gate_size > 0.85 * self.camera_height and aspect_ratio > 0.9 and self.go_through_counter < 100:
            self.go_through_gate()
            self.go_through_counter += 1
            return
        else:
            self.go_through_counter = 0

        if gate_size < 0.4 * self.camera_height:
            self.approach_gate(rect_msg)

        elif aspect_ratio < 0.95:
            self.adjust_position_to_gate(rect_msg)

        self.vel_pub.publish(self.vel_msg)

    def get_slope(self, rect_msg):
        aspect_ratio = rect_msg.w/rect_msg.h
        self.aspect_ratios.append(aspect_ratio)
        if len(self.aspect_ratios) > 3:
            del self.aspect_ratios[0]
            x = np.arange(0, len(self.aspect_ratios))
            slope = np.polyfit(x, self.aspect_ratios, deg=1)[1]
            return slope
        else:
            return 0



    def approach_gate(self, rect_msg):
        self.vel_msg.linear.x = 0.05
        self.vel_msg.linear.y = 0.0
        self.adjust_aim(rect_msg)
        self.adjust_vertical(rect_msg)

    def adjust_position_to_gate(self, rect_msg):
        self.vel_msg.linear.x = 0.02
        if self.vel_msg.linear.y == 0.0:
            self.vel_msg.linear.y = 0.02

        if self.get_slope(rect_msg) > 0:
            self.vel_msg.linear.y = -self.vel_msg.linear.y


    def go_through_gate(self):
        self.vel_msg.angular.z = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.linear.x = 0.1
        self.vel_pub.publish(self.vel_msg)

    def adjust_aim(self, rect_msg):
        rect_mid_x = rect_msg.x + rect_msg.w/2
        if abs(self.middle_x - rect_mid_x) < 20:
            return
        self.vel_msg.angular.z = (self.middle_x - rect_mid_x)/5000

    def adjust_vertical(self, rect_msg):
        rect_mid_y = rect_msg.y + rect_msg.h/2
        if abs(self.middle_y - rect_mid_y) < 20:
            return
        self.vel_msg.linear.z = (self.middle_y - rect_mid_y)/3000

def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()