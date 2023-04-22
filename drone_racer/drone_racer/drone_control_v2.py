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

        self.horizontal_sign = 1
        self.sign_counter = 0

        self.aspect_ratios = [0.5]
        self.aspect_ratio = 0.5

        self.go_through_counter = 0
        self.scan_counter = 0
        self.choose_gate_counter = 0
        
        self.largest_rect = 0

        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.05
 
    def rect_callback(self, rect_msg):
        gate_size = np.maximum(rect_msg.w, rect_msg.h)
        if rect_msg.h > 0:
            self.aspect_ratio = rect_msg.w/rect_msg.h
            self.aspect_ratios.append(self.aspect_ratio)
            if len(self.aspect_ratios) > 20:
                del self.aspect_ratios[0]

        # Stay in go through sequence
        if self.go_through_counter > 0:
            self.go_through_sequence()
            return

        # Stay in scan sequence
        if self.scan_counter > 0:
            self.scan_sequence(rect_msg)
            return
        
        # Stay in choose gate sequence
        if self.choose_gate_counter > 0:
            self.choose_gate_sequence(gate_size)
            return
        
        # Set right direction for horizontal movement
        if self.sign_counter == 50:
            if self.get_slope() < 0:
                self.horizontal_sign *= -1
            self.get_logger().info('set hor sign: %d' % self.horizontal_sign)
            self.sign_counter = 0
        else:
            self.sign_counter += 1

        self.adjust_aim(rect_msg)
        self.adjust_vertical(rect_msg)

        if gate_size < 0.6 * self.camera_height:
            self.approach_gate()
        
        # Adjust position until straight path through gate
        if self.aspect_ratio < 0.95:
            self.adjust_horizontal()
        elif gate_size > 0.8 * self.camera_height:
            # Enter go through sequence
            self.go_through_counter += 1
        
        # Prevent turning too fast
        if abs(self.vel_msg.angular.z) > 0.2:
            self.vel_msg.angular.z = 0.2

        self.vel_pub.publish(self.vel_msg)

    # Go through gate
    def go_through_sequence(self):
        if self.go_through_counter == 1:
            self.get_logger().info('init got through sequence')
            self.vel_msg.angular.z = 0.0
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = 0.0
            self.vel_msg.linear.x = 0.1
        elif self.go_through_counter > 1 and self.go_through_counter <= 60:
            self.vel_msg.linear.x = 0.1
        elif self.go_through_counter > 60:
            self.get_logger().info('end go through through sequence')
            self.go_through_counter = 0
            # Enter scan sequence
            self.scan_counter += 1
            return
        self.go_through_counter += 1

        self.vel_pub.publish(self.vel_msg)

    # Scan for the next gate
    def scan_sequence(self, rect_msg):
        if self.scan_counter == 1:
            self.get_logger().info('init scan sequence')
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = 0.0
            self.vel_msg.angular.z = 0.1
        elif self.scan_counter > 1 and self.scan_counter <= 80:
            self.vel_msg.angular.z = 0.1
        elif self.scan_counter > 80 and self.scan_counter <= 240:
            self.vel_msg.angular.z = -0.1
        elif self.scan_counter > 240:
            self.get_logger().info('end scan through sequence')
            self.scan_counter = 0
            # Enter choose gate sequence
            self.choose_gate_counter +=1
            return
        
        self.scan_counter += 1

        max = np.maximum(rect_msg.w, rect_msg.h)
        if max > self.largest_rect:
            self.largest_rect = max

        self.vel_pub.publish(self.vel_msg)

    def choose_gate_sequence(self, gate_size):
        if self.choose_gate_counter == 1:
            self.get_logger().info('init choose gate sequence')
            self.vel_msg.angular.z = 0.1
        if self.choose_gate_counter < 160:
            if gate_size < self.largest_rect + 30 or gate_size > self.largest_rect - 30:
                self.choose_gate_counter = 0

        
    def approach_gate(self):
        self.vel_msg.linear.x = 0.05
        self.vel_msg.linear.y = 0.0

    # Aim at gate center
    def adjust_aim(self, rect_msg):
        rect_mid_x = rect_msg.x + rect_msg.w/2
        if abs(self.middle_x - rect_mid_x) < 20:
            return
        # Angular velocity proportional to how far gate middle is from middle of the screen
        self.vel_msg.angular.z = (self.middle_x - rect_mid_x)/1000


    def adjust_vertical(self, rect_msg):
        rect_mid_y = rect_msg.y + rect_msg.h/2
        if abs(self.middle_y - rect_mid_y) < 20:
            return
        # Upwards velocity proportional to how far gate middle is from middle of the screen
        self.vel_msg.linear.z = (self.middle_y - rect_mid_y)/2000

    def adjust_horizontal(self):
        self.vel_msg.linear.y = self.horizontal_sign * 0.05


    def get_slope(self):
        #x = np.arange(0, len(self.aspect_ratios))
        #slope = np.polyfit(x, self.aspect_ratios, deg=1)[1]
        slope = self.aspect_ratios[-1] - self.aspect_ratios[0]
        self.get_logger().info('slope: %g' % slope)
        return slope



def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()