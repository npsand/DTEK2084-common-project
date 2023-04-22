import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction
from std_srvs.srv import Trigger
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
        self.tello_req = TelloAction.Request()
        self.tello_req.cmd = 'takeoff'
        self.future = self.tello_action_client.call_async(self.tello_req)

        self.init_stop_client = self.create_client(Trigger, '/drone1/init_stop')
        while not self.init_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for init stop service')
        self.stop_req = Trigger.Request()

        self.camera_width = 480
        self.camera_height = 360

        self.middle_x = int(self.camera_width/2)
        self.middle_y = int(self.camera_height/2)

        self.horizontal_sign = 1
        self.sign_counter = 0

        self.aspect_ratios = [0.5]
        self.aspect_ratio = 0.5

        self.go_through_counter = 0
        self.scan_counter = 1
        self.choose_gate_counter = 0
        
        self.largest_gate = 0

        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.05
 
    def rect_callback(self, rect_msg):
        gate_size = np.maximum(rect_msg.w, rect_msg.h)
        if rect_msg.h > 0:
            self.aspect_ratio = rect_msg.w/rect_msg.h
            self.aspect_ratios.append(self.aspect_ratio)
            if len(self.aspect_ratios) > 15:
                self.aspect_ratios = self.aspect_ratios[1:]

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
        if self.sign_counter == 30:
            if self.get_slope() < 0:
                self.horizontal_sign *= -1
            self.get_logger().info('set hor sign: %d' % self.horizontal_sign)
            self.sign_counter = 0
        else:
            self.sign_counter += 1

        self.adjust_aim(rect_msg)
        self.adjust_vertical(rect_msg)

        if gate_size > 0.8 * self.camera_height and self.aspect_ratio > 0.75:
            # Enter go through sequence
            self.go_through_counter += 1

        if gate_size < 0.6 * self.camera_height:
            self.approach_gate()
        # Adjust position until straight path through gate
        if self.aspect_ratio < 0.9:
            self.adjust_horizontal()
        else:
            self.approach_gate()
        
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
            self.vel_msg.linear.x = 0.05
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
            self.vel_msg.angular.z = 0.15
        elif self.scan_counter > 1 and self.scan_counter <= 80:
            self.vel_msg.angular.z = 0.15
        elif self.scan_counter > 80 and self.scan_counter <= 240:
            self.vel_msg.angular.z = -0.15
        elif self.scan_counter > 240:
            self.get_logger().info('end scan through sequence')
            self.scan_counter = 0
            if self.largest_gate < 20:
                # Enter stop sequence
                self.future = self.init_stop_client.call_async(self.stop_req)
                self.get_logger().info('init stop sequence')
                # Enter scan sequence for stop sign
                self.scan_counter = 1
            else:
                # Enter choose gate sequence
                self.choose_gate_counter +=1
            return
        
        self.scan_counter += 1

        # Save largest gate
        max = np.maximum(rect_msg.w, rect_msg.h)
        if max > self.largest_gate:
            self.largest_gate = max

        self.vel_pub.publish(self.vel_msg)

    def choose_gate_sequence(self, gate_size):
        if self.choose_gate_counter == 1:
            self.get_logger().info('init choose gate sequence')
            self.vel_msg.angular.z = 0.15
            self.vel_pub.publish(self.vel_msg)
        elif self.choose_gate_counter < 120 and gate_size > 2:
            if gate_size < self.largest_gate + 30 and gate_size > self.largest_gate - 30:
                self.choose_gate_counter = 0
                self.largest_gate = 0
                return
        elif self.choose_gate_counter >= 120:
            self.choose_gate_counter = 0
            self.largest_gate = 0
            return

        self.choose_gate_counter += 1
        

        
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
        self.vel_msg.linear.z = (self.middle_y - rect_mid_y)/1000

    def adjust_horizontal(self):
        self.vel_msg.linear.y = self.horizontal_sign * 0.25 * (1- self.aspect_ratio)


    def get_slope(self):
        x = np.arange(0, len(self.aspect_ratios))
        slope = np.polyfit(x, self.aspect_ratios, deg=1)[0]
        #slope = self.aspect_ratios[-1] - self.aspect_ratios[0]
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