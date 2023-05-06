import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction
from std_srvs.srv import Trigger
from racer_interfaces.msg import Rectangle
import time


class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(Rectangle, '/drone1/gate_rectangle', self.rect_callback, qos_profile=qos_policy)
        self.vel_pub = self.create_publisher(Twist, '/control', 10) #/control

        # self.tello_action_client = self.create_client(TelloAction, '/drone1/tello_action')
        # while not self.tello_action_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('waiting for tello action service')
        # self.tello_req = TelloAction.Request()
        # self.tello_req.cmd = 'takeoff'
        # self.future = self.tello_action_client.call_async(self.tello_req)
        time.sleep(3)

        self.init_stop_client = self.create_client(Trigger, '/drone1/init_stop')
        while not self.init_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for init stop service')
        self.stop_req = Trigger.Request()

        self.camera_width = 960
        self.camera_height = 720

        self.middle_x = int(self.camera_width/2)
        self.middle_y = int(self.camera_height/2 - 100)

        self.base_speed = 120
        self.sequence_length = 60 # int(10 * 1/(self.base_speed))

        self.stop = False
        self.state = 'approach'
        self.sequence_counter = 0

        self.horizontal_sign = 1
        self.sign_counter = 0

        self.aspect_ratios = [0.5]
        self.aspect_ratio = 0.5
        self.gate_size = 1
        
        self.largest_gate = 0

        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.05
 
    def rect_callback(self, rect_msg):

        self.gate_size = np.maximum(rect_msg.w, rect_msg.h)

        self.save_aspect_ratio(rect_msg)

        if self.state == 'stop_state':
            self.stop_sequence()
        elif self.state == 'find_stop':
            self.find_stop()
        elif self.state == 'scan':
            self.scan_sequence(rect_msg)
        elif self.state == 'choose_gate':
            self.choose_gate_sequence()
        elif self.state == 'go_through':
            self.go_through_sequence()
        elif self.state == 'approach':
            self.approach(rect_msg)
        elif self.state == 'approach_slowly':
            self.approach_slowly(rect_msg)
        elif self.state == 'adjust_position_to_gate':
            self.adjust_horizontal(rect_msg)
            self.adjust_vertical(rect_msg)
            self.adjust_aim(rect_msg)
            self.move_forward_very_slowly()
        elif self.state == 'move_backwards':
            self.move_backwards()

        self.get_logger().info('state: %s' % self.state)

        if self.sequence_counter > 0:      
            temp_y = self.vel_msg.linear.y
            self.vel_msg.linear.y = self.vel_msg.linear.x
            self.vel_msg.linear.x = -temp_y

            self.vel_pub.publish(self.vel_msg)
            return

        self.choose_action()
        self.check_strafe_dir()

        temp_y = self.vel_msg.linear.y
        self.vel_msg.linear.y = self.vel_msg.linear.x
        self.vel_msg.linear.x = -temp_y
        self.vel_pub.publish(self.vel_msg)


    def choose_action(self):
        
        if self.gate_size < 0.1 * self.camera_height:
            pass
            #    self.state = 'move_backwards'
        elif self.gate_size < 0.4 * self.camera_height:
            self.state = 'approach'
       # elif self.gate_size < 0.85 * self.camera_height and self.aspect_ratio >= 0.85:
            #self.state = 'approach_slowly'
        #elif self.gate_size < 0.85 * self.camera_height and self.aspect_ratio < 0.85:
         #   self.state = 'adjust_position_to_gate'
        elif self.gate_size >= 0.85 * self.camera_height:
            if self.stop:
                self.state = 'stop_state'
                self.sequence_counter = 1
            else:
                self.state = 'go_through'


    # Go through gate
    def go_through_sequence(self):
        if self.sequence_counter == 1:
            self.get_logger().info('Init go through sequence')
            self.set_movement_zero()
            self.move_forward()
        elif self.sequence_counter <= 2 * self.sequence_length:
            self.move_forward()
        elif self.sequence_counter > 2 * self.sequence_length:
            # Enter scan sequence
            self.sequence_counter = 0
            self.state = 'scan'
        
        self.sequence_counter += 1

    # Scan for the next gate
    def scan_sequence(self, rect_msg):

        # Save largest gate
        max = np.maximum(rect_msg.w, rect_msg.h)
        if max > self.largest_gate:
            self.largest_gate = max

        if self.sequence_counter == 1:
            self.get_logger().info('Init scan sequence')
            self.set_movement_zero()
            self.aim_left()
        elif self.sequence_counter <= 3 * self.sequence_length:
            self.aim_left()
        elif self.sequence_counter <= 9 * self.sequence_length:
            self.aim_right()
        elif self.sequence_counter > 9 * self.sequence_length:
            # Enter next sequence
            self.sequence_counter = 0
            if self.largest_gate < 20:
                # Trigger finding stop sign
                self.future = self.init_stop_client.call_async(self.stop_req)
                self.get_logger().info('Trigger finding stop sign')
                self.state = 'find_stop'
                self.stop = True
                self.base_speed *= 0.7
            else:
                # Enter choose gate sequence
                self.state = 'choose_gate'
        
        self.sequence_counter += 1


    def choose_gate_sequence(self):
        if self.sequence_counter == 1:
            self.get_logger().info('Init choose gate sequence')
            self.aim_left()
        elif self.sequence_counter <= 6 * self.sequence_length and self.gate_size > 2:
            # Find the largest gate
            if self.gate_size < self.largest_gate + 30 and self.gate_size > self.largest_gate - 30:
                self.get_logger().info('Largest gate found')
                self.sequence_counter = 0
                self.largest_gate = 0
                return
        # Enter scan sequence if something goes wrong in choosing the gate
        elif self.sequence_counter >= 6 * self.sequence_length:
            self.get_logger().info('Did not find largest gate')
            self.state = 'scan'
            self.sequence_counter = 0

        self.sequence_counter += 1

    def find_stop(self):
        if self.sequence_counter <= 3 * self.sequence_length:
            self.aim_left()
        elif self.sequence_counter > 3 * self.sequence_length:
            self.sequence_counter = 0
            self.state = 'scan'

        self.sequence_counter += 1 

    def stop_sequence(self):
        if self.sequence_counter < 10:
            self.set_movement_zero()
            self.move_backwards()
        else:
            self.tello_req.cmd = 'land'
            self.future = self.tello_action_client.call_async(self.tello_req)
            time.sleep(3)
        self.sequence_counter += 1


    def approach(self, rect_msg):
        self.set_movement_zero()
        self.move_forward()
        self.adjust_aim(rect_msg)
        self.adjust_vertical(rect_msg)
        self.adjust_horizontal(rect_msg)

    def approach_slowly(self, rect_msg):
        self.set_movement_zero()
        self.move_forward_slowly()
        self.adjust_aim(rect_msg)
        self.adjust_vertical(rect_msg)
        self.adjust_horizontal(rect_msg)


    # Aim at gate center
    def adjust_aim(self, rect_msg):
        rect_mid_x = rect_msg.x + rect_msg.w/2
        # Angular velocity proportional to how far gate middle is from middle of the screen
        self.vel_msg.angular.z = self.base_speed * (self.middle_x - rect_mid_x)/(self.camera_height * 2)


    def adjust_vertical(self, rect_msg):
        rect_mid_y = rect_msg.y + rect_msg.h/2
        # Upwards velocity proportional to how far gate middle is from middle of the screen
        self.vel_msg.linear.z = self.base_speed * (self.middle_y - rect_mid_y)/(self.camera_height)

    def adjust_horizontal(self, rect_msg):
        self.vel_msg.linear.y = self.base_speed * self.horizontal_sign * 0.1 * (1.05 - self.aspect_ratio)


    
    def move_forward(self):
        self.vel_msg.linear.x = 0.24 * self.base_speed

    def move_forward_slowly(self):
        self.vel_msg.linear.x = 0.12 * self.base_speed

    def move_forward_very_slowly(self):
        self.vel_msg.linear.x = 0.06 * self.base_speed

    def move_backwards(self):
        self.set_movement_zero()
        self.vel_msg.linear.x = -0.05 * self.base_speed

    def aim_left(self):
        self.vel_msg.angular.z = 0.12 * self.base_speed

    def aim_right(self):
        self.vel_msg.angular.z = -0.12 * self.base_speed

    def set_movement_zero(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.z = 0.0

    def save_aspect_ratio(self, rect_msg):
        if rect_msg.h > 0:
            self.aspect_ratio = rect_msg.w/rect_msg.h
            self.aspect_ratios.append(self.aspect_ratio)
            if len(self.aspect_ratios) > int(self.sequence_length / 3):
                self.aspect_ratios = self.aspect_ratios[1:]

    # Returns the trend (slope) of gate aspect ratios 
    def get_slope(self):
        x = np.arange(0, len(self.aspect_ratios))
        slope = np.polyfit(x, self.aspect_ratios, deg=1)[0]
        return slope
    
    # Set right direction for horizontal movement
    def check_strafe_dir(self):
        if self.sign_counter == self.sequence_length:
            if self.get_slope() < 0:
                self.horizontal_sign *= -1
            self.sign_counter = 0
        else:
            self.sign_counter += 1



def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()