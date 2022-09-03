import rclpy
from rclpy.node import Node

import lgpio
import time

from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped

# Configuration
R_FWD = 18
R_REV = 12
L_FWD = 13
L_REV = 19
FREQ = 100
 
h = lgpio.gpiochip_open(0)

class Robot(Node):

    def __init__(self):
        super().__init__('robot_node')
        self.get_logger().info('Stating robot node...')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cb,
            10)
        self.subscription  # prevent unused variable warning

        self.left_wheel_name = self.declare_parameter("left_wheel_name", "Left")
        self.right_wheel_name = self.declare_parameter("right_wheel_name", "Right")
        self.joystick_name = self.declare_parameter("joystick_frame", "Robot")
        self.track_width = self.declare_parameter("track_width", 0.152)
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.062)

        # GPIO inits
        lgpio.tx_pwm(h, R_FWD, FREQ, 100)
        lgpio.tx_pwm(h, R_REV, FREQ, 100)
        lgpio.tx_pwm(h, L_FWD, FREQ, 100)
        lgpio.tx_pwm(h, L_REV, FREQ, 100)
        time.sleep(1)

    def write_left_value(self, val):
        if val >= 0: # fdw
            pin = L_FWD
        else:
            pin = L_REV
        write_val =  (1.0 - abs(val)) * 100
        lgpio.tx_pwm(h, pin, FREQ, write_val)
        self.get_logger().info(f'[Left] Writing value {write_val} to pin {pin}')
    
    def write_right_value(self, val):
        if val >= 0: # fdw
            pin = R_FWD
        else:
            pin = R_REV
        write_val =  (1.0 - abs(val)) * 100
        lgpio.tx_pwm(h, pin, FREQ, write_val)
        self.get_logger().info(f'Writing value {write_val} to pin {pin}')
    
    def cb(self, msg):
      # self.get_logger().info(f'Received msg {msg}')
      if (msg.header.frame_id == self.joystick_frame):
          # ref: https://www.roboticsbook.org/S52_diffdrive_actions.html
          v = msg.twist.linear.x
          w = msg.twist.angular.z
          linear_term = v / self.wheel_radius
          angular_term = (self.track_width * w) / (2 * self.wheel_radius)
          max_value = (1.0 / self.wheel_radius) + (self.track_width / (2.0 * self.wheel_radius))
          # We divide by max_value to normalize to [0, 1]
          left_val = linear_term - angular_term / max_value
          right_val = linear_term + angular_term / max_value
          self.write_left_value(left_val)
          self.write_right_value(right_val)
          time.sleep(0.1)
          return

      if (msg.header.frame_id == self.left_wheel_name):
          self.write_left_value(msg.twist.linear.x)
      if (msg.header.frame_id == self.right_wheel_name):
          self.write_right_value(msg.twist.linear.x)
      time.sleep(0.1)

    def __del__(self):
        lgpio.tx_pwm(h, R_FWD, FREQ, 100)
        lgpio.tx_pwm(h, R_REV, FREQ, 100)
        lgpio.tx_pwm(h, L_FWD, FREQ, 100)
        lgpio.tx_pwm(h, L_REV, FREQ, 100)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)

    robot = Robot()

    rclpy.spin(robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    lgpio.gpiochip_close(h)
