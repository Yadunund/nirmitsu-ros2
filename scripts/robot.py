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
        self.joystick_name = self.declare_parameter("joystick_name", "Joystick")

        # GPIO inits
        lgpio.tx_pwm(h, R_FWD, FREQ, 100)
        lgpio.tx_pwm(h, R_REV, FREQ, 100)
        lgpio.tx_pwm(h, L_FWD, FREQ, 100)
        lgpio.tx_pwm(h, L_REV, FREQ, 100)
        time.sleep(1)

    def cb(self, msg):
      # self.get_logger().info(f'Received msg {msg}')
      if (msg.header.frame_id == "Left"):
          val = msg.twist.linear.x
          if val >= 0: # fdw
              pin = L_FWD
          else:
              pin = L_REV
          write_val =  (1.0 - abs(val)) * 100
          lgpio.tx_pwm(h, pin, FREQ, write_val)
          self.get_logger().info(f'Writing value {write_val} to pin {pin}')

      if (msg.header.frame_id == "Right"):
          val = msg.twist.linear.x
          if val >= 0: # fdw
              pin = R_FWD
          else:
              pin = R_REV
          write_val =  (1.0 - abs(val)) * 100
          lgpio.tx_pwm(h, pin, FREQ, write_val)
          self.get_logger().info(f'Writing value {write_val} to pin {pin}')
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
