import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select
import time
import atexit

# Teleoperation script that takes key presses as input, and converts them to /cmd_vel commands.
##
# Maximum velocities and desired acceleration values are specified as parameters (for now, simply change the values in this file)
# When releasing a key, the robot decelerates using the specified acceleration value.
# In order to stop suddenly, press "k".
# Velocity limits can also be adjusted at runtime. Acceleration value is fixed at runtime.
# This node was written to generate velocity commands with a tunable acceleration slope.
# This allows us to give smooth acceleration and deceleration commands to the RobAIR robot, even when the low-level MD49 motor driver acceleration limit is set to the maximum value.


class SmoothTeleopNode(Node):
    def __init__(self):
        super().__init__('smooth_teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Parameters
        self.declare_parameter('max_linear_vel', 0.5)  # Robair MD49 motor max = 0.8m/s
        self.declare_parameter('max_angular_vel', 1.0) # Robair MD49 motor max = 4.4rad/s
        self.declare_parameter('linear_accel', 0.7)
        self.declare_parameter('angular_accel', 1.3)
        self.declare_parameter('key_timeout', 0.5)  # seconds

        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.linear_accel = self.get_parameter('linear_accel').get_parameter_value().double_value
        self.angular_accel = self.get_parameter('angular_accel').get_parameter_value().double_value
        self.key_timeout = self.get_parameter('key_timeout').get_parameter_value().double_value

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0

        self.last_key_time = self.get_clock().now()

        # Key mapping
        self.move_bindings = {
            'i': (1, 0),
            'o': (1, -1),
            'u': (1, 1),
            'j': (0, 1),
            'l': (0, -1),
            ',': (-1, 0),
            'n': (-1, 1),
            ';': (-1, -1)
        }
        self.stop_key = 'k'
        self.lin_vel_inc_key = 'r'
        self.lin_vel_dec_key = 'f'
        self.ang_vel_inc_key = 'e'
        self.ang_vel_dec_key = 'd'

        # Last update timestamp
        self.last_time = self.get_clock().now()

    def get_key(self, timeout=0.0):
        """Return the last key pressed in this cycle, draining the buffer."""
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = ''
        while rlist:
            key = sys.stdin.read(1)
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
        return key

    def timer_callback(self):
        key = self.get_key(0.0)

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if key in self.move_bindings:
            lin, ang = self.move_bindings[key]
            self.target_linear = lin * self.max_linear_vel
            self.target_angular = ang * self.max_angular_vel
            self.last_key_time = now
        elif key == self.stop_key:
            # Immediate stop
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.last_key_time = now
        elif key == '\x03':  # Ctrl-C
            raise KeyboardInterrupt
        elif key == self.lin_vel_inc_key:
            self.max_linear_vel += 0.1
        elif key == self.lin_vel_dec_key:
            self.max_linear_vel = max(0, self.max_linear_vel - 0.1)
        elif key == self.ang_vel_inc_key:
            self.max_angular_vel += 0.3
        elif key == self.ang_vel_dec_key:
            self.max_angular_vel = max(0, self.max_angular_vel - 0.3)
        else:
            # No key pressed: check timeout for key release
            elapsed = (now - self.last_key_time).nanoseconds * 1e-9
            if elapsed > self.key_timeout:
                self.target_linear = 0.0
                self.target_angular = 0.0

        # Apply acceleration limits
        self.current_linear = self.update_velocity(self.current_linear, self.target_linear, self.linear_accel, dt)
        self.current_angular = self.update_velocity(self.current_angular, self.target_angular, self.angular_accel, dt)

        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.publisher_.publish(twist)

    @staticmethod
    def update_velocity(current, target, accel, dt):
        step = accel * dt
        if target > current:
            current = min(target, current + step)
        elif target < current:
            current = max(target, current - step)
        return current


def main(args=None):
    # Save original terminal settings
    settings = termios.tcgetattr(sys.stdin)

    # Raw + no-echo mode
    tty.setcbreak(sys.stdin.fileno())
    new_settings = termios.tcgetattr(sys.stdin)
    new_settings[3] = new_settings[3] & ~termios.ECHO
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)

    # Ensure terminal is restored on exit
    def restore_terminal():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    atexit.register(restore_terminal)

    rclpy.init(args=args)
    node = SmoothTeleopNode()

    print("""
Smooth Teleop Node
---------------------------
Control Your Robot!
Moving around:
   u    i    o
   j    k    l
   n    ,    ;

k : immediate stop
e: increase angular velocity limit
d: decrease angular velocity limit
r: increase linear velocity limit
f: decrease linear velocity limit
CTRL-C to quit
""")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
