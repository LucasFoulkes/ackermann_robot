import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray
import math

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        
        # Parameters to tune the mapping
        # max_speed: The linear speed (m/s) that corresponds to 100% throttle
        self.declare_parameter('max_speed', 2.0) 
        
        # max_steering_angle: Reduced by 29% (was 0.52) -> 0.37 rad (~21 degrees)
        self.declare_parameter('max_steering_angle', 0.37) 

        # wheelbase: Distance between front and rear axles in meters
        self.declare_parameter('wheelbase', 0.40)

        self._max_speed = self.get_parameter('max_speed').value
        self._max_steering_angle = self.get_parameter('max_steering_angle').value
        self._wheelbase = self.get_parameter('wheelbase').value

        # Subscribe to standard Twist (Teleop)
        self.sub_twist = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Subscribe to TwistStamped (Nav2)
        self.sub_stamped = self.create_subscription(TwistStamped, 'cmd_vel_nav', self.cmd_vel_stamped_callback, 10)

        self.pub = self.create_publisher(Float32MultiArray, '/ackermann/cmd', 10)
        
        self.get_logger().info(f"CmdVelBridge started. Max Speed={self._max_speed} m/s, Max Steer={self._max_steering_angle} rad, Wheelbase={self._wheelbase} m")

    def cmd_vel_stamped_callback(self, msg):
        # Extract Twist from TwistStamped and process
        self.process_twist(msg.twist)

    def cmd_vel_callback(self, msg):
        # Process Twist directly
        self.process_twist(msg)

    def process_twist(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # 1. Calculate Throttle %
        if self._max_speed > 0:
            throttle = (v / self._max_speed) * 100.0
        else:
            throttle = 0.0
        throttle = max(-100.0, min(100.0, throttle))

        # 2. Calculate Steering Angle using Ackermann geometry
        # delta = arctan(L * w / v)
        # If v is very small, we can't steer effectively based on w alone in this model,
        # but usually for teleop we might want to allow steering in place or near-zero speed.
        # However, true Ackermann cannot rotate in place.
        
        if abs(v) < 0.01:
            # Stopped or very slow: 
            # If we are stopped, we can't really define a turning radius from v and w.
            # But for teleop feel, if user presses 'left' while stopped, they expect wheels to turn.
            # We can approximate by assuming a small 'virtual' speed or just mapping w directly if v=0.
            # Let's map w directly to steering angle rate or just position for simplicity in teleop.
            # A common trick is to treat w as "steering command" when v=0.
            # Let's assume w is proportional to max steering angle.
             steering_angle = 0.0
             if abs(w) > 0.001:
                 # If trying to turn while stopped, turn wheels fully in that direction
                 steering_angle = math.copysign(self._max_steering_angle, w)
        else:
            # Ackermann formula: tan(delta) = L / R = L * (w / v)
            # delta = arctan(L * w / v)
            steering_angle = math.atan(self._wheelbase * w / v)

        # 3. Map Steering Angle to % (-100 to 100)
        if self._max_steering_angle > 0:
            steering_pct = (steering_angle / self._max_steering_angle) * 100.0
        else:
            steering_pct = 0.0
            
        steering_pct = max(-100.0, min(100.0, steering_pct))

        out_msg = Float32MultiArray()
        out_msg.data = [steering_pct, throttle]
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
