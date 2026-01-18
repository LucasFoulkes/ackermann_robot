#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32

class FootprintPublisher(Node):
    def __init__(self):
        super().__init__('footprint_publisher')
        self.publisher_ = self.create_publisher(PolygonStamped, 'robot_footprint', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Define footprint points (Same as in nav2_params.yaml)
        # [[0.33, 0.125], [0.33, -0.125], [-0.06, -0.125], [-0.06, 0.125]]
        self.points = [
            [0.33, 0.125],
            [0.33, -0.125],
            [-0.06, -0.125],
            [-0.06, 0.125]
        ]
        
        self.get_logger().info('Footprint Publisher Node Started')

    def timer_callback(self):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        for p in self.points:
            point = Point32()
            point.x = float(p[0])
            point.y = float(p[1])
            point.z = 0.0
            msg.polygon.points.append(point)
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FootprintPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
