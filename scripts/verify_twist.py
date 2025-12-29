#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class HandednessVerifier(Node):
    def __init__(self):
        super().__init__('handedness_verifier')
        self.pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Handedness Verifier Started: Publishing Forward (+X) twist continuously.")
        
    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" # or tool0, verifying base_link direction
        msg.twist.linear.x = 0.1 # Move 0.1 m/s in +X
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        
        self.pub.publish(msg)
        self.get_logger().info("Published Twist: +0.1 m/s X")

def main(args=None):
    rclpy.init(args=args)
    node = HandednessVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
