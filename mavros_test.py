import sys
import argparse
import rclpy
import mavros
from rclpy.node import Node
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point
from mavros_msgs.msg import OverrideRCIn
from mavros import command
from mavros import setpoint as SP



class rc_override(Node):
    def __init__(self):
        super().__init__('rc_publisher')
        self.override_pub = self.create_publisher(OverrideRCIn,"/mavros/rc/override",10)
        self.rc = OverrideRCIn()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        self.rc.channels = [1000,1000,1000,1000,1000,1000,1000,1000,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535]
        self.get_logger().info('published')
        self.override_pub.publish(self.rc)


def main(args=None):

    rclpy.init(args=args)
    rc_publisher = rc_override()
    rclpy.spin(rc_publisher)
    rc_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()