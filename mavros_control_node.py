import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import transformations as tf
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import OverrideRCIn
from mavros import command
from mavros import setpoint as SP


class control_node(Node):
	def __init__(self):
		super().__init__('control_node')
		self.override_pub = self.create_publisher(OverrideRCIn,"/mavros/rc/override",10)
		self.local_position = self.create_subscription(PoseStamped, '/mavros/local_position/position', self.update_height, 1)
		self.local_position
		self.rc = OverrideRCIn()
		self.change_mode = self.create_client(SetMode, '/mavros/set_mode')



	def update_height(self, data):

		local_position_msg = data
		height = data.pose.position.z

		if height < 0.8:
			self.rc.channels = [1500,1500,1550,1500,1200,1000,1000,1000,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535]

		else:
			self.rc.channels = [1500,1500,1500,1500,1200,1000,1000,1000,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535]
			self.get_logger().info('published')
        	self.override_pub.publish(self.rc)

			while not self.change_mode.wait_for_service(timeout_sec=1.0):
				self.get_logger().info('service not available, waiting again...')
			resp = self.change_mode.call_async(4)
			rclpy.spin_until_future_complete(node, resp)


		self.get_logger().info('published')
        self.override_pub.publish(self.rc)
         # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND

		

def main(args=None):

    rclpy.init(args=args)
    control_node = control_node()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()