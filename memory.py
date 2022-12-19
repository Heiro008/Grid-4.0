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
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros import setpoint as sp
from rclpy.qos import QoSProfile
from multiprocessing import shared_memory


class control_node(Node):
	def __init__(self):
		super().__init__('memory_node')
		#self.override_pub = self.create_publisher(OverrideRCIn,"/mavros/rc/override",10)

		qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=5)
		self.local_position = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.callback, qos_profile)
		self.local_position

		self.a = np.array([0.0, 0.0], dtype=np.float64)
		self.shm = shared_memory.SharedMemory(name = 'Local_position', create=True, size=self.a.nbytes)
		self.b = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm.buf)

		# 0 -> package_detected
		# 1 -> package_coordinate_flag
		# 2 -> near_package 
		# 3 -> pose_package
		self.flags = np.array([0,0,0,0], dtype=np.bool)
		self.shm_flags = shared_memory.SharedMemory(name = 'flags', create=True, size=self.flags.nbytes)
		#self.b = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm.buf)
		self.package_coordinate = np.array([0.0, 0.0], dtype=np.float64)
		self.shm = shared_memory.SharedMemory(name = 'package_coordinate', create=True, size=self.package_coordinate.nbytes)
		#self.b = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm.buf)	



	def __del__(self):
		del self.b
		self.shm.close()
		self.shm.unlink()
		print('closed')

			
	def callback(self, data):

		local_position_msg = data
		height = data.pose.position.z
		print(round(data.pose.position.x,5),round(data.pose.position.y,5),round(data.pose.position.z,5))
		self.a[0] = data.pose.position.x
		self.a[1] = data.pose.position.y
		#b = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm.buf)
		self.b[:] = self.a[:]

		

def main(args=None):

    rclpy.init(args=args)
    control_node_ = control_node()
    rclpy.spin(control_node_)
    print('colsed')
    control_node_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()