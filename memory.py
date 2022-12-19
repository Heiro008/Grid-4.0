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
		# self.rc = OverrideRCIn()
		# self.set_point = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 1)
		# self.set_point_msg = PoseStamped()

		# self.set_point_msg.pose.position.x = 0.1
		# self.set_point_msg.pose.position.y = -0.15
		# self.set_point_msg.pose.position.z = 0.0

		# quat = tf.quaternion_from_euler(0,0,np.pi/2)	

		# self.set_point_msg.pose.orientation.x = quat[1]
		# self.set_point_msg.pose.orientation.y = quat[2]
		# self.set_point_msg.pose.orientation.z = quat[3]
		# self.set_point_msg.pose.orientation.w = quat[0]

		#first arm and then give commands
		#req = CommandBool()
		# self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
		# self.change_mode = self.create_client(SetMode, '/mavros/set_mode')
		# self.takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')


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
		# if height < 0.8:
		# 	if self.flag:
		# 		self.rc.channels = [1500,1500,1590,1500,1100,1000,1000,1000,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535]
		# 	else:	
		# 		self.rc.channels = [1500,1500,1500,1500,1300,1000,1000,1000,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535]


		# else:
		# 	self.rc.channels = [1500,1500,1500,1500,1300,1000,1000,1000,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535]
		# 	self.flag = False
		# 	if(self.flag):
		# 		self.override_pub.publish(self.rc)
		# 		#time.sleep(0.1)
		# 		while not self.change_mode.wait_for_service(timeout_sec=1):
		# 			self.get_logger().info('service not available, waiting again...')
		# 		req = SetMode.Request()
		# 		req.custom_mode = 'GUIDED'
		# 		resp = self.change_mode.call_async(req)
		# 		print('done')
		# 		#rclpy.spin_until_future_complete(self, resp)
		# 		self.get_logger().info('mode changed')
				

		# self.get_logger().info('published')
		# self.override_pub.publish(self.rc)
		# mode 0 = STABILIZE
		# mode 4 = GUIDED
		# mode 9 = LAND

		

def main(args=None):

    rclpy.init(args=args)
    control_node_ = control_node()
    rclpy.spin(control_node_)
    print('colsed')
    control_node_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()