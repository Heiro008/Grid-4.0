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
from pymavlink import mavutil

class control_node(Node):
	def __init__(self,master):
		super().__init__('control_node')
		#self.override_pub = self.create_publisher(OverrideRCIn,"/mavros/rc/override",10)

		# qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        #                                   history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        #                                   depth=5)
		# self.local_position = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.update_height, qos_profile)
		# self.local_position
		self.rc = OverrideRCIn()
		self.set_point = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 1)
		self.set_point_msg = PoseStamped()

		self.a = np.array([0.0, 0.0], dtype=np.float64)
		###############################
		self.shm = shared_memory.SharedMemory(name = 'Local_position', create=False, size=self.a.nbytes)
		self.local_pos_estimate = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm.buf)
		self.new_setpoint = None
		
		self.flags = np.array([0,0,0,0], dtype=np.bool)
		self.shm_flags = shared_memory.SharedMemory(name = 'flags', create=False, size=self.flags.nbytes)
		#self.b = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm.buf)
		self.flags_status = np.ndarray(self.flags.shape, dtype=self.flags.dtype, buffer=self.shm_flags.buf)
		# 0 -> package_detected
		# 1 -> package_coordinate_flag
		# 2 -> near_package 
		# 3 -> pose_package
		
		
		self.package_coordinate = np.array([0.0, 0.0], dtype=np.float64)
		self.shm_pkg_coord = shared_memory.SharedMemory(name = 'package_coordinate', create=False, size=self.package_coordinate.nbytes)
		self.pkg_coord = np.ndarray(self.shm_pkg_coord.shape, dtype=self.shm_pkg_coord.dtype, buffer=self.shm_shm_pkg_coord.buf)
		

		self.set_point_msg.pose.position.x = 0.1
		self.set_point_msg.pose.position.y = -0.15
		self.set_point_msg.pose.position.z = 0.0

		quat = tf.quaternion_from_euler(0,0,np.pi/2)	

		self.set_point_msg.pose.orientation.x = quat[1]
		self.set_point_msg.pose.orientation.y = quat[2]
		self.set_point_msg.pose.orientation.z = quat[3]
		self.set_point_msg.pose.orientation.w = quat[0]

		#first arm and then give commands
		#req = CommandBool()
		target_system = 0
		#target_system = 0   # 0 --> broadcast to everyone
		lat = 110200163  # Coimbatore
		lon = 770039725  # Coimbatore
		alt = 0 
		lattitude = lat
		longitude = lon
		altitude = alt
		master.mav.set_gps_global_origin_send(target_system,
				lattitude, 
				longitude,  
				altitude)
		print('sent origin')

		lattitude = lat
		longitude = lon
		altitude = alt

		x = 0
		y = 0
		z = 0
		q = [1, 0, 0, 0]   # w x y z

		approach_x = 0
		approach_y = 0
		approach_z = 1


		master.mav.set_home_position_send(
				target_system,
				lattitude,
				longitude,
				altitude,
				x,
				y,
				z,
				q,
				approach_x,
				approach_y,
				approach_z)
		print('sent home')


		self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
		self.change_mode = self.create_client(SetMode, '/mavros/set_mode')
		self.takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

		self.start_navigation()


		count = 0
		
		
		target_list = []    ############## to be defined ##############
		
#######################################################################################################################
		for target in target_list:
			self.goto_target(target[0],target[1],1.0)
			while not self.flags_status[0]:     # package_detected
				if self.local_pos_estimate[0]-target[0]<0.05 and self.local_pos_estimate[1]-target[1]<0.05:   # local_point is shared memory
					break
			if self.flags_status[0]:
				self.goto_target(self.local_pos_estimate[0], self.local_pos_estimate[1], 1.0)
				time.sleep(10)
				while not self.flags_status[1]:     # package_coordinate_flag
					pass
				self.goto_target(self.pkg_coord[0], self.pkg_coord[1], 1.0)

				self.flags_status[1] = False
				while not self.flags_status[2]:        # near_package  
					pass
				self.prev_point = self.local_pos_estimate   # array that contains x and y coordinates
				self.goto_target(self.prev_point[0],self.prev_point[0],0.8) ## reduce the height to 0.5  ( fix height based on field of view)
				#pose_package = True
				#pose_publisher pose estimation 
				time.sleep(5)
				self.goto_target(self.prev_point[0],self.prev_point[0],0.7)
				time.sleep(5)
				self.goto_target(self.prev_point[0],self.prev_point[0],0.6)
				time.sleep(5)
				self.goto_target(self.prev_point[0],self.prev_point[0],0.5)
				time.sleep(5)
				self.goto_target(self.prev_point[0],self.prev_point[0],0.4)
				time.sleep(5)
				self.goto_target(self.prev_point[0],self.prev_point[0],0.3)

				self.land()   # normal landing with height reduced

				#publish package_picked up topic

				pose_package = False

				count += 1
				self.package_drop_routine(self.prev_point,count)  # drop and then come back to previous point
				time.sleep(20)
				package_detected = False

#######################################################################################################################




	def start_navigation(self):
		self.takeoff(0.5)
		time.sleep(10)
		self.goto_target(0.1,-0.15,1.0)
		time.sleep(5)



	def goto_target(self,x,y,z):
		self.set_point_msg.pose.position.x = x
		self.set_point_msg.pose.position.y = y
		self.set_point_msg.pose.position.z = z

		print('setpoint')
		self.set_point_msg.header.stamp = self.get_clock().now().to_msg()
		self.set_point.publish(self.set_point_msg)

	def takeoff(self,height):		# change mode , arm , takeoff

		while not self.change_mode.wait_for_service(timeout_sec=1):
			self.get_logger().info('service not available, waiting again...')
		req = SetMode.Request()
		req.custom_mode = 'GUIDED'
		resp = self.change_mode.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('mode changed')

		time.sleep(2)

		while not self.arm_service.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		req = CommandBool.Request()
		req.value = True
		resp = self.arm_service.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('armed')

		time.sleep(2)

		while not self.takeoff.wait_for_service(timeout_sec=1):
			self.get_logger().info('service not available, waiting again...')
		req = CommandTOL.Request()
		req.altitude = height
		resp = self.takeoff.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('takeoff')

	def land(self):
		print('landing')
		while not self.change_mode.wait_for_service(timeout_sec=1):
			self.get_logger().info('service not available, waiting again...')
		req = SetMode.Request()
		req.custom_mode = 'LAND'
		resp = self.change_mode.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('Landed')

	def package_drop_routine(self,prev_point):

		##

		pass
		if count==2:
			## after dropping the package
			## goto_(landind_pad_coordinate)
		##
	def update_height(self, data):

		local_position_msg = data
		height = data.pose.position.z
		print(round(data.pose.position.x,5),round(data.pose.position.y,5),round(data.pose.position.z,5))

	def __del__(self):   # distructor
		del self.local_pos_estimate
		self.shm.close()
		print('closed')

def wait_conn(master):
	"""
	Sends a ping to stabilish the UDP communication and awaits for a response
	"""
	msg = None
	while not msg:
		master.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
		msg = master.recv_match()
		time.sleep(0.5)




def main(args=None):
	master = mavutil.mavlink_connection('udpout:192.168.252.218:14590')
	#master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
	print('waiting')
	#master.wait_heartbeat()
	wait_conn(master)
	rclpy.init(args=args)
	control_node_ = control_node(master)
	rclpy.spin(control_node_)
	control_node_.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
