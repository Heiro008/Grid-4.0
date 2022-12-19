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

		time.sleep(2)

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
		#self.delay_sec(2)

		self.set_point_msg.pose.position.x = 0.1
		self.set_point_msg.pose.position.y = -0.15
		self.set_point_msg.pose.position.z = 1.0

		while not self.takeoff.wait_for_service(timeout_sec=1):
			self.get_logger().info('service not available, waiting again...')
		req = CommandTOL.Request()
		req.altitude = 0.6
		resp = self.takeoff.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('takeoff')


		time.sleep(10)
		#self.delay_sec(10)

		print('setpoint')
		self.set_point_msg.header.stamp = self.get_clock().now().to_msg()
		self.set_point.publish(self.set_point_msg)

		time.sleep(10)
		count = 0

		for targets in target_list:
			goto_target(targets[0],targets[1],1.0)
			while not package_detected:
				if local_point == target:   # local_point is shared memory
					break
			if package_detected:
				goto_(local_position)
				time.sleep(10)
				while not package_coordinate_flag:
					pass
				goto_(package_coordinate)

				package_coordinate_flag = False
				while not near_package:
					pass
				prev_point = local_position
				goto_(local_position) ## reduce the height to 0.5  ( fix height based on field of view)
				pose_package = True
				#pose_publisher pose estimation 
				precision_land()
				#publish package_picked up topic
				pose_package = False
				count += 1
				self.package_drop_routine(prev_point,count)  # drop and then come back to previous point
				time.sleep(20)
				package_detected = False



		self.set_point_msg.pose.position.x = 0.54
		self.set_point_msg.pose.position.y = -0.12
		self.set_point_msg.pose.position.z = 1.0

		print('1st setpoint')
		self.set_point_msg.header.stamp = self.get_clock().now().to_msg()
		self.set_point.publish(self.set_point_msg)

		time.sleep(15)
		# print(self.local_pos_estimate)

		
		# self.set_point_msg.pose.position.x = 0.8
		# self.set_point_msg.pose.position.y = 0.9
		# self.set_point_msg.pose.position.z = 1.0

		# print('2nd setpoint')
		# self.set_point_msg.header.stamp = self.get_clock().now().to_msg()
		# self.set_point.publish(self.set_point_msg)

		# time.sleep(15)
		
		# print(self.local_pos_estimate)
		
		# self.set_point_msg.pose.position.x = 0.8
		# self.set_point_msg.pose.position.y = 1.8
		# self.set_point_msg.pose.position.z = 1.0

		# print('3rd setpoint')
		# self.set_point_msg.header.stamp = self.get_clock().now().to_msg()
		# self.set_point.publish(self.set_point_msg)

		# #self.delay_sec(10)
		# time.sleep(10)

		print(self.local_pos_estimate)
		self.new_setpoint = self.local_pos_estimate

		print('landing')
		while not self.change_mode.wait_for_service(timeout_sec=1):
			self.get_logger().info('service not available, waiting again...')
		req = SetMode.Request()
		req.custom_mode = 'LAND'
		resp = self.change_mode.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('Landed')

		time.sleep(20)

		self.set_point_msg.pose.position.x = self.new_setpoint[0]
		self.set_point_msg.pose.position.y = self.new_setpoint[1]
		self.set_point_msg.pose.position.z = 0.0

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
		req.altitude = 0.8
		resp = self.takeoff.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('takeoff')

		

		time.sleep(10)

		self.set_point_msg.pose.position.x = 0.1
		self.set_point_msg.pose.position.y = -0.15
		self.set_point_msg.pose.position.z = 1.0

		print('setpoint 4')

		self.set_point_msg.header.stamp = self.get_clock().now().to_msg()
		self.set_point.publish(self.set_point_msg)




		time.sleep(10)

		print('landing')
		while not self.change_mode.wait_for_service(timeout_sec=1):
			self.get_logger().info('service not available, waiting again...')
		req = SetMode.Request()
		req.custom_mode = 'LAND'
		resp = self.change_mode.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('Landed')



	def goto_target(x,y,z):
		self.set_point_msg.pose.position.x = x
		self.set_point_msg.pose.position.y = y
		self.set_point_msg.pose.position.z = z

		#print('1st setpoint')
		self.set_point_msg.header.stamp = self.get_clock().now().to_msg()
		self.set_point.publish(self.set_point_msg)


	def __del__(self):
		del self.local_pos_estimate
		self.shm.close()
		print('closed')

	# def delay_sec(self,t):
	# 	i_time = time.time()
	# 	while time.time() - i_time < t:
	# 		#time.sleep(0.5)
	# 		pass

	def package_drop_routine(prev_point):

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