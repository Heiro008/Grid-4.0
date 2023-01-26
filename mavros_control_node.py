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
from mavros_msgs.srv import CommandLong
from mavros import setpoint as sp
from rclpy.qos import QoSProfile
from multiprocessing import shared_memory
from pymavlink import mavutil
from std_msgs.msg import Bool
from multiprocessing import resource_tracker



class control_node(Node):
	def __init__(self,master):
		super().__init__('control_node')

		self.rc = OverrideRCIn()
		self.set_point = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 1)
		self.package_check = self.create_publisher(Bool, '/package_check', 10)
		self.set_point_msg = PoseStamped()

		############################### shared memory variables #############################################
		self.a = np.array([0.0, 0.0], dtype=np.float64)

		self.shm = shared_memory.SharedMemory(name = 'Local_position', create=False, size=self.a.nbytes)
		self.local_pos_estimate = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm.buf)
		self.new_setpoint = None

		self.shm_vel = shared_memory.SharedMemory(name = 'local_velocity', create=False, size=self.a.nbytes)
		self.local_vel_estimate = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm_vel.buf)
		
		self.flags = np.array([0,0,0,0], dtype=bool)
		self.shm_flags = shared_memory.SharedMemory(name = 'flags', create=False, size=self.flags.nbytes)
		#self.b = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm.buf)
		self.flags_status = np.ndarray(self.flags.shape, dtype=self.flags.dtype, buffer=self.shm_flags.buf)
		# 0 -> package_detected
		# 1 -> package_coordinate_flag
		# 2 -> near_package 
		# 3 -> pose_package
		self.package_coordinate = np.array([0.0, 0.0], dtype=np.float64)
		self.shm_pkg_coord = shared_memory.SharedMemory(name = 'package_coordinate', create=False, size=self.package_coordinate.nbytes)
		self.pkg_coord = np.ndarray(self.package_coordinate.shape, dtype=self.package_coordinate.dtype, buffer=self.shm_pkg_coord.buf)
		
		self.angle = np.array([0.0], dtype=np.float64)
		self.shm_yaw_angle = shared_memory.SharedMemory(name = 'yaw_angle', create=False, size=self.angle.nbytes)
		self.yaw_angle = np.ndarray(self.angle.shape, dtype=self.angle.dtype, buffer=self.shm_yaw_angle.buf)

		resource_tracker.unregister("/Local_position", "shared_memory")
		resource_tracker.unregister("/flags", "shared_memory")
		resource_tracker.unregister("/package_coordinate", "shared_memory")
		resource_tracker.unregister("/yaw_angle", "shared_memory")
		resource_tracker.unregister("/local_velocity", "shared_memory")


		#####################################################################################################33
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
		self.takeoff_service = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
		self.command_long = self.create_client(CommandLong, '/mavros/cmd/command')
		self.start_navigation()



	def start_navigation(self):
		self.flags_status[3] = False   # pose_package_flag
		self.flags_status[0] = False
		self.flags_status[1] = True
		self.flags_status[2] = False
		self.pkg_coord[0] = 0.0 
		self.pkg_coord[1] = 0.0

		self.takeoff(1.0)
		# time.sleep(70)
		# self.land()
		# time.sleep(100)

		# self.goto_target((170.1+10)/100, (-88-15)/100, 1.0)
		time.sleep(10)
		count = 0
		target_list = [[1.2, 0.5], [0.0, 0.5], [0.0, 1.8], [1.2, 1.8]] 

		for target in target_list:
			self.flags_status[0] = False        #package_detected_flag  
			self.flags_status[1] = False        #package_coordinate_flag
			self.goto_target(target[0], target[1], 1.0)
			#self.flags_status[0] = True 		# package_detected_flag
			while not self.flags_status[0]:     # package_detected_flag
				if abs(self.local_pos_estimate[0]-target[0])<0.05 and abs(self.local_pos_estimate[1]-target[1])<0.05:   # local_point is shared memory
					print('Crossed target', target)
					break
				time.sleep(0.1)

			if self.flags_status[0]:      # package_detected_flag
				print('Package detected')
				print('package_coodinates',self.pkg_coord[0], self.pkg_coord[1])
				time.sleep(1)
				self.goto_target(self.local_pos_estimate[0], self.local_pos_estimate[1], 1.0)
				# time.sleep(5)

				while not self.flags_status[1]:     # package_coordinate_flag
					print('waiting for package_coodinates')
					time.sleep(0.1)

				print('package_coodinates',self.pkg_coord[0], self.pkg_coord[1])
				self.goto_target(self.pkg_coord[0], self.pkg_coord[1], 1.0)

				print('waiting for near_package')

				while True:        # near_package_flag  
					if abs(self.pkg_coord[0] - self.local_pos_estimate[0]) < 0.02 and abs(self.pkg_coord[1] - self.local_pos_estimate[1]) < 0.02:
							#self.flags_status[2] = True  #  near_package_flag
							print('near_package_set')
							break
					time.sleep(0.01)

				
				self.prev_point = self.local_pos_estimate.copy()   # array that contains x and y coordinates

				self.goto_target(self.pkg_coord[0],self.pkg_coord[1],1.0) ## reduce the height to 0.5  ( fix height based on field of view)
				time.sleep(1)
				self.goto_target(self.pkg_coord[0],self.pkg_coord[1],0.8)
				time.sleep(1)
				self.flags_status[3] = True   # pose_package_flag
				#pose_publisher pose estimation 
				self.goto_target(self.pkg_coord[0],self.pkg_coord[1],0.45)
				time.sleep(5)
				landing_offset = [0.0,-0.08]
				# setpoint as rotation
				# rotation_angle = self.yaw_angle[0] + np.pi/4
				rotation_angle = np.pi/2 + ((self.yaw_angle[0] % (np.pi/2)) - np.pi/4)   # check code: 90 + np.degrees(np.radians(30)%(np.pi/2) - np.pi/4)
				if self.yaw_angle[0]>0:
					rotation_angle = np.pi/2 - (self.yaw_angle[0] % (np.pi/2))
				else:
					rotation_angle = np.pi/2 - (self.yaw_angle[0] % (np.pi/2))

				# if count==0:
				# 	rotation_angle = np.radians(95)
				# else:
				# 	rotation_angle = np.radians(125)
				if rotation_angle<(np.pi/4):
					rotation_angle += np.pi/2			#90 + rotation_angle
				

				x_offset =  landing_offset[1] * np.sin(np.pi/2 - rotation_angle) 
				y_offset =  landing_offset[1] * np.cos(np.pi/2 - rotation_angle)
				print('offset_angle',np.degrees(self.yaw_angle[0]))    
				print('rotation_angle',np.degrees(rotation_angle),'offsets',x_offset,y_offset)
				# print('yaw angle', self.yaw_angle[0])

				while True:
					if abs(self.local_pos_estimate[0]-(self.pkg_coord[0]))<0.015 and abs(self.local_pos_estimate[1]-(self.pkg_coord[1]))<0.015:   # local_point is shared memory
						self.goto_target(self.pkg_coord[0], self.pkg_coord[1], 0.4, 0,0,rotation_angle)
						time.sleep(1)
						break
					time.sleep(0.1)

				#self.goto_target(self.pkg_coord[0], self.pkg_coord[1], 0.4, 0,0, rotation_angle)
				time.sleep(3)
				#self.goto_target(self.pkg_coord[0]-0.08, self.pkg_coord[1]-0.08, 0.4, 0,0,rotation_angle)
				# stability_count = 0
				# package_detected_flag
				self.goto_target(self.pkg_coord[0]+x_offset, self.pkg_coord[1]+y_offset, 0.35, 0,0,rotation_angle)

				while True:
					if abs(self.local_pos_estimate[0]-(self.pkg_coord[0]+x_offset))<0.010 and abs(self.local_pos_estimate[1]-(self.pkg_coord[1]+y_offset))<0.010:   # local_point is shared memory
						#self.goto_target(self.pkg_coord[0]+x_offset, self.pkg_coord[1]+y_offset, 0.1, 0,0,rotation_angle)
						print('vel',self.local_vel_estimate[0],self.local_vel_estimate[1] )
						if abs(self.local_vel_estimate[0])<0.1 and abs(self.local_vel_estimate[1])<0.06:
							self.land()
							time.sleep(2)
							self.force_disarm()
							break
					time.sleep(0.1)

		
				self.land()   # normal landing with height reduced
				time.sleep(2)
				for i in range(5):						# activate electromagnet
					package_checked_msg = Bool()
					package_checked_msg.data = True
					self.package_check.publish(package_checked_msg)
					print('picked up package')
					#self.flags_status[2] = True    # package_picked
				time.sleep(2)

				while not self.flags_status[2]:   # package_picked
					# takeoff and land again routne
					self.takeoff_and_land(rotation_angle,x_offset,y_offset)

				time.sleep(1)

				self.flags_status[3] = False   # pose_package_flag
				
				count += 1
				self.package_drop_routine(self.prev_point,count)  # drop and then come back to previous point

				self.flags_status[1] = False   # package_coordinate_flag
				self.flags_status[3] = False   # pose_package_flag
				self.flags_status[0] = False   # package_detected
				self.flags_status[2] = False 	# package_picked
				if count == 2:
					print('finish')
					break

	def force_disarm(self):
		while not self.command_long.wait_for_service(timeout_sec=1):
			self.get_logger().info('service not available, waiting again...')
		req = CommandLong.Request()
		req.broadcast = True
		req.command = 400     ## MAV_CMD_COMPONENT_ARM_DISARM
		req.param1 = 0.0
		req.param2 = 21196.0
		resp = self.command_long.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('Force Disarmed')



	def turn_off_motors(self):
		while not self.arm_service.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		req = CommandBool.Request()
		req.value = False
		resp = self.arm_service.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('disarmed')

	def goto_target(self,x,y,z,roll=0,pitch=0,yaw=np.pi/2):

		self.set_point_msg.pose.position.x = x
		self.set_point_msg.pose.position.y = y
		self.set_point_msg.pose.position.z = z

		quat = tf.quaternion_from_euler(roll,pitch,yaw)	

		self.set_point_msg.pose.orientation.x = quat[1]
		self.set_point_msg.pose.orientation.y = quat[2]
		self.set_point_msg.pose.orientation.z = quat[3]
		self.set_point_msg.pose.orientation.w = quat[0]

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

		while not self.takeoff_service.wait_for_service(timeout_sec=1):
			self.get_logger().info('service not available, waiting again...')
		req = CommandTOL.Request()
		req.altitude = height
		resp = self.takeoff_service.call_async(req)
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

	def terminate_motors(self):

		while not self.command_long.wait_for_service(timeout_sec=1):
			self.get_logger().info('service not available, waiting again...')
		req = CommandLong.Request()
		req.broadcast = True
		req.command = 185     ## MAV_CMD_DO_FLIGHTTERMINATION 
		req.param1 = 1.0
		resp = self.command_long.call_async(req)
		rclpy.spin_until_future_complete(self, resp)
		print('Motors off')

	def package_drop_routine(self,prev_point,count):

		drop_zone_x = (-34.5)/100
		drop_zone_y = (-90.3)/100
		self.takeoff(1.0)
		time.sleep(10)
		self.goto_target(drop_zone_x, drop_zone_y, 1.0) # drop_zone target
		while True:        # near_package_flag  
			if abs(drop_zone_x - self.local_pos_estimate[0]) < 0.05 and abs(drop_zone_y- self.local_pos_estimate[1]) < 0.05:
				#self.flags_status[2] = True  #  near_package_flag
				print('Near Dropzone')
				break
			time.sleep(0.1)

		#time.sleep(20)
		# self.goto_target(drop_zone_x, drop_zone_y, 0.6)
		time.sleep(1)

		# release electromagnet
		for i in range(5):
			package_picked_msg = Bool()
			package_picked_msg.data = False
			self.package_check.publish(package_picked_msg)
			print('Package', count, 'dropped')

		# self.goto_target(drop_zone_x, drop_zone_y, 1.0)	
		# time.sleep(1)

		if count == 2:
			print('setpoint landing pad')
			self.goto_target(-10/100, -50/100, 1.0)
			time.sleep(10)
			self.goto_target((119.5+20)/100, -50/100, 1.0)
			time.sleep(10)
			self.goto_target((170.2+10)/100, (-90.3-15-10)/100, 1.0)  # landing pad coordinates
			time.sleep(10)
			self.land()
		else:
			print('setpoint', self.prev_point)
			self.goto_target(prev_point[0],prev_point[1],1.0)

			#time.sleep(5)
			while True:  
				if abs(prev_point[0] - self.local_pos_estimate[0]) < 0.05 and abs(prev_point[1] - self.local_pos_estimate[1]) < 0.05:
					#self.flags_status[2] = True  #  near_package_flag	
					break
				time.sleep(0.1)

			## after dropping the package
			## goto_(landind_pad_coordinate)
	def takeoff_and_land(self,rotation_angle,x_offset,y_offset):
		self.takeoff(0.6)
		time.sleep(5)
		self.goto_target(self.pkg_coord[0],self.pkg_coord[1],0.45, 0,0,rotation_angle)
		print('takeoff_and_land',self.pkg_coord[0],self.pkg_coord[1],x_offset,y_offset)
		while True:
			if abs(self.local_pos_estimate[0]-(self.pkg_coord[0]))<0.015 and abs(self.local_pos_estimate[1]-(self.pkg_coord[1]))<0.015:   # local_point is shared memory
				self.goto_target(self.pkg_coord[0], self.pkg_coord[1], 0.4, 0,0,rotation_angle)
				time.sleep(1)
				break
			time.sleep(0.1)

		time.sleep(3)

		self.goto_target(self.pkg_coord[0]+x_offset, self.pkg_coord[1]+y_offset, 0.35, 0,0,rotation_angle)

		time.sleep(3)
		while True:
			if abs(self.local_pos_estimate[0]-(self.pkg_coord[0]+x_offset))<0.01 and abs(self.local_pos_estimate[1]-(self.pkg_coord[1]+y_offset))<0.01:   # local_point is shared memory
				print('vel',self.local_vel_estimate[0],self.local_vel_estimate[1] )
				if abs(self.local_vel_estimate[0])<0.1 and abs(self.local_vel_estimate[1])<0.06:
					self.land()
					time.sleep(2)
					self.force_disarm()
					break
			time.sleep(0.1)

		time.sleep(1)
		for i in range(5):						# activate electromagnet
			package_picked_msg = Bool()
			package_picked_msg.data = True
			self.package_check.publish(package_picked_msg)
			print('picked up package')
		time.sleep(2)


	def update_height(self, data):

		local_position_msg = data
		height = data.pose.position.z
		print(round(data.pose.position.x,5),round(data.pose.position.y,5),round(data.pose.position.z,5))

	def __del__(self):   # distructor
		del self.local_pos_estimate
		del self.flags_status
		del self.pkg_coord
		del self.yaw_angle
		del self.local_vel_estimate
		self.shm.close()
		self.shm_flags.close()
		self.shm_pkg_coord.close()
		self.shm_yaw_angle.close()
		self.shm_vel.close()
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
	master = mavutil.mavlink_connection('udpout:192.168.237.218:14590')
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
