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
from std_msgs.msg import Float32
from multiprocessing import shared_memory
from multiprocessing import resource_tracker




def isRotationMatrix(R):
	Rt = np.transpose(R)
	shouldBeIdentity = np.dot(Rt, R)
	I = np.identity(3, dtype=R.dtype)
	n = np.linalg.norm(I - shouldBeIdentity)
	return n < 1e-6
def rotationMatrixToEulerAngles(R):
	assert (isRotationMatrix(R))
	sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
	singular = sy < 1e-6
	if not singular:
		x = np.arctan2(R[2, 1], R[2, 2])
		y = np.arctan2(-R[2, 0], sy)
		z = np.arctan2(R[1, 0], R[0, 0])
	else:
		x = np.arctan2(-R[1, 2], R[1, 1])
		y = np.arctan2(-R[2, 0], sy)
		z = 0
	return np.array([x, y, z])


class ImageSubscriber(Node):
	def __init__(self):
		super().__init__('image_subscriber')   # subscriber node name
		#self.subscription = self.create_subscription(CompressedImage, 'camera_image', self.listener_callback, 10)
		########################### shared memory variables#######################################################

		self.a = np.array([0.0, 0.0], dtype=np.float64)
		self.shm = shared_memory.SharedMemory(name = 'Local_position', create=False, size=self.a.nbytes)
		self.local_pos_estimate = np.ndarray(self.a.shape, dtype=self.a.dtype, buffer=self.shm.buf)
		
		self.flags = np.array([0,0,0,0], dtype=bool)
		self.shm_flags = shared_memory.SharedMemory(name = 'flags', create=False, size=self.flags.nbytes)
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
		print(self.pkg_coord)
		resource_tracker.unregister("/Local_position", "shared_memory")
		resource_tracker.unregister("/flags", "shared_memory")
		resource_tracker.unregister("/package_coordinate", "shared_memory")
		resource_tracker.unregister("/yaw_angle", "shared_memory")

		###########################################################################################################

		self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 1)
		self.subscription
		self.range_finder = self.create_subscription(Float32,'/range_finder/ultrasonic', self.update_height, 10)
		self.range_finder
		self.object_pose = self.create_publisher(PoseStamped, '/vision/pose', 1)
		self.camera_pose = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 1)
		#self.create_timer(1/30, self.callback)
		self.height = 0
		self.object_pose_msg = PoseStamped()
		self.camera_pose_msg = PoseStamped()
		self.set_point_msg = PoseStamped()
		self.imgae_header = 0
		self.camera_pose_msg.pose.position.x = (170.2+10)/100
		self.camera_pose_msg.pose.position.y = (-90.3-15)/100
		self.camera_pose_msg.pose.position.z = 0.0
		self.camera_pose_msg.pose.orientation.x = 0.0
		self.camera_pose_msg.pose.orientation.y = 0.0
		self.camera_pose_msg.pose.orientation.z = 0.7071 
		self.camera_pose_msg.pose.orientation.w = 0.7071
		self.offset_yaw = None
		self.q_rot_offset = None
		self.prev_yaw_angle = None
		self.to_be_named_by_dev = False
		self.br = CvBridge()
		self.image_data = None
		
		self.change_pose_count = 0

		self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
		self.arucoDict_package = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
		self.arucoParams = cv2.aruco.DetectorParameters_create()

		##########################################################################################################
		self.matrix_coefficients = np.array( [[472.46389744  , 0.      ,   265.15033673],
 									[  0.    ,     473.06566789 ,285.9249716 ],
 									[  0.    ,       0.   ,        1.        ]] )

		self.distortion_coefficients = np.array([[-0.39297927,  0.148505,   -0.00703395,  0.01307684 ,-0.03332911]])

		self.matrix_coefficients = np.array(  [[432.68955569 ,  0.       ,  327.94237612],
										 [  0.        , 432.12302131 ,245.54538111],
 									[  0.  ,         0.    ,       1.        ]]  )

		self.distortion_coefficients = np.array( [[-3.89046329e-01 , 1.80504834e-01 , 2.65083029e-05,  8.39683505e-06 , -4.45792442e-02]])
		############################################################################################################
		self.tag_length = 0.055  # in metres, length of one marker on the board
		self.tag_separation = 0.007   # HAVE TO REDECLARE PROPERLY (in metres again, distance between adjacent markers)
		#self.board = cv2.aruco.GridBoard_create(4, 3, self.tag_length, self.tag_separation, self.arucoDict)
		        # first number = no. of columns of markers in the board
		        # second number = no. of rows of markers in the board
		self.tag_length_package = 0.058
		self.no_of_boards = 8
		self.boards = []
		self.board_details = {}
		for i in range(self.no_of_boards):
			self.boards.append(cv2.aruco.GridBoard_create(4, 3, self.tag_length, self.tag_separation, self.arucoDict, 12*i))
		        	# first number = no. of columns of markers in the board
			        # second number = no. of rows of markers in the board
			        # last number = id of the first marker in the gridboard! very important.
			self.board_details[i] = [[j for j in range(12*i, 12*(i+1))], None, None, None, None]
		
		######## CHECK THESE VALUES ###########################################
		self.board_details[0][1] = (0,0)
		self.board_details[1][1] = (123.2/100, 0)           #(119.5/100, 0)
		self.board_details[2][1] = (0, 96/100)              #(0, 95.3/100)
		self.board_details[3][1] = (123.2/100, 96/100)     #(119.5/100, 95.3/100)		# 189.5
		self.board_details[4][1] = (0, 190.8/100)           #(0, 189.5/100)
		self.board_details[5][1] = (123.2/100, 190.8/100)   #(119.5/100, 189.5/100)
		self.board_details[6][1] = (-4.5/100 , -90.3/100)		#(-8/100 , -88/100)
		self.board_details[7][1] = (170.2/100, -90.3/100)     #(170.1/100, -88.8/100)
		###########################################################################3

		self.pkg_coord_x_list = []
		self.pkg_coord_y_list = []
			
	def update_height(self,data):
		self.height = float(data.data/100)


	def listener_callback(self, data):
		#self.get_logger().info('Receiving video frame')    # for logging with timestamp
		#print(data)
		#np_arr = np.array(data.data, np.uint8)
		#self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		self.image_header = data.header.frame_id
		self.image_data = self.br.imgmsg_to_cv2(data)
		self.image_data = cv2.cvtColor(self.image_data, cv2.COLOR_RGB2BGR)
		self.image_flag = True
		grayColor = cv2.cvtColor(self.image_data, cv2.COLOR_BGR2GRAY)
		sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
		sharpen = cv2.filter2D(grayColor, -1, sharpen_kernel)
		lwr = np.array([0, 0, 100])
		upr = np.array([179, 61, 252])
		hsv = cv2.cvtColor(self.image_data, cv2.COLOR_BGR2HSV)
		msk = cv2.inRange(hsv, lwr, upr)
		krn = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
		dlt = cv2.dilate(msk, krn, iterations=1)
		#res = 255 - cv2.bitwise_and(dlt, msk)
		# res = np.uint8(res)
		self.main_process(self.image_data)
		#cv2.imshow("camera", current_frame)
		#cv2.waitKey(1)
		
	def main_process(self,image):
		# self.flags_status[3] = True
		# self.flags_status[0] = True
		#self.flags_status[1] = False
		(corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
		#(corners_package, ids_package, rejected_package) = cv2.aruco.detectMarkers(image, self.arucoDict_package, parameters=self.arucoParams)
		#cv2.aruco.refineDetectedMarkers(image, self.board, corners, ids, rejected)  # not sure what this is for
		       # maybe this above function can be applied only when the board alone is visible in frame?
		       # i think not, but just check once to be sureee

		tvec = None
		rvec = None

		# if 0 in ids_package:
		# 	package_detected = True   
		if not self.flags_status[2]:     # package_picked_flag
			self.flags_status[0] , package_corners, package_tag_length = self.detect_get_pkg_corners(image)   # package_detected_flag
		 
		if len(corners) > 6 and not self.flags_status[3]:   # or ids!=None
			if self.flags_status[2]:
				self.to_be_named_by_dev = False
			self.offset_yaw = None
			self.change_pose_count = 0      # reset the counter if normal board is detected
			corners_split = [[] for i in range(self.no_of_boards)]  # have to check if datatype matches
				# this will store the corners of each board in the arena, separately.
				# for boards that are not detected in the frame, empty list [] will be stored
			
			ids_split = [[] for i in range(self.no_of_boards)]


			for i in range(len(ids)):
				if ids[i]<96:
					j = ids[i][0] // 12
					ids_split[j].append([ids[i]])
					corners_split[j].append(corners[i])
						
			for i in range(len(ids_split)):
				ids_split[i] = np.reshape(ids_split[i], (len(ids_split[i]), 1))

			
			boards_detected = set()
			board_selected = None
			
			for i in range(len(ids_split)):
				tvec = None
				rvec = None
				
				if len(ids_split[i]) > 6:
					ret_val, rvec, tvec = cv2.aruco.estimatePoseBoard(corners_split[i], ids_split[i], self.boards[i], self.matrix_coefficients, self.distortion_coefficients,rvec,tvec)
					self.board_details[i][2] = tvec
					self.board_details[i][3] = rvec
					rvec_tmp, tvec_tmp, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners_split[i][0], self.tag_length, self.matrix_coefficients,self.distortion_coefficients)
					self.board_details[i][4] = float(tvec_tmp[0][0][0])**2 + float(tvec_tmp[0][0][1])**2
					boards_detected.add(i)
				else:
					continue
					
			boards_detected = list(boards_detected)
				
			if boards_detected:
				min_dis = self.board_details[boards_detected[0]][4]
				for i in boards_detected:
					if self.board_details[i][4] <= min_dis:
						min_dis = self.board_details[i][4]
						board_selected = i		

			#rvec, tvec, markerPoints = cv2.aruco.estimatePoseBoard(corners,ids, self.tag_length, self.matrix_coefficients, self.distortion_coefficients, tvec, rvec)
			if board_selected!=None:
				tvec = None
				rvec = None
				
				detectedCorners, detectedIds, rejectedCorners,_ = cv2.aruco.refineDetectedMarkers(image, self.boards[board_selected], corners_split[board_selected], ids_split[board_selected], rejected)
				# ret_val, rvec, tvec = cv2.aruco.estimatePoseBoard(corners_split[board_selected], ids_split[board_selected], self.boards[board_selected], self.matrix_coefficients, self.distortion_coefficients,rvec,tvec)
				ret_val, rvec, tvec = cv2.aruco.estimatePoseBoard(detectedCorners, detectedIds, self.boards[board_selected], self.matrix_coefficients, self.distortion_coefficients,rvec,tvec)

				if (not self.flags_status[1]) and self.flags_status[0] :   # package_detected_flag  and  package_coodinate_flag
					########## tvec of 1st marker of board and tvec of package marker  ##################
					rvec_tmp, tvec_tmp, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners_split[board_selected][0], self.tag_length, self.matrix_coefficients,self.distortion_coefficients)
					id_tmp = ids_split[board_selected][0]
					rvec_pkg, tvec_pkg, markerPoints = cv2.aruco.estimatePoseSingleMarkers(package_corners, package_tag_length, self.matrix_coefficients,self.distortion_coefficients)
					j = (id_tmp%12)
					col = j % 4
					row = j // 4

					marker_offset_wrt_board_x = col*(self.tag_length + self.tag_separation) + self.tag_length/2
					marker_offset_wrt_board_y = row*(self.tag_length + self.tag_separation) + self.tag_length/2

					offset_x = (tvec_pkg[0][0][0] - tvec_tmp[0][0][0]) + marker_offset_wrt_board_x
					offset_y = (tvec_tmp[0][0][1] - tvec_pkg[0][0][1]) - marker_offset_wrt_board_y
					offset_x  += self.board_details[board_selected][1][0]
					offset_y  += self.board_details[board_selected][1][1]
					#print('tvec',tvec_pkg[0][0][0], tvec_pkg[0][0][1])
					# self.pkg_coord[0] = offset_x
					# self.pkg_coord[1] = offset_y
					if len(self.pkg_coord_x_list) < 5:
						self.pkg_coord_x_list.append(offset_x)
						self.pkg_coord_y_list.append(offset_y)
					else:
						if np.std(self.pkg_coord_x_list) < 0.05  and  np.std(self.pkg_coord_y_list) < 0.05:
							self.pkg_coord[0] = offset_x
							self.pkg_coord[1] = offset_y

							self.flags_status[1] = True   # package_coodinate_flag
							
							print('package_coordinate',self.pkg_coord[0],self.pkg_coord[1])

							self.pkg_coord_y_list = []
							self.pkg_coord_x_list = []

						else:
							print(np.std(self.pkg_coord_x_list), np.std(self.pkg_coord_y_list))
							self.pkg_coord_x_list.pop(0)
							self.pkg_coord_x_list.append(offset_x)
							self.pkg_coord_y_list.pop(0)
							self.pkg_coord_y_list.append(offset_y)


							
					
				# Draw a square around the markers
				cv2.aruco.drawDetectedMarkers(image, corners_split[board_selected], ids_split[board_selected], (0,255,0)) 
				
				# tvec = self.board_details[board_selected][2]
				# rvec = self.board_details[board_selected][3]
				
				self.object_pose_msg.header.stamp = self.get_clock().now().to_msg()
				self.object_pose_msg.header.frame_id = 'camera'

				self.object_pose_msg.header.stamp = self.get_clock().now().to_msg()
				self.object_pose_msg.pose.position.x = float(tvec[0][0])   #+ offset[ids[i]][0]
				self.object_pose_msg.pose.position.y = float(tvec[1][0])   #+ offset[ids[i]][1]
				self.object_pose_msg.pose.position.z = float(tvec[2][0])

				rvec = rvec.reshape(1,1,3)
				tvec = tvec.reshape(1,1,3)

				cv2.drawFrameAxes(image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)  

				rot_mat = cv2.Rodrigues(rvec)
				euler_angles = rotationMatrixToEulerAngles(rot_mat[0])
				p_quat = Quaternion()
				if euler_angles[0] > 0:
					euler_angles[0] = euler_angles[0] - np.pi
				else:
					euler_angles[0] = euler_angles[0] + np.pi
				p_quat_raw = tf.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
				p_quat.w = p_quat_raw[0]		
				p_quat.x = p_quat_raw[1]
				p_quat.y = p_quat_raw[2]
				p_quat.z = p_quat_raw[3]
				#p_quat = createQuaternionMsgFromRollPitchYaw(euler_angles[0], euler_angles[1], euler_angles[2])

				self.object_pose_msg.pose.orientation = p_quat

				self.object_pose.publish(self.object_pose_msg)

				transform = tf.compose_matrix(translate=tvec,angles=euler_angles)
				inv_transform = tf.inverse_matrix(transform)
				camera_origin = tf.translation_from_matrix(inv_transform)
				camera_quaternion = tf.quaternion_from_euler(np.pi, 0, euler_angles[2])
				#'sxyz'
				q_rot = tf.quaternion_from_euler(np.pi,0,np.pi*1.5)	  # yaw subtracted from euler_angle
				q_new = tf.quaternion_multiply(q_rot,camera_quaternion)
				q_new = tf.unit_vector(q_new)
				#q_new.normalize()
				self.camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
				self.camera_pose_msg.header.frame_id = 'map'
				camera_origin[0] = camera_origin[0] + self.board_details[board_selected][1][0] 
				camera_origin[1] = camera_origin[1] + self.board_details[board_selected][1][1]
				#print(i, camera_origin)
				self.camera_pose_msg.pose.position.x = camera_origin[0]
				self.camera_pose_msg.pose.position.y = camera_origin[1]
				self.camera_pose_msg.pose.position.z = camera_origin[2]    #float(self.height) 
				self.camera_pose_msg.pose.orientation.x = -q_new[2]  # y value
				self.camera_pose_msg.pose.orientation.y = -q_new[1]  # x value
				self.camera_pose_msg.pose.orientation.z = -q_new[3]  # z value
				self.camera_pose_msg.pose.orientation.w = q_new[0]  # w value

				self.camera_pose.publish(self.camera_pose_msg)

		elif self.flags_status[3]:   # pose_package_flag
			#pose estimation based on package marker 
			self.publish_pose_based_on_marker(image)

		else:
			# if self.flags_status[2] and self.change_pose_count > 5:
			# 	self.flags_status[3] = True
			self.change_pose_count += 1         # count to change the pose estimation method
			self.camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
			self.camera_pose_msg.header.frame_id = 'map'
			if self.height < 50 :
				self.camera_pose_msg.pose.position.z = float(self.height)
			self.camera_pose.publish(self.camera_pose_msg)

		cv2.imshow("image", image)
		key = cv2.waitKey(1)



	def publish_pose_based_on_marker(self,image):
		tag_length = 0.058
		#(corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.arucoDict_package,parameters=self.arucoParams)
		detected , corners, tag_length = self.detect_get_pkg_corners(image)
		tvec = None
		rvec = None
		if detected:

        	# Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
			rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], tag_length, self.matrix_coefficients,self.distortion_coefficients)
			#rvec, tvec, markerPoints = cv2.aruco.estimatePoseBoard(corners,ids, self.tag_length, self.matrix_coefficients,self.distortion_coefficients,tvec,rvec)
			cv2.aruco.drawDetectedMarkers(image, corners) 
			#print(rvec[i][0])
			rot_mat = cv2.Rodrigues(rvec[0][0])
			euler_angles = rotationMatrixToEulerAngles(rot_mat[0])

			self.yaw_angle[0] = euler_angles[2]
			if self.offset_yaw == None:
				self.offset_yaw = euler_angles[2]
				self.prev_yaw_angle = euler_angles[2] 
				self.q_rot_offset = tf.quaternion_from_euler(0,0,self.offset_yaw)
				print('offset angle', np.degrees(self.offset_yaw))

			p_quat = Quaternion()
			p_quat_raw = tf.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
			p_quat.w = p_quat_raw[0]		
			p_quat.x = p_quat_raw[1]
			p_quat.y = p_quat_raw[2]
			p_quat.z = p_quat_raw[3]
			#p_quat = createQuaternionMsgFromRollPitchYaw(euler_angles[0], euler_angles[1], euler_angles[2])


			if self.detected_id in [3,4,5]:
				if self.prev_yaw_angle - euler_angles[2]  > np.radians(80):
					print('shifted1')
					self.offset_yaw -= np.pi/2
					#euler_angles[2] = euler_angles[2] - self.offset_yaw
				elif self.prev_yaw_angle - euler_angles[2]  < -np.radians(80):
					print('shifted2')
					self.offset_yaw += np.pi/2
					#self.prev_yaw_angle = euler_angles[2] + np.pi/2

			self.prev_yaw_angle = euler_angles[2]

			self.q_rot_offset = tf.quaternion_from_euler(0,0,self.offset_yaw)

			q_rotated_along_yaw = tf.quaternion_multiply(p_quat_raw, self.q_rot_offset)
				
			euler_angles_rotated = tf.euler_from_quaternion(q_rotated_along_yaw,axes='sxyz')
			# print(np.degrees(euler_angles[2]),np.degrees(euler_angles_rotated[2]))

			euler_angles[2] = euler_angles[2] - self.offset_yaw   ## this is must for the orientation offset
			transform = tf.compose_matrix(translate=tvec,angles=euler_angles_rotated)

			inv_transform = tf.inverse_matrix(transform)
			camera_origin = tf.translation_from_matrix(inv_transform)
			camera_quaternion = tf.quaternion_from_euler(np.pi, 0, euler_angles[2])
			#'sxyz'
			q_rot = tf.quaternion_from_euler(np.pi,0, np.pi*1.5) #+self.offset_yaw
			q_new = tf.quaternion_multiply(q_rot,camera_quaternion)					
			q_new = tf.unit_vector(q_new)  		# q_new.normalize()
			
			self.camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
			self.camera_pose_msg.header.frame_id = 'map'
			self.camera_pose_msg.pose.position.x = camera_origin[0]  + self.pkg_coord[0]
			self.camera_pose_msg.pose.position.y = camera_origin[1]  + self.pkg_coord[1] 
			self.camera_pose_msg.pose.position.z = camera_origin[2]
			self.camera_pose_msg.pose.orientation.x = -q_new[2]  # y value
			self.camera_pose_msg.pose.orientation.y = -q_new[1]  # x value
			self.camera_pose_msg.pose.orientation.z = -q_new[3]  # z value
			self.camera_pose_msg.pose.orientation.w = q_new[0]  # w value
			self.camera_pose.publish(self.camera_pose_msg)

			cv2.drawFrameAxes(image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)  
		else:

			self.camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
			self.camera_pose_msg.header.frame_id = 'map'
			if self.height < 50 :
				self.camera_pose_msg.pose.position.z = float(self.height)
			self.camera_pose.publish(self.camera_pose_msg)

		cv2.imshow("image", image)
		key = cv2.waitKey(1)

	def detect_get_pkg_corners(self,image):
		known_colors = {10:[(41, 92, 69) , (85, 255, 255)],11:[(94, 135, 82) , (255, 255, 255)],12:[(151, 83, 194) , (255, 255, 255)]}
		if self.to_be_named_by_dev:
			(corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.arucoDict_package,parameters=self.arucoParams)
			if len(corners)>0:
				for ids_ in ids[0]:
					#print(ids_)
					if ids_ == self.detected_id:
						print('detected_id',ids_)
						return True , corners , 0.058
					else:
						if self.detected_id in [10,11,12]:
							hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
							lower = known_colors[self.detected_id][0]  #(45, 82, 176)    # lower value of color id  
							upper = known_colors[self.detected_id][1]   # (132, 185, 255)
							mask = cv2.inRange(hsv,lower,upper)
							# mask = cv2.dilate(mask,None,iterations=2)
							mask = cv2.erode(mask, None,iterations=1)
							cnts,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
							
							for i,c in enumerate(cnts):
								if len(c)> 50 : # calibrate the values
									epsilon = 0.1*cv2.arcLength(c,True)
									approx = cv2.approxPolyDP(c,epsilon,True)
									approx = [i[0] for i in approx]
									approx = approx[::-1]

									for i in approx:
										if i[0]<5 or i[0]>635 or i[1]<5 or i[1]>475:
											return False, None, 0

									if len(approx) == 4:
										corners_new = tuple(np.array([[np.concatenate(([approx[3]],approx[0:3]))]],dtype=np.float32))
										cv2.imshow('temp__',mask)
										return True , corners_new , 0.07
						return False, None, 0

			elif self.detected_id in [10,11,12]:
				hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
				lower = known_colors[self.detected_id][0]  #(45, 82, 176)    # lower value of color id  
				upper = known_colors[self.detected_id][1]   # (132, 185, 255)
				mask = cv2.inRange(hsv,lower,upper)
				# mask = cv2.dilate(mask,None,iterations=2)
				mask = cv2.erode(mask, None,iterations=1)
				cnts,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
				
				for i,c in enumerate(cnts):
					if len(c)> 50 : # calibrate the values
						epsilon = 0.1*cv2.arcLength(c,True)
						approx = cv2.approxPolyDP(c,epsilon,True)
						approx = [i[0] for i in approx]
						approx = approx[::-1]

						for i in approx:
							if i[0]<5 or i[0]>635 or i[1]<5 or i[1]>475:
								return False, None, 0

						if len(approx) == 4:
							corners_new = tuple(np.array([[np.concatenate(([approx[3]],approx[0:3]))]],dtype=np.float32))
							cv2.imshow('temp__',mask)
							return True , corners_new , 0.07

				return False, None , 0
			
			else:
				return False, None , 0


		else:
			(corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.arucoDict_package,parameters=self.arucoParams)
			if len(corners)>0:
				print(ids[0])
				for ids_ in ids[0]:
					if ids_ in [0,1,2]:
						self.detected_id = ids_
						print('detected_id_color',ids_)
						self.to_be_named_by_dev = True
						return True , corners , 0.058
					else:
						return False, None , 0
			else:
				for color_id in known_colors:
					hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
					lower = known_colors[color_id][0]   # (45, 82, 176)  
					upper = known_colors[color_id][1]   # (132, 185, 255)
					# (color_id, lower, upper)
					mask = cv2.inRange(hsv,lower,upper)
					# mask = cv2.dilate(mask,None,iterations=2)
					mask = cv2.erode(mask, None,iterations=1)
					cnts,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
					
					for i,c in enumerate(cnts):
						if len(c)> 50 : # calibrate the values
							epsilon = 0.1*cv2.arcLength(c,True)
							approx = cv2.approxPolyDP(c,epsilon,True)
							approx = [i[0] for i in approx]
							approx = approx[::-1]
							for i in approx:
								if i[0]<100 or i[0]>540 or i[1]<100 or i[1]>380:
									return False, None, 0
							if len(approx) == 4:
								corners_new = tuple(np.array([[np.concatenate(([approx[3]],approx[0:3]))]],dtype=np.float32))
								cv2.imshow('temp__',mask)
								self.detected_id = color_id
								print('detected_id',color_id)
								self.to_be_named_by_dev = True

								return True , corners_new , 0.07

				return False, None , 0



	def __del__(self):   # distructor
		del self.local_pos_estimate
		del self.flags_status
		del self.pkg_coord
		del self.yaw_angle
		self.shm.close()
		self.shm_flags.close()
		self.shm_pkg_coord.close()
		self.shm_yaw_angle.close()
		print('closed')

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
