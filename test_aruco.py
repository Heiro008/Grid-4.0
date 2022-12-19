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
		
		self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 1)
		self.subscription
		self.range_finder = self.create_subscription(Float32,'/range_finder/ultrasonic', self.update_height, 10)
		self.range_finder

		self.object_pose = self.create_publisher(PoseStamped, '/vision/pose', 1)

		self.set_point = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 1)

		self.camera_pose = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 1)

		#self.create_timer(1/30, self.callback)
		self.height = 0
		self.object_pose_msg = PoseStamped()
		self.camera_pose_msg = PoseStamped()
		self.set_point_msg = PoseStamped()
		self.imgae_header = 0
		self.camera_pose_msg.pose.position.x = 0.1
		self.camera_pose_msg.pose.position.y = -0.15
		self.camera_pose_msg.pose.position.z = 0.0
		self.camera_pose_msg.pose.orientation.x = 0.0
		self.camera_pose_msg.pose.orientation.y = 0.0
		self.camera_pose_msg.pose.orientation.z = 0.7071 
		self.camera_pose_msg.pose.orientation.w = 0.7071

		self.set_point_msg.pose.position.x = 0.0
		self.set_point_msg.pose.position.y = 0.0
		self.set_point_msg.pose.position.z = 1.0

		self.br = CvBridge()
		self.image_data = None
		self.image_flag = False
		
		self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
		self.arucoParams = cv2.aruco.DetectorParameters_create()

		###########################################
		self.matrix_coefficients = np.array([[1.26415545e+03, 0.00000000e+00, 6.14268000e+02],
									[0.00000000e+00, 1.26790106e+03, 5.07574844e+02],
									[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

		self.distortion_coefficients = np.array([[ 0.24030483, -0.75567233,  0.00286373, -0.00462205, -0.65268243]])
		###########################################
		self.tag_length = 0.055  # in metres, length of one marker on the board
		self.tag_separation = 0.008   # HAVE TO REDECLARE PROPERLY (in metres again, distance between adjacent markers)
		self.board = cv2.aruco.GridBoard_create(4, 3, self.tag_length, self.tag_separation, self.arucoDict)
		        # first number = no. of columns of markers in the board
		        # second number = no. of rows of markers in the board
		
		self.no_of_boards = 6
		self.boards = []
		self.board_details = {}
		for i in range(self.no_of_boards):
			self.boards.append(cv2.aruco.GridBoard_create(4, 3, self.tag_length, self.tag_separation, self.arucoDict, 12*i))
		        	# first number = no. of columns of markers in the board
			        # second number = no. of rows of markers in the board
			        # last number = id of the first marker in the gridboard! very important.
			self.board_details[i] = [[j for j in range(12*i, 12*(i+1))], None, None, None, None]
		
		######## CHECK THESE VALUES ###########################
		self.board_details[0][1] = (0,0)
		self.board_details[1][1] = (119.5/100, 0)
		self.board_details[2][1] = (0, 93.5/100)
		self.board_details[3][1] = (119.5/100, 93.5/100)
		#self.board_details[4][1] = (0, )
		#self.board_details[5][1] = (119.5/100, )
		#######################################################
			
		#self.board_offset = {}
		#self.board_offset[0] = (0, 0)	# (x,y) offset values for board 0. If you consider as (0,0), define all other
						# board offsets from this board.
		#self.board_offset[1] = (30/100, 0)

		#self.board_ids = {0: [i for i in range(0, 11+1)], 1: [i for i in range(12, 23+1)]}
			# add ids present in each board here. the keys start from 0,1,2,3.. and so on
		
		#self.tf_broadcaster = TransformBroadcaster(self)
		
	def update_height(self,data):
		self.height = float(data.data/100)
		#print(self.height)

	def callback(self):

		self.camera_pose_msg_tmp = PoseStamped()
		self.camera_pose_msg_tmp.header.stamp = self.get_clock().now().to_msg()
		self.camera_pose_msg_tmp.header.frame_id = 'world'
		self.camera_pose_msg_tmp.pose.position.x = 1.0
		self.camera_pose_msg_tmp.pose.position.y = 0.0
		self.camera_pose_msg_tmp.pose.position.z = 1.0

		transform = tf.compose_matrix(translate=[-1.0,1.0,1.0],angles=[np.pi,0,0])  # update only the yaw angle 
		inv_transform = tf.inverse_matrix(transform)
		camera_origin = tf.translation_from_matrix(inv_transform)
		camera_quaternion = tf.quaternion_from_matrix(inv_transform)
		self.camera_pose_msg_tmp.pose.position.x = camera_origin[0]   #offset[ids] 
		self.camera_pose_msg_tmp.pose.position.y = camera_origin[1]   #offset[ids]
		self.camera_pose_msg_tmp.pose.position.z = camera_origin[2]
		q_rot = tf.quaternion_from_euler(np.pi,0,-np.pi/2)
		q_new = tf.quaternion_multiply(q_rot,camera_quaternion)
		q_new = tf.unit_vector(q_new)

		self.camera_pose_msg_tmp.pose.orientation.x = -q_new[2]  # y value
		self.camera_pose_msg_tmp.pose.orientation.y = -q_new[1]  # x value
		self.camera_pose_msg_tmp.pose.orientation.z = -q_new[3]  # z value
		self.camera_pose_msg_tmp.pose.orientation.w = q_new[0]  # w value

		self.set_point_msg.header.stamp = self.get_clock().now().to_msg()

		#self.set_point.publish(self.set_point_msg)

		self.camera_pose.publish(self.camera_pose_msg_tmp)

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

		#frame = self.image_data.copy()
		#self.image_flag = False
		#image = cv2.cvtColor(self.image_data,cv2.COLOR_RGB2BGR)
		(corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
		#cv2.aruco.refineDetectedMarkers(image, self.board, corners, ids, rejected)  # not sure what this is for
		       # maybe this above function can be applied only when the board alone is visible in frame?
		       # i think not, but just check once to be sureee

		tvec = None
		rvec = None
		
		
		if len(corners) > 6 :   # or ids!=None
		
			corners_split = [[] for i in range(self.no_of_boards)]  # have to check if datatype matches
				# this will store the corners of each board in the arena, separately.
				# for boards that are not detected in the frame, empty list [] will be stored
			
			ids_split = [[] for i in range(self.no_of_boards)]
			#ids_split_new = [[] for i in range(len(self.board_ids))]
			
			
			#for i in range(len(ids)):    # first, split the corners
			#	for j in range(len(self.board_ids)):
			#		if ids[i] in self.board_ids[j]:
			#			ids_split[j].append([ids[i]])
			#			corners_split[j].append(corners[i])
						
			for i in range(len(ids)):    ###### if ids[i] < 72
				j = ids[i][0] // 12
				ids_split[j].append([ids[i]])
				corners_split[j].append(corners[i])
						
			for i in range(len(ids_split)):
				ids_split[i] = np.reshape(ids_split[i], (len(ids_split[i]), 1))
			
			#i_max = 0
			#ids_split_new[i_max] = ids_split[i_max]
			#for i in range(1, len(ids_split)):
			#	if len(ids_split[i]) > len(ids_split[i_max]):
			#		ids_split_new[i] = ids_split[i]
			#		ids_split_new[i_max] = []
			#		i_max = i
			#	else:
			#		ids_split_new[i] = []
			
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
					#print(i, tvec, tvec_tmp)
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
				
				
				#ret_val_list.append(ret_val)
				#rvec_list.append(rvec)
				#tvec_list.append(tvec)

			#rvec, tvec, markerPoints = cv2.aruco.estimatePoseBoard(corners,ids, self.tag_length, self.matrix_coefficients, self.distortion_coefficients, tvec, rvec)
			if board_selected!=None:
				
				# Draw a square around the markers
				cv2.aruco.drawDetectedMarkers(image, corners_split[board_selected], ids_split[board_selected], (0,255,0)) 
				
				tvec = self.board_details[board_selected][2]
				rvec = self.board_details[board_selected][3]
				
				self.object_pose_msg.header.stamp = self.get_clock().now().to_msg()
				self.object_pose_msg.header.frame_id = 'camera'
				#print(tvec.shape)
				#object_pose_msg_tranform = TransformStamped()
				#object_pose_msg_tranform.header.stamp = self.get_clock().now().to_msg()
				#object_pose_msg_tranform.header.frame_id = 'tf_broadcaster_'
				#print(tvec)
				self.object_pose_msg.header.stamp = self.get_clock().now().to_msg()
				self.object_pose_msg.pose.position.x = float(tvec[0][0])   #+ offset[ids[i]][0]
				self.object_pose_msg.pose.position.y = float(tvec[1][0])   #+ offset[ids[i]][1]
				self.object_pose_msg.pose.position.z = float(tvec[2][0])
				#print(rvec[i][0])
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

				#self.tf_broadcaster.sendTransform(object_pose_msg_tranform)
				#use python transformation library to find the inverse transforms

				#print(tvec)
				transform = tf.compose_matrix(translate=tvec,angles=euler_angles)
				inv_transform = tf.inverse_matrix(transform)
				camera_origin = tf.translation_from_matrix(inv_transform)
				camera_quaternion = tf.quaternion_from_euler(np.pi, 0, euler_angles[2])
				#'sxyz'
				q_rot = tf.quaternion_from_euler(np.pi,0,np.pi*1.5)	
				q_new = tf.quaternion_multiply(q_rot,camera_quaternion)
				q_new = tf.unit_vector(q_new)
				#q_new.normalize()
				self.camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
				self.camera_pose_msg.header.frame_id = 'map'
				camera_origin[0] = camera_origin[0] + self.board_details[board_selected][1][0] 
				camera_origin[1] = camera_origin[1] + self.board_details[board_selected][1][1]
				print(i, camera_origin)
				self.camera_pose_msg.pose.position.x = camera_origin[0]
				self.camera_pose_msg.pose.position.y = camera_origin[1]
				#feeding the range_finder value to the z position
				self.camera_pose_msg.pose.position.z = camera_origin[2]    #float(self.height) 
				self.camera_pose_msg.pose.orientation.x = -q_new[2]  # y value
				self.camera_pose_msg.pose.orientation.y = -q_new[1]  # x value
				self.camera_pose_msg.pose.orientation.z = -q_new[3]  # z value
				self.camera_pose_msg.pose.orientation.w = q_new[0]  # w value
				#self.camera_pose_msg.pose.position = self.object_pose_msg.pose.position
				#print('OBJECT position:', tvec)
				#print('CAMERA position:', camera_origin)
				
				# ------ THIS PART NEEDS EDITING ------ #
				#for i in ids:
					#if i == 3:
						#self.camera_pose.publish(self.camera_pose_msg)
						#print('marker',tvec)
						#print(rvec)
						#print(euler_angles)
						#print('camera',round(camera_origin[0],5), round(camera_origin[1],5), round(camera_origin[2], 5))
				# ------------------------------------ #
				self.camera_pose.publish(self.camera_pose_msg)
			
			#  print(tvec)
        	# Draw Axis
        	
			
		else:

			self.camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
			self.camera_pose_msg.header.frame_id = 'map'
			if self.height < 50:
				self.camera_pose_msg.pose.position.z = float(self.height)
			self.camera_pose.publish(self.camera_pose_msg)
			#print("CAMERA position: ", self.camera_pose_msg)

		cv2.imshow("image", image)
		key = cv2.waitKey(1)


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
