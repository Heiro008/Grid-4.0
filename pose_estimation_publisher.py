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
import tf
from tf2_ros import TransformBroadcaster

class ImageSubscriber(Node):
	def __init__(self):
		super().__init__('image_subscriber')   # subscriber node name
		#self.subscription = self.create_subscription(CompressedImage, 'camera_image', self.listener_callback, 10)

		self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
		self.subscription

		self.object_pose = self.create_publisher(PoseStamped, '/vision/pose', 1)

        self.camera_pose = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 1)

        self.object_pose_msg = PoseStamped()
        self.camera_pose_msg = PoseStamped()
        self.imgae_header = 0
        self.camera_pose_msg.pose.position.x = 0
        self.camera_pose_msg.pose.position.y = 0
        self.camera_pose_msg.pose.position.z = 0
        self.camera_pose_msg.pose.orientation.x = 0
        self.camera_pose_msg.pose.orientation.y = 0
        self.camera_pose_msg.pose.orientation.z = 0.7071
        self.camera_pose_msg.pose.orientation.w = 0.7071

		self.br = CvBridge()
		self.image_data = None
		self.image_flag = False
		
		self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
		self.arucoParams = cv2.aruco.DetectorParameters_create()

		self.matrix_coefficients = np.array([[1.41286392e+03, 0.00000000e+00 ,6.41248237e+02],
                         [0.00000000e+00 ,1.41389251e+03, 4.26079102e+02],
                         [0.00000000e+00, 0.00000000e+00 ,1.00000000e+00]])

		self.distortion_coefficients = np.array([[-5.90931946e-02,  2.30945178e+00,  1.13648731e-02 ,-9.87470154e-04 ,-9.38992815e+00]])

		self.tag_length = 0.086

	def listener_callback(self, data):
		#self.get_logger().info('Receiving video frame')    # for logging with timestamp
		#print(data)
		#np_arr = np.array(data.data, np.uint8)
		#self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		self.imgae_header = data.header.frame_id
		self.image_data = self.br.imgmsg_to_cv2(data)	
		self.image_flag = True
		self.main_process()
		#cv2.imshow("camera", current_frame)
		#cv2.waitKey(1)
	def main_process(self):

		#frame = self.image_data.copy()
		#self.image_flag = False
		image = cv2.cvtColor(self.image_data,cv2.COLOR_RGB2BGR)
		(corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.arucoDict,parameters=self.arucoParams)

		if len(corners)>0:
			for i in range(0, len(ids)):
            	# Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
				rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.tag_length, self.matrix_coefficients,self.distortion_coefficients)
            	# Draw a square around the markers
				cv2.aruco.drawDetectedMarkers(image, corners) 

				self.object_pose_msg.header.stamp = self.get_clock().now().to_msg()
				self.object_pose_msg.header.frame_id = self.image_header
				self.object_pose_msg.pose.position.x = tvec[0]   #+ offset[ids[i]][0]
				self.object_pose_msg.pose.position.y = tvec[1]   #+ offset[ids[i]][1]
				self.object_pose_msg.pose.position.z = tvec[2]
 				
				rot_mat = cv2.Rodrigues(rvec)
				euler_angles = rotationMatrixToEulerAngles(rot_mat)
				p_quat = Quaternion()

				p_quat = createQuaternionMsgFromRollPitchYaw(euler_angles[0], euler_angles[1], euler_angles[2])

				self.object_pose_msg.pose.orientation = p_quat

				self.object_pose.publish(object_pose_msg)


				#if ids[i] == 2:

				#	print(tvec)
            	# Draw Axis
				cv2.drawFrameAxes(image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)  

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