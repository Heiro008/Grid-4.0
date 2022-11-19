import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np
from sensor_msgs.msg import CompressedImage

class ImageSubscriber(Node):
	def __init__(self):
		super().__init__('image_subscriber')   # subscriber node name
		#self.subscription = self.create_subscription(CompressedImage, 'camera_image', self.listener_callback, 10)
		self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
		self.subscription
		self.br = CvBridge()
		self.image_data = None
		self.image_flag = False

	def listener_callback(self, data):
		#self.get_logger().info('Receiving video frame')    # for logging with timestamp
		#print(data)
		#np_arr = np.array(data.data, np.uint8)
		#self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		self.image_data = self.br.imgmsg_to_cv2(data)
		self.image_flag = True
		self.main_process()
		#cv2.imshow("camera", current_frame)
		#cv2.waitKey(1)
	def main_process(self):

		#frame = self.image_data.copy()
		#self.image_flag = False
		
		cv2.imshow("image", self.image_data)
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