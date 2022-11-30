import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import time
from imutils.video import VideoStream
from cv_bridge import CvBridge

import numpy as np
class Image_publisher(Node):
	def __init__(self):
		super().__init__('camera_node')
		#self.publisher = self.create_publisher(CompressedImage,'camera_image',10)
		self.publisher = self.create_publisher(Image,'camera_image',10)

		#self.cap = VideoStream(0,resolution=(720,1280),framerate=30).start()
		self.cap = cv2.VideoCapture("rtsp://192.168.216.218:8554/test/?tcp")
		self.br = CvBridge()
		#self.cap.set(cv2.CAP_PROP_FOURCC, 0x47504A4D)
		#self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
		#self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
		#self.msg = CompressedImage()
		while True:
			ret , frame = self.cap.read()
			frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
			if ret:
				#print(self.get_clock().now())
				#self.msg.header.stamp = self.get_clock().now().to_msg()
				#print(self.msg.header.stamp)
				#self.msg.format = "jpeg"
				#self.msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
				image = self.br.cv2_to_imgmsg(frame,encoding="rgb8")
				self.publisher.publish(image)
				#time.sleep(0.02)
				#cv2.imshow('tmp',frame)
				#if cv2.waitKey(1) == 27:
				#	break
			
		self.cap.release()
		cv2.destroyAllWindows()


def main():
	rclpy.init(args=None)
	image_publisher = Image_publisher()

	rclpy.spin(image_publisher)
	image_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
  main()