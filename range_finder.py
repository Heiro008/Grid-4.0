import RPi.GPIO as GPIO
import time
from pymavlink import mavutil
import rclpy
from std_msgs.msg import Float32
from rclpy.node import Node

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 16
GPIO_ECHO = 24
GPIO_MAGNET = 21

start_time = time.time()

def wait_conn():
    """
    Sends a ping to the autopilot to stabilish the UDP connection and waits for a reply
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

master = mavutil.mavlink_connection('udpout:0.0.0.0:14595')
print('waiting')
wait_conn()

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_MAGNET, GPIO.OUT)
print('on')
GPIO.output(GPIO_MAGNET, True)



def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time()

    temp_time = time.time()
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
        if StartTime-temp_time > 1:
            error = True
            print('error')
            break

    temp_time = time.time()
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
        if StopTime-temp_time > 1:
            error = True
            print('error')
            break


    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance
 


class range_finder(Node):
    def __init__(self):
        super().__init__('range_finder')
        self.range_pub = self.create_publisher(Float32,"/range_finder/ultrasonic",10)
        self.msg = Float32()
        self.prev_dist = 0
        self.count = 0
        self.dist = 0
        #self.create_timer(0.05, self.callback)
        try:
            while True:
                self.dist = abs(distance())
                print ("Measured raw Distance = %.1f cm" % self.dist)
                if self.dist>400:
                    self.dist = self.prev_dist
                if abs(self.dist-self.prev_dist)>30 and self.count<7:
                    self.dist = self.prev_dist
                    self.count+=1
                else:
                    self.count = 0
                    self.prev_dist = self.dist
                print ("Measured Distance = %.1f cm" % self.dist)
                time.sleep(0.05)    
                self.msg.data = float(self.dist)

                self.range_pub.publish(self.msg)

                master.mav.distance_sensor_send(time_boot_ms=int((time.time()-start_time)*1000),
                    min_distance=0,
                    max_distance=200,
                    current_distance=int(self.dist),
                    type=1,id=1,orientation=25,covariance=255)
        except KeyboardInterrupt:

            GPIO.output(GPIO_MAGNET, False)
            print('off')
            print("Measurement stopped by User")
            GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    range_finder_ = range_finder()
    rclpy.spin(range_finder_)
    range_finder_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






# if __name__ == '__main__':
#     try:
#         prev_dist = 0
#         count = 0
#         while True:
#             #MAV_DISTANCE_SENSOR_ULTRASOUND    1
#             #MAV_SENSOR_ROTATION_PITCH_270   25

#             dist = abs(distance())
#             #print ("Measured Distance = %.1f cm" % dist)
#             if dist>400:
#                 dist = 1
#             if abs(dist-prev_dist)>40 and count<5:
#                 dist = prev_dist
#                 count+=1
#             else:
#                 count = 0
#             print ("Measured Distance = %.1f cm" % dist)
#             time.sleep(0.05)
#             master.mav.distance_sensor_send(time_boot_ms=int((time.time()-start_time)*1000),
#                 min_distance=0,
#                 max_distance=200,
#                 current_distance=int(dist),
#                 type=1,id=1,orientation=25,covariance=255)

#     # Reset by pressing CTRL + C
#     except KeyboardInterrupt:
#         print("Measurement stopped by User")
#         GPIO.cleanup()
