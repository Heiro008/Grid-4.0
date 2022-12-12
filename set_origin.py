#!/usr/bin/env python

##
#
# Send SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION messages
#
##
import rclpy
from rclpy.node import Node

from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from mavros.mavlink import convert_to_rosmsg

from pymavlink import mavutil

from mavros_msgs.msg import Mavlink
import time
# Global position of the origin
lat = 110200163  # Coimbatore
lon = 770039725  # Coimbatore
alt = 0 


def wait_conn():
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


master = mavutil.mavlink_connection('udpout:192.168.75.218:14590')
#master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
print('waiting')
#master.wait_heartbeat()
wait_conn()


class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def send_message(msg, mav, pub):
    """
    Send a mavlink message
    """
    msg.pack(mav)
    rosmsg = convert_to_rosmsg(msg)
    pub.publish(rosmsg)

    print("sent message %s" % msg)

def set_global_origin(mav, pub):
    """
    Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
    to use local position information without a GPS.
    """
    target_system = mav.srcSystem
    #target_system = 0   # 0 --> broadcast to everyone
    lattitude = lat
    longitude = lon
    altitude = alt

    msg = MAV_APM.MAVLink_set_gps_global_origin_message(
            target_system,
            lattitude, 
            longitude,
            altitude)

    master.mav.set_gps_global_origin_send(target_system,
            lattitude, 
            longitude,  
            altitude)
    print('sent origin')

    #send_message(msg, mav, pub)

def set_home_position(mav, pub):
    """
    Send a mavlink SET_HOME_POSITION message, which should allow
    us to use local position information without a GPS
    """
    target_system = mav.srcSystem
    #target_system = 0  # broadcast to everyone

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

    msg = MAV_APM.MAVLink_set_home_position_message(
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
    #send_message(msg, mav, pub)


class set_origin(Node):
    def __init__(self):
        super().__init__('origin_publisher')
        self.mavlink_pub = self.create_publisher(Mavlink, '/mavlink/to', 20)
        self.f = fifo()
        self.mav = MAV_APM.MAVLink(self.f, srcSystem=1, srcComponent=1)

        while self.mavlink_pub.get_subscription_count() <= 0:
            #print(self.mavlink_pub.get_subscription_count())
            pass

        for _ in range(2):
            time.sleep(2)
            set_global_origin(self.mav, self.mavlink_pub)
            set_home_position(self.mav, self.mavlink_pub)

f = fifo()

mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

if __name__=="__main__":

    # args=None
    # rclpy.init(args=args)
    
    # origin_publisher = set_origin()
    # rclpy.spin(origin_publisher)
    # origin_publisher.destroy_node()
    # rclpy.shutdown()
    for _ in range(2):
        time.sleep(2)
        set_global_origin(mav,None)
        set_home_position(mav,None)