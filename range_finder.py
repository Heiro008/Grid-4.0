import RPi.GPIO as GPIO
import time
 from pymavlink import mavutil

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 16
GPIO_ECHO = 24
 
start_time = time.time() 

master = mavutil.mavlink_connection('udpout:0.0.0.0:14595')
print('waiting')
wait_conn()

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
 
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance
 
if __name__ == '__main__':
    try:
        while True:
            #MAV_DISTANCE_SENSOR_ULTRASOUND    1
            #MAV_SENSOR_ROTATION_PITCH_270   25
        
            dist = distance()
            #print ("Measured Distance = %.1f cm" % dist)
            if dist>400:
                dist = 0
            print ("Measured Distance = %.1f cm" % dist)
            time.sleep(0.05)

            master.mav.distance_sensor_send(time_boot_ms=int((time.time()-start_time)*1000),
                min_distance=0,
                max_distance=200,
                current_distance=int(dist),
                type=1,id=0,orientation=25,covariance=255)

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
