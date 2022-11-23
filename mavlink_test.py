from pymavlink import mavutil
import time
# Create the connection
master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

boot_time = time.time()

def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
    	print("Channel does not exist.")
    	return


    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.
	#master.mav.rc_channels_raw_send(int(1e3 * (time.time() - boot_time)),0,1500,1500,pwm,1500,1500,1500,1500,1500,254)
def select_mode(mode):
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)
    mode_id = master.mode_mapping()[mode]

    master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

def mannual_control(pitch,roll,thrust,yaw):
    master.mav.manual_control_send(
        master.target_system,
        pitch,
        roll,
        thrust,
        yaw,
        0)

select_mode('ALT_HOLD')
master.arducopter_arm()
# Set some roll
#set_rc_channel_pwm(2, 1600)

# Set some yaw
#set_rc_channel_pwm(4, 1600)

#set mode to alt_hold

t = time.time()

alt = 0
while True:
    try:
        alt = master.recv_match(type='AHRS2',blocking=False).to_dict()['altitude']
        print(alt)
    except:
        pass

    #set_rc_channel_pwm(3, 1600)
    '''
    if(time.time()-t > 50000):
        set_rc_channel_pwm(3, 1500)
    else:
        set_rc_channel_pwm(3, 1600)
    '''
    if alt < 587:
        master.mav.manual_control_send(
        master.target_system,
        0,
        0,
        800,
        0,
        0)
    else:
        master.mav.manual_control_send(
        master.target_system,
        0,
        0,
        500,
        0,
        0)
        
    time.sleep(0.1)

