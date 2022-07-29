##########################################################################################
# ilk olarak 6 metre havaya kalkiyor sonra gonderdigimiz hiz degerleriyle t harfini ciziyor.
##########################################################################################

from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil
import math
#connection_string = "127.0.0.1:14550"
connection_string = "/dev/ttyUSB0"
iha = connect(connection_string,wait_ready=True, timeout=100, baud=57600)

font=4
delay=font+0.5
def arm_ol_ve_yuksel(hedef_yukseklik):
    iha.mode = VehicleMode("GUIDED")
    while iha.mode != 'GUIDED':
        print('Guided moduna gecis yapiliyor')
        time.sleep(1.5)

    print("Guided moduna gecis yapildi")
    iha.armed = True
    while iha.armed is False:
        print("Arm icin bekleniliyor")
        time.sleep(1)

    print("Ihamiz arm olmustur")

    iha.simple_takeoff(hedef_yukseklik)
    while iha.location.global_relative_frame.alt <= hedef_yukseklik * 0.99:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    print("Takeoff gerceklesti")

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        iha.send_mavlink(msg)
        time.sleep(1)


arm_ol_ve_yuksel(6)
#kuzey dronun onu olacak ve t yazacak. Global yonlere gore yaziyor.
send_ned_velocity(0,2,0,2)
time.sleep(5)
send_ned_velocity(0,-1,0,2)
time.sleep(5)
send_ned_velocity(0,0,1,2)
time.sleep(5)
send_ned_velocity(0,1,-1,2)
time.sleep(5)
iha.mode="LAND"

