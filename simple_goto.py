from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import math
connection_string = "/dev/ttyUSB0"
#connection_string = "127.0.0.1:14550"
iha = connect(connection_string, wait_ready=True, timeout=100, baud=57600)

copter_horizontal_velocity=70
copter_vertical_velocity=70

iha.parameters.set("WP_YAW_BEHAVIOR",0)#bir yere giderken aciyi degistirmiyor.
iha.parameters.set("WPNAV_SPEED",copter_horizontal_velocity)
iha.parameters.set("WPNAV_SPEED_UP",copter_vertical_velocity)


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = iha.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    iha.send_mavlink(msg)
def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;

def arm_ol_ve_yuksel(hedef_yukseklik):
    while iha.is_armable == False:
        print("Arm ici gerekli sartlar saglanamadi.")
        time.sleep(1)
    print("Iha su anda armedilebilir")

    iha.mode = VehicleMode("GUIDED")
    while iha.mode != 'GUIDED':
        print('Guided moduna gecis yapiliyor')
        time.sleep(1.5)

    print("Guided moduna gecis yapildi")
    iha.armed = True
    print("iha arm ediliyor ve 3 sn bekleyiniz...")
    time.sleep(3)
    print("Ihamiz arm olmustur")

    """while iha.armed is False:
        print("Arm icin bekleniliyor")
        time.sleep(1)"""
    iha.simple_takeoff(hedef_yukseklik)
    while iha.location.global_relative_frame.alt <= hedef_yukseklik * 0.99:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
    print("Takeoff gerceklesti")

    print("Global Location (relative altitude): %s" % iha.location.global_relative_frame)
    print("Altitude relative to home_location: %s" % iha.location.global_relative_frame.alt)
    print("Local Location: %s" % iha.location.local_frame)


#simulasyon icin  yonunu cevirdiginde duzeltsin diye
"""arm_ol_ve_yuksel(6)
konum_1=LocationGlobalRelative(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,iha.location.global_relative_frame.alt)
konum_2=LocationGlobalRelative(-35.36291020,149.16513715,6)
heading=get_bearing(konum_1,konum_2)
print (iha.heading)
condition_yaw(iha.heading,False)
print("Heading is : {}".format(heading))
#iha.simple_goto(konum_2)
mesafe=get_distance_metres(konum_1,konum_2)
print ("iki konum arasi mesafe: {}".format(mesafe))
wait_time=math.ceil(mesafe*100/copter_horizontal_velocity) + 3
time.sleep(wait_time)
condition_yaw(heading,False) #false saat yonunun tersine True saat yonunde"""

arm_ol_ve_yuksel(6)
konum_1=LocationGlobalRelative(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,iha.location.global_relative_frame.alt)
konum_2=LocationGlobalRelative(41.129313, 28.997534,8) #orman
mesafe=get_distance_metres(konum_1,konum_2)
print ("iki konum arasi mesafe: {}".format(mesafe))
print ("Belirtilen konuma gidiyorum...")
iha.simple_goto(konum_2)
wait_time=math.ceil(mesafe*100/copter_horizontal_velocity) + 3
time.sleep(wait_time)
print("Belirtilen konuma ulastim.")
iha.mode="LAND"


#konum = LocationGlobalRelative(41.101374, 29.023356,8) #stadyum
#konum = LocationGlobalRelative(41.1031276,29.0320731, 4) #eko yapi
