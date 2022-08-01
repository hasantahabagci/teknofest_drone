from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import math
#connection_string = "127.0.0.1:14550"
#connection_string = "/dev/ttyUSB0"
connection_string = "/dev/ttyACM0"
iha = connect(connection_string,wait_ready=True, timeout=100, baud=57600)

copter_horizontal_velocity=70
copter_vertical_velocity=70
copter_vertical2_velocity=70 #inis
iha.parameters.set("WP_YAW_BEHAVIOR",0)#bir yere giderken aciyi degistirmiyor.
iha.parameters.set("WPNAV_SPEED",copter_horizontal_velocity)
iha.parameters.set("WPNAV_SPEED_UP",copter_vertical_velocity)
iha.parameters.set("WPNAV_SPEED_DN",copter_vertical2_velocity)
print("iha yatay hizi:{} iha kalkis hizi: {}".format(copter_horizontal_velocity,copter_vertical_velocity))


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

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

arm_ol_ve_yuksel(4)
konum_1=LocationGlobalRelative(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,iha.location.global_relative_frame.alt)
konum_2=LocationGlobalRelative(41.101579, 29.023312,8) #baslangic
konum_3=LocationGlobalRelative(41.10157899999016, 29.0233596157126,8) #4 metre sag
konum_4=LocationGlobalRelative(41.1015789999877, 29.023335807856302,8)#2 metre sol
konum_5=LocationGlobalRelative(41.1015789999877, 29.023335807856302,4) #4 metre asagi
konum_6=LocationGlobalRelative(41.10157899999016, 29.0233596157126,8) #4 metre sag olan konuma gitme
konum_7=LocationGlobalRelative(41.1015789999877, 29.023383423568898,8) #2 metre bosluk ve diger harf

print("T CIZMEYE BASLIYORUM")

mesafe=get_distance_metres(konum_1,konum_2)
print ("iki konum arasi mesafe: {}".format(mesafe))
print ("Belirtilen konuma gidiyorum...")
iha.simple_goto(konum_2)
wait_time=math.ceil(mesafe*100/copter_horizontal_velocity) + 3
time.sleep(wait_time)
print("Belirtilen konuma ulastim.")

mesafe=get_distance_metres(konum_2,konum_3)
print ("iki konum arasi mesafe: {}".format(mesafe))
print ("Belirtilen konuma gidiyorum...")
iha.simple_goto(konum_3)
wait_time=math.ceil(mesafe*100/copter_horizontal_velocity) + 3
time.sleep(wait_time)
print("Belirtilen konuma ulastim.")

mesafe=get_distance_metres(konum_3,konum_4)
print ("iki konum arasi mesafe: {}".format(mesafe))
print ("Belirtilen konuma gidiyorum...")
iha.simple_goto(konum_4)
wait_time=math.ceil(mesafe*100/copter_horizontal_velocity) + 3
time.sleep(wait_time)
print("Belirtilen konuma ulastim.")

mesafe=get_distance_metres(konum_4,konum_5)
print ("iki konum arasi mesafe: {}".format(mesafe))
print ("Belirtilen konuma gidiyorum...")
iha.simple_goto(konum_5)
wait_time=math.ceil(mesafe*100/copter_horizontal_velocity) + 3
time.sleep(wait_time)
print("Belirtilen konuma ulastim.")

mesafe=get_distance_metres(konum_5,konum_6)
print ("iki konum arasi mesafe: {}".format(mesafe))
print ("Belirtilen konuma gidiyorum...")
iha.simple_goto(konum_6)
wait_time=math.ceil(mesafe*100/copter_horizontal_velocity) + 3
time.sleep(wait_time)
print("Belirtilen konuma ulastim.")

mesafe=get_distance_metres(konum_6,konum_7)
print ("iki konum arasi mesafe: {}".format(mesafe))
print ("Belirtilen konuma gidiyorum...")
iha.simple_goto(konum_7)
wait_time=math.ceil(mesafe*100/copter_horizontal_velocity) + 3
time.sleep(wait_time)
print("Belirtilen konuma ulastim.")

print("T CIZMEYI BITIRDIM INISE GECEIYORUM..")
iha.mode="LAND"
