from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil
import math
#connection_string = "127.0.0.1:14550"
connection_string = "/dev/ttyUSB0"
iha = connect(connection_string,wait_ready=True, timeout=100, baud=57600)

copter_horizontal_velocity=70
copter_vertical_velocity=70
iha.parameters.set("WP_YAW_BEHAVIOR",0)#bir yere giderken aciyi degistirmiyor.
iha.parameters.set("WPNAV_SPEED",copter_horizontal_velocity)
iha.parameters.set("WPNAV_SPEED_UP",copter_vertical_velocity)
print("iha yatay hizi:{} iha kalkis hizi: {}".format(copter_horizontal_velocity,copter_vertical_velocity))

font=4
def arm_ol_ve_yuksel(hedef_yukseklik):
    iha.mode = VehicleMode("GUIDED")
    while iha.mode != 'GUIDED':
        print('Guided moduna gecis yapiliyor')
        time.sleep(1.5)
    print("Guided moduna gecis yapildi")
    iha.armed = True
    print("iha arm ediliyor ve 3 sn bekleyiniz...")
    time.sleep(3)
    print("Ihamiz arm olmustur")
    iha.simple_takeoff(hedef_yukseklik)
    while iha.location.global_relative_frame.alt <= hedef_yukseklik * 0.8:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
    print("Takeoff gerceklesti")

    print("Global Location (relative altitude): %s" % iha.location.global_relative_frame)
    print("Altitude relative to home_location: %s" % iha.location.global_relative_frame.alt)
    print("Local Location: %s" % iha.location.local_frame)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def basic_position(x, y, z):  # posx, posy, posz, yaw_rate
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        0b0000011111111000,
        x, y, z,  # pozisyonlar(metre)
        0, 0, 0,  # hizlar(metre/s)
        0, 0, 0,  # akselarasyon(fonksiyonsuz)
        0, 0) # yaw,yaw_rate(rad,rad/s)

    iha.send_mavlink(msg)
# x=5 ise 500cm demek hiz 70misse min 7sn hata payi ile 10sn demek

def basic_movements(x,y): #sadece bir degiskene 0 dan farkli gir.
    konum_1=iha.location.global_relative_frame
    if (x!=0):
        if(x<0):
            wait_time=math.ceil( (x*-1*100)/copter_horizontal_velocity)+3
            print("Wait time = {}".format(wait_time))
            print("{} metre geri gidiyorum...".format(x)) #x ekseni dronun onu
        else:
            wait_time = math.ceil( (x * 100) / copter_horizontal_velocity) + 3
            print("{} metre one gidiyorum...".format(x))
        basic_position(x, 0, 0)
        time.sleep(wait_time)
        konum_2 = iha.location.global_relative_frame
        mesafe=get_distance_metres(konum_2,konum_1)

        if (x < 0):
            print("{} metre geri gittim.".format(mesafe))
        else:
            print("{} metre one gittim.".format(mesafe))
    else:
        if(y<0):
            wait_time = math.ceil((y * -1 * 100) / copter_horizontal_velocity) + 3
            print("{} metre sola gidiyorum...".format(y))
        else:
            wait_time = math.ceil((y * 100) / copter_horizontal_velocity) + 3
            print("{} metre saga gidiyorum...".format(y))
        basic_position(0, y, 0)
        time.sleep(wait_time)
        konum_2 = iha.location.global_relative_frame
        mesafe = get_distance_metres(konum_2, konum_1)
        if (y < 0):
            print("{} metre sola gittim.".format(mesafe))
        else:
            print("{} metre saga gittim.".format(mesafe))
    print("Global Location (relative altitude): %s" % iha.location.global_relative_frame)
    print("Altitude relative to home_location: %s" % iha.location.global_relative_frame.alt)
    print("Local Location: %s" % iha.location.local_frame)

def t_yaz():
    konum_1 = iha.location.global_relative_frame
    print("T yaziyorum")

    basic_position(0,font,0)
    wait_time = math.ceil((font * 100) / copter_horizontal_velocity) + 3
    print("{} metre saga gidiyorum...".format(font))
    print("{} saniye bekleyecegim.".format(wait_time))
    time.sleep(wait_time)
    konum_2 = iha.location.global_relative_frame
    mesafe = get_distance_metres(konum_2, konum_1)
    print("{} metre saga gittim.".format(mesafe))
    print("Global Location (relative altitude): %s" % iha.location.global_relative_frame)
    print("Altitude relative to home_location: %s" % iha.location.global_relative_frame.alt)
    print("Local Location: %s" % iha.location.local_frame)

    konum_1 = iha.location.global_relative_frame
    basic_position(0,-font/2,0)
    wait_time = math.ceil((font * 50) / copter_horizontal_velocity) + 3
    print("{} metre sola gidiyorum...".format(font/2))
    print("{} saniye bekleyecegim.".format(wait_time))
    time.sleep(wait_time)
    konum_2 = iha.location.global_relative_frame
    mesafe = get_distance_metres(konum_2, konum_1)
    print("{} metre sola gittim.".format(mesafe))
    print("Global Location (relative altitude): %s" % iha.location.global_relative_frame)
    print("Altitude relative to home_location: %s" % iha.location.global_relative_frame.alt)
    print("Local Location: %s" % iha.location.local_frame)


    alt_1=iha.location.global_relative_frame.alt
    basic_position(0,0,font)
    wait_time = math.ceil((font * 100) / copter_horizontal_velocity) + 3
    print("{} metre asagi gidiyorum...".format(font))
    print("{} saniye bekleyecegim.".format(wait_time))
    time.sleep(wait_time)
    alt_2=iha.location.global_relative_frame.alt
    mesafe=alt_1-alt_2
    print("{} metre asagi gittim.".format(mesafe))
    print("Global Location (relative altitude): %s" % iha.location.global_relative_frame)
    print("Altitude relative to home_location: %s" % iha.location.global_relative_frame.alt)
    print("Local Location: %s" % iha.location.local_frame)

    alt_1=iha.location.global_relative_frame.alt
    konum_1 = iha.location.global_relative_frame
    basic_position(0,font/2,-font)
    wait_time = math.ceil((font * 100) / copter_vertical_velocity) + 3
    print("{} metre saga {} metre yukari gidiyorum...".format(font/2,font))
    print("{} saniye bekleyecegim.".format(wait_time))
    time.sleep(wait_time)
    konum_2 = iha.location.global_relative_frame
    mesafe=get_distance_metres(konum_2, konum_1)
    alt_2 = iha.location.global_relative_frame.alt
    alt_distance=alt_2-alt_1
    print("{} metre saga {} metre yukari gittim.".format(mesafe,alt_distance))
    print("Global Location (relative altitude): %s" % iha.location.global_relative_frame)
    print("Altitude relative to home_location: %s" % iha.location.global_relative_frame.alt)
    print("Local Location: %s" % iha.location.local_frame)

    print("T YAZDIM :))) ")

arm_ol_ve_yuksel(6)
time.sleep(2)
#basic_movements(3,0)  #x dronun onu arkasi y dronun sagi solu
t_yaz()
iha.mode="LAND"







