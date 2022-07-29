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
    while iha.location.global_relative_frame.alt <= hedef_yukseklik * 0.8:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    print("Takeoff gerceklesti")





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
# 2 meters right with timesleep
def basic_movements(x,y): #sadece bir degiskene 0 dan farkli gir.
    if (x!=0):
        print("{} metre saga gidiyorum...".format(x)) #x ekseni dronun onu
        basic_position(x, 0, 0)
        time.sleep(5)
    else:
        print("{} metre saga gidiyorum...".format(y))
        basic_position(0, y, 0)
        time.sleep(5)

def t_yaz():
    print("T yaziyorum")
    basic_position(font,0,0)
    time.sleep(5)
    basic_position(-font/2,0,0)
    time.sleep(5)
    basic_position(0,0,font)
    time.sleep(5)
    basic_position(font/2,0,-font)
    time.sleep(5)


position_array_T = [[font, 0, 0, 0, 1],
                    [-font / 2, 0, 0, 0, 0],
                    [0, 0, font, 0, 1],
                    [font / 2, 0, -font, 0, 0]]



iha.parameters.set("WPNAV_SPEED",70)
iha.parameters.set("WPNAV_SPEED_UP",70)
arm_ol_ve_yuksel(6)
time.sleep(delay)
basic_movements(0,5) #x ve y yi not al 
#t_yaz()
iha.mode="RTL"







