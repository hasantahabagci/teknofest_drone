from dronekit import Command, connect, VehicleMode
import time
from pymavlink import mavutil
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


def takeoff(irtifa):
    while iha.is_armable is not True:
        print("iHA arm edilebilir durumda degil.")
        time.sleep(1)
    print("iHA arm edilebilir.")

    iha.mode = VehicleMode("GUIDED")
    while iha.mode != 'GUIDED':
        print('Guided moduna gecis yapiliyor')
        time.sleep(1.5)

    iha.armed = True
    while iha.armed is not True:
        print("iHA arm ediliyor...")
        time.sleep(0.5)

    print("iHA arm edildi.")
    iha.simple_takeoff(irtifa)

    while iha.location.global_relative_frame.alt < irtifa * 0.9:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        print("iha hedefe yukseliyor.")
        time.sleep(1)
    print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
    print("Takeoff gerceklesti")
#orman rastgele 3 konum
#1 : 41.129641, 28.997460   3-1:7 metre
#2 : 41.129564, 28.997433   1-2:9 metre
#3 : 41.129605, 28.997528   2-3:9 metre

def gorev_ekle():
    global komut
    komut = iha.commands
    # ilk iki parametre plane icin. toplam 9 deger olacak.
    # send command to vehicle
    time=3
    # TAKEOFF
    komut.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                0, 0, 0, 0, 10))
    #waypoint orman

    komut.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, time,
                0, 0, 0, 41.129641, 28.997460, 10))
    komut.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, time,
                0, 0, 0, 41.129564, 28.997433, 10))
    komut.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, time,
                0, 0, 0, 41.129605, 28.997528, 10))

    # RTL
    komut.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                0, 0, 0, 0, 0, 0, 0, 0))

    # DOgRULAMA
    komut.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                0, 0, 0, 0, 0, 0, 0, 0))

    komut.upload()
    print("Komutlar yukleniyor...")


takeoff(3)

gorev_ekle()

komut.next = 0

iha.mode = VehicleMode("AUTO")

while True:
    next_waypoint = komut.next

    print("Siradaki komut {}".format(next_waypoint))
    time.sleep(1)

    if next_waypoint is iha.commands.count-1:
        print("Gorev bitti.")
        break

print("Donguden cikildi.")

