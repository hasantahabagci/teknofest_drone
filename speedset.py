from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil

connection_string = "/dev/ttyUSB0"
iha = connect(connection_string, wait_ready=True, timeout=100, baud=57600)



def takeoff(irtifa):
    while iha.is_armable is not True:
        print("iHA arm edilebilir durumda degil.")
        time.sleep(1)
    print("iHA arm edilebilir.")

    iha.mode = VehicleMode("GUIDED")

    iha.armed = True

    while iha.armed is not True:
        print("iHA arm ediliyor...")
        time.sleep(0.5)

    print("iHA arm edildi.")

    iha.simple_takeoff(irtifa)

    while iha.location.global_relative_frame.alt < irtifa * 0.9:
        print("iha hedefe yukseliyor.")
        time.sleep(1)

iha.parameters.set("WPNAV_SPEED_UP",1000)
def gorev_ekle():
    global komut
    komut = iha.commands
    # send command to vehicl

    # TAKEOFF
    komut.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                0, 0, 0, 0, 6))

    # WAYPOINT
    komut.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
                0, 0, 0, 41.103204, 29.020609, 10))

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

    if next_waypoint is 4:
        print("Gorev bitti.")
        break

print("Donguden cikildi.")

