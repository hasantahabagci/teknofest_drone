from dronekit import connect, VehicleMode
import time

connection_string = "/dev/ttyUSB0"
iha = connect(connection_string, wait_ready=True, baud=57600)
iha.parameters.set("WPNAV_SPEED_UP",70)
def arm_ol_ve_yuksel(hedef_yukseklik):
    iha.armed=True
    iha.mode = VehicleMode("GUIDED")
    while iha.mode != 'GUIDED':
        print('Guided moduna gecis yapiliyor')
        time.sleep(1.5)

    print("Guided moduna gecis yapildi")
    iha.armed = True
    while iha.armed is False:
        print("Arm icin bekleniliyor")
        iha.armed = True
        time.sleep(1)
    print("Ihamiz arm olmustur")

    iha.simple_takeoff(hedef_yukseklik)
    while iha.location.global_relative_frame.alt <= hedef_yukseklik * 0.8:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    print("Takeoff gerceklesti")


arm_ol_ve_yuksel(5)
time.sleep(2)
iha.mode = VehicleMode("LAND")
