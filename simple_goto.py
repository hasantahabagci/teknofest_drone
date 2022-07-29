from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

connection_string = "/dev/ttyUSB0"

iha = connect(connection_string, wait_ready=True, timeout=100, baud=57600)


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
    while iha.armed is False:
        print("Arm icin bekleniliyor")
        time.sleep(1)

    print("Ihamiz arm olmustur")

    iha.simple_takeoff(hedef_yukseklik)
    while iha.location.global_relative_frame.alt <= hedef_yukseklik * 0.99:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    print("Takeoff gerceklesti")
arm_ol_ve_yuksel(6)
time.sleep(5)
iha.parameters.set("WPNAV_SPEED",100)
konum = LocationGlobalRelative(41.103204, 29.020609, 8)
#konum = LocationGlobalRelative(41.1031276,29.0320731, 4) #eko yapi
iha.simple_goto(konum)



# LocationGlobal,LocationGlobalRelative
