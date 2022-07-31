from dronekit import connect, VehicleMode
import time
#connection_string = "127.0.0.1:14550"
connection_string = "/dev/ttyUSB0"
iha = connect(connection_string, wait_ready=True, baud=57600)
copter_horizontal_velocity=70
copter_vertical_velocity=70
iha.parameters.set("WPNAV_ACCEL_Z",iha.parameters.get("WPNAV_ACCEL_Z"))
iha.parameters.set("WP_YAW_BEHAVIOR",0)#bir yere giderken aciyi degistirmiyor.
iha.parameters.set("WPNAV_SPEED",copter_horizontal_velocity)
iha.parameters.set("WPNAV_SPEED_UP",copter_vertical_velocity)

print("iha yatay hizi:{} iha kalkis hizi: {}".format(copter_horizontal_velocity,copter_vertical_velocity))

def arm_ol_ve_yuksel(hedef_yukseklik):
    # denemek icin koydum olmazsa asagidakilari yorum satirina al
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
    #print("iha arm ediliyor ve 3 sn bekleyiniz...")
    #time.sleep(3)

    #sadece denemek icin koydum olmazsa yukaridakilari yorum satirindan cikar bunu yoruma al direk
    while iha.armed is False:
        time.sleep(1)
        print("Arm icin bekleniliyor")
        iha.armed = True
        print("iha arm edildi.")
        a=+1
        if(a>5):
            break
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

arm_ol_ve_yuksel(5)
time.sleep(2)
iha.mode = VehicleMode("LAND")
