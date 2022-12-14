# encoding:utf-8
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time
import math
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

yazi = "TEK"
font = 4  # metre
iki_harf_arasi_bosluk = font / 2
delay=2
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_control(array):
    currentLocation = iha.location.global_relative_frame

    if (array[0] != 0 and array[2]==0):
        targetLocation = get_location_metres(currentLocation, 0, array[0])
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        print("Initial distance: ", targetDistance)
        while iha.mode == "GUIDED":
            remainingDistance = get_distance_metres(iha.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance <= targetDistance * 0.08:  # Just below target, in case of undershoot.
                print("Reached target")
                print(iha.location.global_relative_frame)
                break
            time.sleep(2)

    elif (array[0] != 0 and array[2]!=0 ):
        if(array[2]<0):
            print("1\n")
            target_alt=int(iha.location.global_relative_frame.alt)-array[2]
            print("target alt ",target_alt)
            while True:
                remaining_distance= target_alt -int(iha.location.global_relative_frame.alt)
                print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
                time.sleep(1)
                print("remaining distance: {}".format(remaining_distance))
                if(remaining_distance<=1 and remaining_distance>=0):
                    print("Yukselme gerceklesti")
                    break
        else:
            print("2\n")
            target_alt=int(iha.location.global_relative_frame.alt)-array[2]
            while True:
                remaining_distance= int(iha.location.global_relative_frame.alt) - target_alt
                print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
                time.sleep(1)
                print("remaining distance: {}".format(remaining_distance))
                if(remaining_distance<=1):
                    print("Alcalma gerceklesti")
                    break
        print(iha.location.global_relative_frame)
        time.sleep(2)

    elif (array[0] == 0 and array[2]>0): #??u an 10 inece??i 4 var???? 6 alcalma
        target_alt=int(iha.location.global_relative_frame.alt)-array[2] #6 
        print("3\n")
        while True:
            remaining_distance= int(iha.location.global_relative_frame.alt) - target_alt #0a yak??nsar.
            print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
            time.sleep(1)
            print("remaining distance: {}".format(remaining_distance))
            if(remaining_distance<=1):
                print("Alcalma gerceklesti")
                break
        print(iha.location.global_relative_frame)
        time.sleep(2)
        
    elif (array[0] == 0 and array[2]<0): #??u an 10 inece??i 4 var???? 6 
        print("4\n")
        target_alt=int(iha.location.global_relative_frame.alt)-array[2] #6 
        while True:
            remaining_distance= target_alt - int(iha.location.global_relative_frame.alt)#0a yak??nsar.
            print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
            time.sleep(1)
            print("remaining distance: {}".format(remaining_distance))
            if(remaining_distance<=1 and remaining_distance>=0):
                print("Yukselme gerceklesti")
                break
        print(iha.location.global_relative_frame)
        time.sleep(2)

def position_and_nozzle_control(array):  # posx, posy, posz, yaw_rate
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        0b0000011111111000,
        array[1], array[0], array[2],  # pozisyonlar(metre)
        0, 0, 0,  # hizlar(metre/s)
        0, 0, 0,  # akselarasyon(fonksiyonsuz)
        0, math.radians(0))  # yaw,yaw_rate(rad,rad/s)

    iha.send_mavlink(msg)


def nozzle_on_off(nozzle_status):
    # if nozzle_status == 1:
    # iha.parameters["RELAY_PIN"] = 54 # Aux 5
    # print(iha.parameters.get("RELAY_PIN"))

    # elif nozzle_status == 0:
    # iha.parameters["RELAY_PIN"] = -1
    # print(iha.parameters.get("RELAY_PIN"))

    pass


def arm_ol_ve_yuksel(hedef_yukseklik):  # iha ilk harfi yazdirmak icin arm olup 4 metre yukar?? kalkacak.
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
    print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
    print("Takeoff gerceklesti")

arm_ol_ve_yuksel(5)
time.sleep(2)


# Array Values = [x ekseni, y ekseni, z ekseni, i??aret??i]

position_array_T = [[font,0 , 0, 1],
                    [-font / 2, 0, 0, 0],
                    [0, 0, font, 1],
                    [font / 2, 0,-font,0]]

position_array_E = [[iki_harf_arasi_bosluk, 0, 0, 0],  # Harflerin aras??nda bo??luk olmas?? i??in
                    [0, 0, font, 1],
                    [font / 2, 0, 0, 1],
                    [0, 0, -font / 2,0],
                    [-font / 2, 0, 0, 1],
                    [0, 0, -font / 2, 0],
                    [font / 2, 0, 0, 1]]

position_array_K = [[iki_harf_arasi_bosluk, 0, 0, 0],
                    [0, 0, font, 1],
                    [font / 2, 0, 0, 0],
                    [-font / 2, 0, -font / 2, 1],
                    [font / 2, 0, -font / 2, 1]]

position_array_N = [[iki_harf_arasi_bosluk, 0, 0, 0],
                    [0, 0, font,1],
                    [0, 0, -font, 0],
                    [font / 2, 0, font, 1],
                    [0, 0, -font, 1]]

position_array_O = [[iki_harf_arasi_bosluk, 0, 0, 0],
                    [0, 0, font, 1],
                    [font / 2, 0, 0, 1],
                    [0, 0, -font,  1],
                    [-font / 2, 0, 0, 1],
                    [font / 2, 0, 0, 0]]

position_array_F = [[iki_harf_arasi_bosluk, 0, 0, 0],
                    [0, 0, font, 1],
                    [0, 0, -font / 2, 0],
                    [font / 4, 0, 0, 1],
                    [-font / 4, 0, -font / 2, 0],
                    [font / 2, 0, 0, 1]]

position_array_S = [[iki_harf_arasi_bosluk, 0, 0, 0],
                    [0, 0, font, 0],
                    [font / 2, 0, 0, 1],
                    [0, 0, -font / 2, 1],
                    [-font / 2, 0, -0, 1],
                    [0, 0, -font / 2, 1],
                    [font / 2, 0, 0, 1]]

position_array_2 = [[iki_harf_arasi_bosluk, 0, 0, 0],
                    [font / 2, 0, 0, 1],
                    [0, 0, font / 2, 1],
                    [-font / 2, 0, 0, 1],
                    [0, 0, font / 2, 1],
                    [font / 2, 0, 0, 1],
                    [0, 0, -font, 0]]


for i in yazi:
    if i == "T" or i == "t":
        for i in position_array_T:
            nozzle_on_off(i[3])
            position_and_nozzle_control(i)
            distance_control(i)
        print("T yazd??rma i??lemi tamamland??.")

    if i == "E" or i == "e":
        for i in position_array_E:
            nozzle_on_off(i[3])
            position_and_nozzle_control(i)
            distance_control(i)
        print("E yazd??rma i??lemi tamamland??.")

    if i == "K" or i == "k":
        for i in position_array_K:
            nozzle_on_off(i[3])
            position_and_nozzle_control(i)
            distance_control(i)
        print("K yazd??rma i??lemi tamamland??.")

    if i == "N" or i == "n":
        for i in position_array_N:
            nozzle_on_off(i[3])
            position_and_nozzle_control(i)
            distance_control(i)
        print("N yazd??rma i??lemi tamamland??.")

    if i == "O" or i == "o" or i == "0":
        for i in position_array_O:
            nozzle_on_off(i[3])
            position_and_nozzle_control(i)
            distance_control(i)
        print("O yazd??rma i??lemi tamamland??.")

    if i == "F" or i == "f":
        for i in position_array_F:
            nozzle_on_off(i[3])
            position_and_nozzle_control(i)
            distance_control(i)
        print("F yazd??rma i??lemi tamamland??.")

    if i == "S" or i == "s":
        for i in position_array_S:
            nozzle_on_off(i[3])
            position_and_nozzle_control(i)
            distance_control(i)
        print("S yazd??rma i??lemi tamamland??.")

    if i == "2":
        for i in position_array_2:
            nozzle_on_off(i[3])
            position_and_nozzle_control(i)
            distance_control(i)
        print("2 yazd??rma i??lemi tamamland??.")

print('Return to launch')
while (iha.mode!= "RTL"):
    iha.mode = "RTL"
    time.sleep(0.1)

print("Close iha object")
iha.close()

