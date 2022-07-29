
#! /usr/bin/env python
# -*- coding: UTF-8 -*-

from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil
import math


connection_string = "/dev/ttyUSB0"
#connection_string = "127.0.0.1:14550"

iha = connect(connection_string, wait_ready=True, baud=57600)

#yazi = "TEKNOFEST 2022"
yazi = "T"
font = 6  # metre
iki_harf_arasi_bosluk = font / 2
delay = font*3-1
iha.parameters.set("WPNAV_SPEED",50)
iha.parameters.set("WPNAV_SPEED_UP",70)

def position_and_nozzle_control(array): # posx, posy, posz, yaw_rate
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        0b0000011111111000,
        array[1], array[0], array[2],  # pozisyonlar(metre)
        0, 0, 0,  # hizlar(metre/s)
        0, 0, 0,  # akselarasyon(fonksiyonsuz)
        0, math.radians(array[3]))  # yaw,yaw_rate(rad,rad/s)
        
    iha.send_mavlink(msg)

def nozzle_on_off(nozzle_status):
    pass
    #if nozzle_status == 1:
        #iha.parameters["RELAY_PIN"] = 54 # Aux 5
        #print(iha.parameters.get("RELAY_PIN"))

    #elif nozzle_status == 0:
        #iha.parameters["RELAY_PIN"] = -1
        #print(iha.parameters.get("RELAY_PIN"))





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
position_array_T = [[font, 0, 0, 0, 1],
                    [-font / 2, 0, 0, 0, 0],
                    [0, 0, font, 0, 1],
                    [font / 2, 0, -font, 0, 0]]

position_array_E = [[iki_harf_arasi_bosluk, 0, 0, 0, 0],
                    [0, 0, font, 0, 1],
                    [font / 2, 0, 0, 0, 1],
                    [0, 0, -font/2, 0, 0],
                    [-font / 2, 0, 0, 0, 1],
                    [0, 0, -font / 2, 0, 0],
                    [font / 2, 0, 0, 0, 1]]

position_array_K = [[iki_harf_arasi_bosluk, 0, 0, 0, 0],
                    [0, 0, font, 0, 1],
                    [font/2, 0, 0, 0, 0],
                    [-font / 2, 0, -font / 2, 0, 1],
                    [font / 2, 0, -font / 2, 0, 1]]

position_array_N = [[iki_harf_arasi_bosluk, 0, 0, 0, 0],
                    [0, 0, font, 0, 1],
                    [0, 0, -font, 0, 0],
                    [font/2, 0, font, 0, 1],
                    [0, 0, -font, 0, 1]]

position_array_O = [[iki_harf_arasi_bosluk, 0, 0, 0, 0],
                    [0, 0, font, 0, 1],
                    [font/2, 0, 0, 0, 1],
                    [0, 0, -font, 0, 1],
                    [-font/2, 0, 0, 0, 1],
                    [font/2, 0, 0, 0, 0]]

position_array_F = [[iki_harf_arasi_bosluk, 0, 0, 0, 0],
                    [0, 0, font, 0, 1],
                    [0, 0, -font / 2, 0, 0],
                    [font / 4, 0, 0, 0, 1],
                    [-font / 4, 0, -font/2, 0, 0],
                    [font / 2, 0, 0, 0, 1]]


position_array_S = [[iki_harf_arasi_bosluk, 0, 0, 0, 0],
                    [0, 0, font, 0, 0],
                    [font/2, 0, 0, 0, 1],
                    [0, 0, -font / 2, 0, 1],
                    [-font / 2, 0, -0, 0, 1],
                    [0, 0, -font / 2, 0, 1],
                    [font/2, 0, 0, 0, 1]]


position_array_2 = [[iki_harf_arasi_bosluk, 0, 0, 0, 0],
                    [font/2, 0, 0, 0, 1],
                    [0, 0, font / 2, 0, 1],
                    [-font/2, 0, 0, 0, 1],
                    [0, 0, font / 2, 0, 1],
                    [font/2, 0, 0, 0, 1],
                    [0, 0, -font, 0, 0]]

for i in yazi:
    if i == "T" or i == "t":
        for i in position_array_T:
            nozzle_on_off(i[4])
            position_and_nozzle_control(i)
            time.sleep(delay)
        print("T yazdirma islemi tamamlandi.")

    if i == "E" or i == "e":
        for i in position_array_E:
            nozzle_on_off(i[4])
            position_and_nozzle_control(i)
            time.sleep(delay)
        print("E yazdirma islemi tamamlandi.")

    if i == "K" or i == "k":
        for i in position_array_K:
            nozzle_on_off(i[4])
            position_and_nozzle_control(i)
            time.sleep(delay)
        print("K yazdirma islemi tamamlandi.")

    if i == "N" or i == "n":
        for i in position_array_N:
            nozzle_on_off(i[4])
            position_and_nozzle_control(i)
            time.sleep(delay)
        print("N yazdirma islemi tamamlandi.")

    if i == "O" or i == "o" or i == "0":
        for i in position_array_O:
            nozzle_on_off(i[4])
            position_and_nozzle_control(i)
            time.sleep(delay)
        print("O yazdirma islemi tamamlandi.")

    if i == "F" or i == "f":
        for i in position_array_F:
            nozzle_on_off(i[4])
            position_and_nozzle_control(i)
            time.sleep(delay)
        print("F yazdirma islemi tamamlandi.")

    if i == "S" or i == "s":
        for i in position_array_S:
            nozzle_on_off(i[4])
            position_and_nozzle_control(i)
            time.sleep(delay)
        print("S yazdirma islemi tamamlandi.")

    if i == "2":
        for i in position_array_2:
            nozzle_on_off(i[4])
            position_and_nozzle_control(i)
            time.sleep(delay)
        print("2 yazdirma islemi tamamlandi.")

print('Return to launch')
while (iha.mode.name != "LAND"):
    iha.mode = "LAND"
    time.sleep(0.1)

print("Close iha object")
iha.close()


"""
Delay ortadan kaldirilacak, konuma gore kod icerisinde bekleme yapilacak
"""
