from dronekit import connect, VehicleMode
import time


connection_string = "/dev/ttyUSB0"
#connection_string ="127.0.0.1:14550"
vehicle = connect(connection_string, wait_ready=True, baud=57600)

def arm_and_takeoff(aTargetAltitude):

    print ("Arming motors")
    vehicle.mode= VehicleMode("GUIDED")
    vehicle.armed= True
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) 

    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt<=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
vehicle.parameters.set("WPNAV_SPEED_UP",10)
arm_and_takeoff(6)
time.sleep(2)
vehicle.mode = VehicleMode("LAND")
time.sleep(5)
print( "Global Location: %s" % vehicle.location.global_frame)
print( "Velocity: %s" % vehicle.velocity)
print( "GPS: %s" % vehicle.gps_0)
print ("Groundspeed: %s" % vehicle.groundspeed)
print ("Airspeed: %s" % vehicle.airspeed)
print ("Battery: %s" % vehicle.battery)
