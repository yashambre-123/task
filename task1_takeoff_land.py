#!/usr/bin/env python3
from dronekit import connect, VehicleMode
import time


# Connecting to the Vehicle
connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string, baud=921600, wait_ready=True)

# function to takeoff
# change the mode to GUIDED and then arm, finally takeoff 
def arm_and_takeoff(aTargetAltitude):

  while not vehicle.is_armable:
    print("Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

  print("Takeoff initiated")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Function to check that vehicle has reached takeoff altitude or not 
  while True:
    print("Altitude = ", vehicle.location.global_relative_frame.alt)      
    if (vehicle.location.global_relative_frame.alt>=aTargetAltitude): 
      print("Reached the target altitude")
      break
    time.sleep(1)


# takeoff to a height of 10m
arm_and_takeoff(10)
print("Take off complete")

# Hover for 10 seconds
print("Hovering for 10 seconds")
time.sleep(10)

print("Landing Initiated")
vehicle.mode = VehicleMode("LAND")
print("Landed Successfully")

# Close vehicle object
vehicle.close()