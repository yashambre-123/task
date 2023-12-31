#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import math
import numpy as np
import time
# import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array
from cv_bridge import CvBridge, CvBridgeError

print("start!!")

connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string, baud=921600, wait_ready=False)
vehicle.wait_ready(True, timeout=300)


# changing the default parameters
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1
vehicle.parameters['PLND_EST_TYPE'] = 0
vehicle.parameters['LAND_SPEED'] = 20

bridge = CvBridge()

velocity = 0.5
takeoff_height = 5

# newimg_pub = rospy.Publisher("/iris_demo/usb_cam/image_new", Image, queue_size=10)

id_to_find = 55
marker_size = 100

# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

aruco_type = "DICT_4X4_100"

aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])

parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480
horizontal_fov = 1.047 * (math.pi / 180)
# vertical_fov = 48.8 * (math.pi / 180)
vertical_fov = 3.6 * (math.pi / 180)

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0]
camera_matrix = [[1861.635, 0.0, 648.5], [0.0, 1861.635, 360.5], [0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

# vehicle changes to required modes, takes off to the required height 
def arm_and_takeoff(target_altitude):
    
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Takeoff initiated")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print("Altitude = ", vehicle.location.global_relative_frame.alt)      
        if (vehicle.location.global_relative_frame.alt>=target_altitude): 
            print("Reached the target altitude")
            break
        time.sleep(1)


def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        0, 0, 0, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        vx, vy, vz, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

# function to set the landing position
def send_land_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0,          # time target data was processed, as close to sensor capture as possible
        0,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame, not used
        x,          # X-axis angular offset, in radians
        y,          # Y-axis angular offset, in radians
        0,          # distance, in meters
        0,          # Target x-axis size, in radians
        0          # Target y-axis size, in radians
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def image_callback(ros_image):
    global bridge, id_to_find
    
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        print(e)
    
    gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    ids = ''
    (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

    print(ids)
    
    if ids is not None:
        if (ids[0]==id_to_find):
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
            (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])

            y_sum = 0
            x_sum = 0

            x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
            y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

            x_avg = x_sum / 4
            y_avg = y_sum / 4

            x_ang = (x_avg - horizontal_res*0.5) * horizontal_fov/horizontal_res
            y_ang = (y_avg - vertical_res*0.5) * vertical_fov/vertical_res

            if vehicle.mode!='LAND':
                vehicle.mode = VehicleMode('LAND')
                while vehicle.mode != 'LAND':
                    time.sleep(1)
                print("vehicle in LAND mode")
                send_land_message(x_ang, y_ang)
            else:
                send_land_message(x_ang, y_ang)
            
            # marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z
            marker_position = (x,y)
            aruco.drawDetectedMarkers(cv_image, corners)
            aruco.drawAxis(cv_image, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)

            # cv2.putText(cv_image, str(marker_position), [10, 50], 0, 0.5, (255, 0, 0), thickness=1)
            print(marker_position)
            # print("FOUND COUNT:"+str(found_count)+' not found count:'+str(not_found_count))
            # found_count = found_count + 1
        else:
            print("marker is not the intended one")
    else:
        print("no marker detected")
    
    cv2.imshow("Image Monitor", cv_image)
    

def main():
    print("yash")
    rospy.init_node("task3_node", anonymous=True)
    print("start")
    sub = rospy.Subscriber("/iris_demo/usb_cam/image_raw", Image, image_callback)
    rospy.spin()


if __name__=='__main__':
    try:
        arm_and_takeoff(takeoff_height)
        time.sleep(20)
        print("local ned")
        send_local_ned_velocity(velocity, velocity, velocity)
        time.sleep(1)
        main()
    except rospy.ROSInterruptException:
        pass
