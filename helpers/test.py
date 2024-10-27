 # coding: utf-8

import serial
import cv2
import numpy as np
import time
import dronekit as dk
from pymavlink import mavutil
from datetime import datetime
import os
import math
#import RPI.GPIO as GPIO


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = dk.VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    time.sleep(8)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

#Note: go to https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD for message references
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame --- CHECK IF WE CHANGE THIS TO MAV_FRAME_BODY_FRD (forward, right down)
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def goto_position_target_local_frd(forward, right, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_FRD, #mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame --- CHECK IF WE CHANGE THIS TO MAV_FRAME_BODY_FRD (forward, right down)
        0b0000111111111000,  # type_mask (only positions enabled)
        forward, right, down,  # x, y, z positions (or Forward, Right, Down in MAV_FRAME_BODY_FRD frame
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

#to initiliaze the variance of image 
def sortSecond(val):       
    return val[1]

# ----------------------------------------------KEEP IT THERE FOR REFERENCE
# connection_string = '/dev/ttyACM0'	#Establishing Connection With Flight Controller
# vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)
# cmds = vehicle.commands
# cmds.download()
# cmds.wait_ready()
# waypoint1 = dk.LocationGlobalRelative(cmds[0].x, cmds[0].y, 3)  # Destination point 1
# ----------------------------------------------

# END of definitions!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# In[3]:

# In[4]:
# camera (input) configuration
# frame_in_w = 640
# frame_in_h = 480

# do need to set becasue for some reason the analog to digital convertor
# randomily set the size base on the computer and changing it will casue to not
# capture

# In[5]:
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")
with open(time_stamp + '_logs.txt', 'a+') as f:
    f.write('FLIGHT TEST: ' + time_stamp)
    f.write('\n')
    f.write('Settings threshold: Lower 45 Upper 50')
    f.write('\n')

cap = cv2.VideoCapture(0)
print("capture device is open: " + str(cap.isOpened()))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'XVID'))
writer = cv2.VideoWriter(time_stamp + '.avi', cv2.VideoWriter_fourcc(*'XVID'), cap.get(cv2.CAP_PROP_FPS), (int(cap.get(3)), int(cap.get(4))))
writer_resframe = cv2.VideoWriter(time_stamp + 'RES.avi', cv2.VideoWriter_fourcc(*'XVID'), cap.get(cv2.CAP_PROP_FPS), (int(cap.get(3)), int(cap.get(4))))
#add another writer later for just fire images
#change the 20 to what the cap_prop_fps is set at
fps = cap.get(cv2.CAP_PROP_FPS)
w1 = cap.get(3)
h1 = cap.get(4)
centerx = w1/2
centery = h1/2
roistartw = cap.get(3) / 10
roistarth = cap.get(4) / 10

lower_red = np.array([0, 100, 100]) #change this to increase the red detection should work for under 15 meters
upper_red = np.array([0, 255, 255])
with open(time_stamp + '_logs.txt', 'a+') as f:
    f.write('Lower Red: ' + str(lower_red) + ' Upper Red: ' + str(upper_red))
    f.write('\n')
    f.write('FPS: ' + str(fps))
    f.write('\n')


# setting up xbee communication
#GPIO.setwarnings(False)
# ser = serial.Serial(

#     port='/dev/ttyUSB0',
#     baudrate=9600,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     bytesize=serial.EIGHTBITS,
#     timeout=1
# )


# INITIALIZING DRONE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
connection_string = '/dev/ttyACM0'  # Establishing Connection With PIXHAWK
vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)  # PIXHAWK is PLUGGED to NUC (RPi too?) VIA USB
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

altitude = 0.01
speed = 1.0

waypoint1 = dk.LocationGlobalRelative(cmds[0].x, cmds[0].y, altitude)


arm_and_takeoff(altitude)
vehicle.airspeed = speed  # set drone speed to be used with simple_goto
vehicle.simple_goto(waypoint1)  # trying to reach 1st waypoint

with open(time_stamp + '_logs.txt', 'a+') as f:
    f.write('Altitude: ' + str(altitude))
    f.write('\n')
    f.write('Airspeed: ' + str(speed))
    f.write('\n')
# time.sleep(30)
# ----------------------------------------------

# Variables for iso-variance
all_var = []
intesnity_total = []
size = []
percentdiff = []
numberofframes = 0

frameCount = 0
fire_found = 0
fire_counter = 15
#change the counter to 15 can change it later
while not fire_found:
    # read next image
    ret, frame = cap.read()
    frame = cv2.medianBlur(frame, 5) #add a image smoother medianBlur at 50%
    writer.write(frame)

    if not ret:
        print('Error Camera not connected')
        with open(time_stamp + '_logs.txt', 'a+') as f:
            f.write('Camera Error : RET returned False')
            f.write('\n')

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    writer_resframe.write(res)

    #display video feeds
    cv2.imshow('Live feed', frame) #raw footage
    cv2.imshow('res', res) #isolated fire

    if cv2.countNonZero(gray) != 0:
        fire_counter = fire_counter - 1

        #Put iso-variance measurement here
        #~~~~~~
        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(gray, None, None, None, 8, cv2.CV_32S)
        print(labels.shape)
        areas = stats[1:, cv2.CC_STAT_AREA]

        img = np.zeros((labels.shape), np.uint8)
        img_f = np.zeros((labels.shape), np.uint8)
        amax = max(areas)
        #print('Max Area: ', amax)
        areas_include = []
        f_k_x_y_images = []
        for i in range(0, nlabels - 1):
            if areas[i] > 40:
                if areas[i] > 0.2*amax:
                    img[labels == i + 1] = 255
                    img_f[labels == i + 1] = 255
                    f_k_x_y_images.append((img_f, areas[i]))
                    img_f = np.zeros((labels.shape), np.uint8)

        f_k_x_y = img


        f_k_x_y_images.sort(key=sortSecond)

        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(img, None, None, None, 8, cv2.CV_32S)

        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) == 0:
            variance_al = 0
            all_var.append(variance_al)

        centroids_index = 1
        contours_index = len(contours) - 1

        while centroids_index != len(centroids):
            xy = contours[contours_index]
            x = [0 for i in range(0, len(xy))]
            y = [0 for i in range(0, len(xy))]
            for i in range(0, len(contours[contours_index])):
                x[i] = xy[i][0][0]
            for i in range(0, len(contours[contours_index])):
                y[i] = xy[i][0][1]

            cx = round(centroids[centroids_index][0])
            cy = round(centroids[centroids_index][1])

            distance = [0 for i in range(0, len(xy))]
            for i in range(0, len(xy)):
                distance[i] = math.sqrt(((x[i] - cx) ** 2) + ((y[i] - cy) ** 2))

            max_distance = max(distance)
            max_location = distance.index(max(distance))

            angle = [0 for i in range(0, len(xy))]  # for now it does not have 0 to 360 degrees
            for i in range(0, len(angle)):
                angle[i] = i

            for i in range(0, len(xy)):
                x[i] = x[i] - cx

            for i in range(0, len(xy)):
                y[i] = y[i] - cy

            angles = [0 for i in range(0, len(xy))]

            for i in range(0, len(angles)):
                if x[i] > 0 and y[i] >= 0:
                    angles[i] = math.atan2(y[i], x[i]) * (180 / math.pi)
                elif x[i] < 0 and y[i] >= 0:
                    angles[i] = math.atan2(y[i], x[i]) * (180 / math.pi)
                elif x[i] < 0 and y[i] < 0:
                    angles[i] = 360 + (math.atan2(y[i], x[i]) * (180 / math.pi))
                elif x[i] > 0 and y[i] < 0:
                    angles[i] = 360 + (math.atan2(y[i], x[i]) * (180 / math.pi))
                elif x[i] == 0 and y[i] >= 0:
                    angles[i] = 90
                elif x[i] == 0 and y[i] < 0:
                    angles[i] = 270

            d = np.array([angles])
            g = angles[max_location]

            for i in range(0, len(angles)):
                angles[i] = angles[i] - g
                if angles[i] < 0:
                    angles[i] = angles[i] + 360

            d = np.array([angles])

            xy_pairs = list(zip(angles, distance))
            xy_pairs.sort()

            new_x = [0 for i in range(0, len(angles))]
            new_y = [0 for i in range(0, len(angles))]
            for i in range(0, len(contours[contours_index])):
                new_x[i] = xy_pairs[i][0]
            for i in range(0, len(contours[contours_index])):
                new_y[i] = xy_pairs[i][1]

            h = [-1/4, 1/2, -1/4]
            g = [1/4, 1/2, 1/4]
            a_l = np.convolve(new_y, h, 'valid')  # high
            d_l = np.convolve(new_y, g, 'valid')  # low
            sumal_noabs = sum(a_l)
            sumdl_noabs = sum(d_l)
            mean_al_noabs = sumal_noabs/len(a_l)
            var_num = 0
            for x in range(0, len(a_l)):
                var_num = var_num + ((a_l[x]-mean_al_noabs) ** 2)
            variance_al = var_num/(len(a_l)-1)
            #print("Mean a[l]:", mean_al_noabs, "Variance a[l]: ", variance_al)

            all_var.append(variance_al)
            print("variance = ",variance_al)

            centroids_index = centroids_index + 1
            contours_index = contours_index - 1
            break

        avg_var =  sum(all_var)/ len(all_var)
        print("average variance = ", avg_var)
        #~~~~~~

        if fire_counter == 0:
            now = datetime.now()
            time_stamp1 = now.strftime("%Y-%m-%d_%H_%M_%S")
            cv2.imwrite(time_stamp + '_original.jpg', frame)
            #cv2.imwrite(time_stamp + '_res.jpg', res)
            #cv2.imwrite(time_stamp + '_gray.jpg', gray)
            cv2.imwrite(time_stamp + '_mask.jpg', mask)
            fire_found = 1

            if avg_var > 3: #Higher variance -> more likely is a moving flame than a static heat source (tweak threshold later)
                            #Variance closer to 0 -> Hot spot or static heat source. Less likely to be flames
                            #Note: Figure out how we want to handle these two scenarios later
                print("FIRE")
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('FIRE AT: ' + time_stamp1)
                    f.write('\n')
                    f.write('Number of Red pixels: ' + str(cv2.countNonZero(gray)))
                    f.write('\n')
                    f.write('Avg. Variance = ' + avg_var)
                    f.write('\n')
            else:
                print("Hot spot detected")
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('Hot spot detected at: ' + time_stamp1)
                    f.write('\n')
                    f.write('Number of Red pixels: ' + str(cv2.countNonZero(gray)))
                    f.write('\n')
                    f.write('Avg. Variance = ' + str(avg_var))
                    f.write('\n')

    if cv2.countNonZero(gray) == 0:
        fire_counter = 10
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting...")
        break
    cv2.waitKey(int((1 / int(fps)) * 1000))
# STOP Flying --------------------------------
send_ned_velocity(0, 0, 0)  # stop the vehicle
# sleepNrecord(2)
time.sleep(3)  # for 3 seconds

# CENTERING -----------------------------------------------------------------------------------------------------------
pixel_len = 0.53714285 #cm and if drone is 8 meter altitutde - Note sure how they got this number
#pixel_len = 0.0012 #Bosun pixel length in cm acccording to camera's data sheet
N = 0
E = 0
flag = 0
while (vehicle.armed and flag != 15):
    ret, frame = cap.read()
    #writer.write(frame)

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(gray, None, None, None, 8, cv2.CV_32S)
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) != 0: #assuming there is only one contour (Changed to if contours isn't empty)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            distance_w = w * pixel_len
            distance_h = h * pixel_len
            with open(time_stamp + '_logs.txt', 'a+') as f:
                f.write('Width: ' + str(distance_w))
                f.write('\n')
                f.write('Length:  ' + str(distance_h))
                f.write('\n')

            cx = round(centroids[1][0])
            cy = round(centroids[1][1])
            line = cv2.line(frame, (round(centerx), round(centery)), (cx, cy), color=(255, 0, 0), thickness=5)
            cv2.imwrite(time_stamp + '_line.jpg', line)
            distance = math.sqrt(((centerx - cx) ** 2) + ((centery - cy) ** 2))
            with open(time_stamp + '_logs.txt', 'a+') as f:
                f.write('Distance: ' + str(distance*pixel_len))
                f.write('\n')

            if (cy < centery and cx < centerx): #I
                #print("I")
                flag = flag + 1
                N = (abs(cy-centery) * pixel_len)/100
                E = -(abs(cx-centerx) * pixel_len)/100
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('N: ' + str(N))
                    f.write('\n')
                    f.write('E:  ' + str(E))
                    f.write('\n')
            elif (cy < centery and cx >= centerx): #II
                #print("II")
                flag = flag + 1
                N = (abs(cy-centery) * pixel_len)/100
                E = (abs(cx-centerx) * pixel_len)/100
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('N: ' + str(N))
                    f.write('\n')
                    f.write('E:  ' + str(E))
                    f.write('\n')
            elif (cy >= centery and cx < centerx): #III
                #print("III")
                flag = flag + 1
                N = -(abs(cy-centery) * pixel_len)/100
                E = -(abs(cx-centerx) * pixel_len)/100
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('N: ' + str(N))
                    f.write('\n')
                    f.write('E:  ' + str(E))
                    f.write('\n')
            elif (cy >= centery and cx >= centerx): #IV
                #print("IV")
                flag = flag + 1
                N = -(abs(cy-centery) * pixel_len)/100
                E = (abs(cx-centerx) * pixel_len)/100
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('N: ' + str(N))
                    f.write('\n')
                    f.write('E:  ' + str(E))
                    f.write('\n')
    cv2.waitKey(int((1 / int(fps)) * 1000))
    cv2.imwrite(time_stamp + '_line.jpg', line)
# CENTERING -----------------------------------------------------------------------------------------------------------

#Move to new location -------------------------------------------------------------------------------------------------

goto_position_target_local_frd(N, E, 0)

send_ned_velocity(0, 0, 0)  # stop the vehicle
time.sleep(3)

#Move to new location -------------------------------------------------------------------------------------------------


# READ CURRENT COORDINATES FROM PIXHAWK-------------------
lat = vehicle.location.global_relative_frame.lat  # get the current latitude
lon = vehicle.location.global_relative_frame.lon  # get the current longitude



coords = "RESCUE " + str(lat) + " " + str(lon)
print(coords)
with open(time_stamp + '_logs.txt', 'a+') as f:
    f.write('Coordinates: ' + coords)
    f.write('\n')
# TRANSMIT CURRENT COORDINATES TO RESCUE DR --------------
# ser.write(coords.encode())

# RETURN HOME CODE ----------------------------
vehicle.mode = dk.VehicleMode("RTL")
# ANOTHER LOOP
while vehicle.armed:
    ret, frame = cap.read()
    #writer.write(frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting...")
        break
    cv2.waitKey(int((1 / int(fps)) * 1000))

# ---------------------------------------------
vehicle.mode = dk.VehicleMode("LAND")
vehicle.flush()


# add to keep recourding??
# or move code to into look and if statement
# and add another flag
