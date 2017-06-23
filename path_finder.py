# -*- coding: utf-8 -*-
"""
Based on Nikolai K. code

Implementation of Bug Path Finder Algorithm with obstacle avoidance.

Leonardo Claudio de Paula e Silva
Jessica Aissa Bargas
"""

import vrep
import sys
import time
import numpy as np
import math

PI = math.pi

vrep.simxFinish(-1)  # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print 'Connected to remote API server'

else:
    print 'Connection not successful'
    sys.exit('Could not connect')

# handles
errorCode, pioneer_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)
errorCode, target_handle = vrep.simxGetObjectHandle(clientID, 'Target', vrep.simx_opmode_oneshot_wait)

errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                                        vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                                         vrep.simx_opmode_oneshot_wait)
errorCode, gps_pioneer_handle = vrep.simxGetObjectHandle(clientID, 'GPS_Pioneer', vrep.simx_opmode_oneshot_wait)
errorCode, gps_target_handle = vrep.simxGetObjectHandle(clientID, 'GPS_Target', vrep.simx_opmode_oneshot_wait)

# Initialize Position and Orientation
errorCode, pos = vrep.simxGetObjectPosition(clientID, gps_pioneer_handle, -1, vrep.simx_opmode_streaming)
errorCode, pos = vrep.simxGetObjectPosition(clientID, gps_target_handle, -1, vrep.simx_opmode_streaming)
errorCode, pioneer_orientation = vrep.simxGetObjectOrientation(clientID, pioneer_handle, -1, vrep.simx_opmode_streaming )
errorCode, target_orientation = vrep.simxGetObjectOrientation(clientID, target_handle, -1, vrep.simx_opmode_streaming )

# Sensors
sensor_h = []
sensor_val = np.array([])

# orientation of all the sensors:
sensor_loc = np.array(
    [-PI / 2, -50 / 180.0 * PI, -30 / 180.0 * PI, -10 / 180.0 * PI, 10 / 180.0 * PI, 30 / 180.0 * PI, 50 / 180.0 * PI,
     PI / 2, PI / 2, 130 / 180.0 * PI, 150 / 180.0 * PI, 170 / 180.0 * PI, -170 / 180.0 * PI, -150 / 180.0 * PI,
     -130 / 180.0 * PI, -PI / 2])

# Initializing sensors
for x in range(1, 16 + 1):
    errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x),
                                                        vrep.simx_opmode_oneshot_wait)
    sensor_h.append(sensor_handle)  # keep list of handles
    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle, vrep.simx_opmode_streaming)
    sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))  # get list of values

# Main Loop (Simulation)
while True:

    # Update Position, Orientation and Sensors
    errorCode, gps_pioneer_pos = vrep.simxGetObjectPosition(clientID, gps_pioneer_handle, -1, vrep.simx_opmode_buffer)
    errorCode, gps_target_pos = vrep.simxGetObjectPosition(clientID, gps_target_handle, -1, vrep.simx_opmode_buffer)
    errorCode, pioneer_orientation = vrep.simxGetObjectOrientation(clientID, pioneer_handle, -1,
                                                                   vrep.simx_opmode_buffer)
    errorCode, target_orientation = vrep.simxGetObjectOrientation(clientID, target_handle, -1, vrep.simx_opmode_buffer)
    pioneer_orientation = [x * (180.0 / np.pi) for x in pioneer_orientation]
    target_orientation = [x * (180.0 / np.pi) for x in target_orientation]

    sensor_val = np.array([])
    for x in range(1, 16 + 1):
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_h[x - 1], vrep.simx_opmode_buffer)
        sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))  # get list of values

    # controller specific
    sensor_sq = sensor_val[0:8] * sensor_val[0:8]  # square the values of front-facing sensors 1-8

    min_ind = np.where(sensor_sq == np.min(sensor_sq))
    try:
        min_ind = min_ind[0][0]
    except:
        min_ind = 0

    # Continue if target has not been reached
    if abs(abs(gps_target_pos[0]) - abs(gps_pioneer_pos[0])) > 0.1 or abs(
                    abs(gps_target_pos[1]) - abs(gps_pioneer_pos[1])) > 0.1:

        if sensor_sq[min_ind] < 0.15:
            steer = -1 / sensor_loc[min_ind]

            v = 1  # forward velocity
            kp = 0.5  # steering gain
            vl = v + kp * steer
            vr = v - kp * steer
            print "V_l =", vl
            print "V_r =", vr
        else:
            dx = gps_target_pos[0] - gps_pioneer_pos[0]
            dy = gps_target_pos[1] - gps_pioneer_pos[1]

            phi = np.arctan2(dy, dx)
            theta = max(phi * (180.0 / np.pi), pioneer_orientation[2]) - min(phi * (180.0 / np.pi),
                                                                             pioneer_orientation[2])
            v_des = 0.1
            om_des = 0.05 * theta
            d = 0.038  # wheel separation
            r_w = 0.09  # wheel radius

            v_right = v_des + d * om_des
            v_left = v_des - d * om_des
            vl = v_right / r_w
            vr = v_left / r_w

        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, vl, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, vr, vrep.simx_opmode_streaming)
    else:
        # Reached target. Stop Motors
        print("Reached Target!! =D")
        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.0, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.0, vrep.simx_opmode_streaming)

    time.sleep(0.2)

# Post Allocation
errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)

