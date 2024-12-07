from Grid_Detection.trackdet import grid
from Path_planning.Astar import astar
from Aruko_Detection import aruco
from Comms.socket_server import input_commands
from Comms.Motor_pwm_PID import PID_Control
from Comms.connection import client_return
import cv2
import numpy as np
import socket
import time
import json
import math

# path = "/home/shubho/Grid_UAS/Grid_Detection/PICs/check_image_10.jpg"
sequence = 0
client = client_return()
distance_to_target = 0
desired_angle = 0
curr_orientation = 0
error = 0
error_integral = 0
q = []
error_list = []
start_check = 0
save = False
start1 = True


def getAngle(a, b, c):
    ang = math.degrees(math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(a[1] - b[1], a[0] - b[0]))
    return ang


def save_json(data, save):
    if save:
        with open('data.json', 'w') as f:
            json.dump(data, f)


def drawBox(frame, bbox):
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5)


# error_list.append(0)
cam = 1
vid = cv2.VideoCapture(2)  # Starting point camera
vid1 = cv2.VideoCapture(4)  # Turning point camera
# vid2 = cv2.VideoCapture(8) #Destination camera
# vid3 = cv2.VideoCapture(9)

# vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
tracking = False
tracker_locked = False
start_check = True
start_check1 = True
start = time.time()
stop = False
while (True):

    # frame = cv2.imread(path)
    ret, frame = vid.read()
    # cv2.imshow("frame", frame)

    ret1, frame1 = vid1.read()
    # cv2.imshow("frame1", frame1)
    # print(frame.shape)

    # ret2,frame2 = vid2.read()
    # cv2.imshow("frame2",frame2)
    time_now = time.time()

    # aruko detection
    detected, center_det, orientation, bbox_det = aruco.track(frame,5)
    detected1, center_det1, orientation1, bbox_det1 = aruco.track(frame1,5)

    print(aruco.track(frame,2))
    # detected2, center_det2, orientation2, bbox_det2 = aruco.track(frame2)
    if detected and start_check and cam == 1:
        initial_orientation = orientation * -1
        print("Initial orientation camera 1 = ", initial_orientation)
        start_check = False

    # if detected and bot_return and cam == 1:
    #     initial_orientation = orientation * -1
    #     print("Initial orientation camera 1 = ", initial_orientation)
    #     bot_return = False

    if detected1 and start_check1  and cam == 2:
        initial_orientation1 = ((initial_orientation * -1)) * -1
        print("Initial orientation camera 2 = ", initial_orientation1)
        start_check1 = False


    ######SETTING DESTINATION POINTS BASED ON WHICH CAMERA'S DETCTION IS ONLINE##########
    if detected and cam == 1:
        stop = False
        if sequence == 0:
            dest = (118, 319)
        elif sequence == 1:
            dest = (111, 405)
        elif sequence == 4:
            dest = (226, 294)
        elif sequence == 5:
            dest = (530, 319)
        aruco_center = center_det
        curr_orientation = orientation
        if (curr_orientation <= 180 - initial_orientation and curr_orientation >= -180 - initial_orientation):
            curr_orientation += initial_orientation
        else:
            curr_orientation += 360 - initial_orientation

        '''if(abs(curr_orientation) > 180):
            if(curr_orientation > 0):
                curr_orientation = - ( 360 - abs(curr_orientation) )
            else:
                curr_orientation = 360 - abs(curr_orientation)'''
    elif cam == 1:
        stop = True
        print("[DETECTION LOST] Camera 1 unable to detect aruco")

    if detected1 and cam == 2:
        stop = False
        if sequence == 2:
            dest = (79, 257)
        elif sequence == 3:
            dest = (583, 251)
        aruco_center = center_det1
        curr_orientation = orientation1
        if (curr_orientation <= 180 - initial_orientation1 and curr_orientation >= -180 - initial_orientation1):
            curr_orientation += initial_orientation1
        else:
            curr_orientation -= 360 - initial_orientation1

    elif cam == 2:
        stop = True
        print("[DETECTION LOST] Camera 2 unable to detect aruco")

    ##############################################################################

    ########## STOP BOT WITHOUT CALCULATING PWM IF STOP VARIABLE IS TRUE##########
    if stop:
        commands = 'S0,0:'
    ##############################################################################

    #############DECIDING ERROR AND PWM BASED ON DESTINATION AND CURRENT COORDINATES###########################
    else:
        distance_to_target = math.sqrt((aruco_center[0] - dest[0]) ** 2 + (aruco_center[1] - dest[1]) ** 2)
        desired_angle = math.atan2(aruco_center[1] - dest[1], aruco_center[0] - dest[0])
        desired_angle *= 57.2958
        y = desired_angle
        x = curr_orientation

        error = y-x
        if (abs(error) > 180):
            if error > 0:
                error = -(360 - abs(error))
            else:
                error = 360 - abs(error)

        # STOP AND TURN BOT IF ERROR GOES BEYOND 45 DEGREES
        ###################################################
        # print(error)
        # print("Desired - ", desired_angle * 57.2958)
        # print("Orientation - ", orientation)

        if "error_past" in locals():
            de = (error - error_past) / (time_now - time_past)
            error_integral += (time_now - time_past) * error_past
            pwm = PID_Control(error, de, 0)
        else:
            pwm = PID_Control(error, 0, 0)

        commands = 'F' + pwm
        error_past = error

        #######COMMANDS TO EXECUTE WHEN BOT IS NEAR DESTINATION FOR SELECTED CAM###########
        if distance_to_target < 50:
            print("REACHED WAYPOINT " + str(sequence + 1))
            start = time.time()
            end = time.time()
            #####SEND STOP COMMAND TO BOT FOR 2 SECONDS###
            '''while (end - start < 0.1):
                commands = 'F-10,-10:'
                input_commands(commands, client)
                end = time.time()'''
            ##############################################
            if sequence==3:
                while(end-start > 0.001):
                    commands = 'S0,0:'
                    input_commands(commands,client)
                    end = time.time()
                while(end - start > 0.00001):
                    commands = 'X0,0:'
                    input_commands(commands,client)
                    end = time.time()

            sequence += 1
            if (sequence == 0 or sequence == 1 or sequence == 4 or sequence == 5):
                cam = 1
            else:
                cam = 2
        #####################################################################################

    ################################################################################################

    print("Using Camera " + str(cam))
    # print("Distance to target = " + str(distance_to_target))
    print("Sending command - " + str(commands))
    print("Desired angle = " + str(desired_angle))
    print("Current orientation = " + str(curr_orientation))
    print("Error = " + str(error))
    # commands = 'S0,0:'
    input_commands(commands, client)
    end = time.time()
    fps = 1.0 / (end - start)
    start = end
    # print("fps:  ", fps)
    cv2.imshow('aruco_frame', frame)
    cv2.imshow("aruco_frame1", frame1)
    time_past = time_now

    if cv2.waitKey(1) & 0xFF == ord('q'):
        start = time.time()
        end = time.time()
        #####SEND STOP COMMAND TO BOT FOR 2 SECONDS###
        while (end - start < 0.01):
            commands = 'S0,0:'
            input_commands(commands, client)
            end = time.time()
        ##############################################
        client.close()
        break

# After the loop release the cap object
# vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
# client.close()