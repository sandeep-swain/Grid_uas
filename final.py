from Grid_Detection.trackdet import grid
from Path_planning.Astar import astar
from Aruko_Detection import aruco
from Comms.socket_server import input_commands
from Comms.Motor_pwm_PID import PID_Control
from Comms.Multibot_connection import connect_bots
# from Grid_Detection.select_coord import select_coord
import cv2
import numpy as np
import socket
import time
import json
import math

path = r'C:\Users\japni\Desktop\Grid_UAS\Grid_Detection\PICs\check_image_10.jpg'
# waypoints_dict = {0:{"waypoint1":(135,111),"waypoint2":(122,67),"waypoint3":(89,258),"waypoint4":(517,250),"waypoint5":(549,87)},1:{"waypoint1":(133,161),"waypoint2":(55,51),"waypoint3":(90,314),"waypoint4":(517,250),"waypoint5":(551,159)}}
waypoints_dict = {0:{"waypoint1":(138,133),"waypoint2":(113,97),"waypoint3":(143,257),"waypoint4":(535, 270),"waypoint5":(540,130)},1:{"waypoint1":(98,203),"waypoint2":(73,14),"waypoint3":(136,333),"waypoint4":(595,305),"waypoint5":(545,195)}}

sequence = 0
distance_to_target = 10000000
desired_angle = 0
curr_orientation = 0
error = 0
error_integral = 0
q = []
error_list = []
save = False
start1 = True
dest = (0,0)
aruco_ids = [0,2,3]
bot_clients = connect_bots(len(aruco_ids))
initial_orientation = 0
initial_orientation1 = 0
start_check = True
start_check1 = True
cam = 1
bot_return = False
vid = cv2.VideoCapture(2)  # Starting point camera
vid1 = cv2.VideoCapture(4)  # d1 d2 camera
# vid2 = cv2.VideoCapture(4) # d3 d4 camera

tracking = False
tracker_locked = False
start = time.time()
stop = False

index = 0
while (True):


    # frame = cv2.imread(path)
    # frame1 = cv2.imread(path)

    ret, frame = vid.read()
    # cv2.imshow("frame", frame)
    #
    ret1, frame1 = vid1.read()
    # cv2.imshow("frame1", frame1)
    # print(frame.shape)

    # ret2,frame2 = vid2.read()
    # cv2.imshow("frame2",frame2)
    time_now = time.time()

    # aruko detection
    detected, center_det, orientation, bbox_det = aruco.track(frame,aruco_ids[index])
    detected1, center_det1, orientation1, bbox_det1 = aruco.track(frame1,aruco_ids[index])
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
        initial_orientation1 = ((initial_orientation * -1) - 90) * -1
        print("Initial orientation camera 2 = ", initial_orientation1)
        start_check1 = False

    ######SETTING DESTINATION POINTS BASED ON WHICH CAMERA'S DETCTION IS ONLINE##########
    if detected and cam == 1:
        stop = False
        if (sequence == 0):
            dest = waypoints_dict[index]["waypoint1"]
        elif (sequence == 1):
            dest = waypoints_dict[index]["waypoint2"]
        elif sequence == 4:
            dest = waypoints_dict[index]["waypoint1"]
            bot_return = True
        elif sequence == 5:
            dest = waypoints_dict[index]["waypoint5"]
        elif sequence == 6:
            stop = True
            print("Bot1 Reached S1")
        aruco_center = center_det
        curr_orientation = orientation
        if (curr_orientation <= 180 - initial_orientation and curr_orientation >= -180 - initial_orientation):
            curr_orientation += initial_orientation
        else:
            curr_orientation += 360 + initial_orientation
    elif cam == 1:
        stop = True
        print("[DETECTION LOST] Camera 1 unable to detect aruco")

    if detected1 and cam == 2:
        stop = False
        if (sequence == 2):
            dest = waypoints_dict[index]["waypoint3"]
        elif (sequence == 3):
            dest = waypoints_dict[index]["waypoint4"]
        aruco_center = center_det1
        curr_orientation = orientation1
        if (curr_orientation <= 180 - initial_orientation1 and curr_orientation >= -180 - initial_orientation1):
            curr_orientation += initial_orientation1
        else:
            curr_orientation += 360 + initial_orientation1

        if(curr_orientation>=-90):
            curr_orientation-=90
        else:
            curr_orientation+=270

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
        error = y - x
        if (abs(error) > 180):
            if error > 0:
                error = -(360 - abs(error))
            else:
                error = 360 - abs(error)
        ###################################################

        if "error_past" in locals():
            de = (error - error_past) / (time_now - time_past)
            error_integral += (time_now - time_past) * error_past
            pwm = PID_Control(error, de, index)
        else:
            pwm = PID_Control(error, 0, index)

        # print('PWM:- ', pwm)
        commands = 'F' + pwm
        error_past = error

        #######COMMANDS TO EXECUTE WHEN BOT IS NEAR DESTINATION FOR SELECTED CAM###########
        if distance_to_target < 30:
            print("REACHED WAYPOINT " + str(sequence + 1))
            #####SEND STOP COMMAND TO BOT FOR 0.1 SECONDS###
            if sequence == 2:
                start = time.time()
                end = time.time()
                while (end - start < 2):
                    commands = 'S0,0:'
                    input_commands(commands, client)
                    end = time.time()
            ##############################################
            sequence += 1
            if (sequence == 0 or sequence == 1 or sequence == 4 or sequence == 5):
                cam = 1
            elif (sequence == 2 or sequence == 3):
                cam = 2
            elif (sequence == 6):
                if index == len(aruco_ids) :
                    print("All Bots Reached")
                    break
                else:
                    sequence = 0
                    commands = 'S0,0:'
                    input_commands(commands, client)
                    index += 1
                    start = time.time()
                    start_check = True
                    start_check1 = True
                    error_integral = 0
                    del error_past

        #####################################################################################

    ################################################################################################

    print("Using Camera " + str(cam))
    print("Distance to target = " + str(distance_to_target))
    print("Sending command - " + str(commands))
    print("Desired angle = " + str(desired_angle))
    print("Initial orientation camera 1 = ", initial_orientation)
    print("Initial orientation camera 2 = ",initial_orientation1)
    print("Current orientation = " + str(curr_orientation))
    print("Error = " + str(error))
    # commands = 'S0,0:'
    client = bot_clients[index]
    input_commands(commands, client)
    end = time.time()
    fps = 1.0 / (end - start)
    start = end
    # print("fps:  ", fps)
    cv2.imshow('aruco_frame', frame)
    cv2.imshow("aruco_frame1", frame1)
    # cv2.imshow("aruco_frame1", frame2)
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
vid.release()
vid1.release()
# Destroy all the windows
cv2.destroyAllWindows()
client.close()