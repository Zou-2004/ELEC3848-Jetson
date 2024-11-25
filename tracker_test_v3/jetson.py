import cv2
import threading 
import numpy as np
import serial
import time
from communication import Communication
import math
from detectnet import detectPlane
import argparse
import sys
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, Log
from light_seeker import MecanumLightSeeker
from car_visualizer import CarDataVisualizer
import tkinter as tk


def get_opmode(detect_light_flag,tracker_flag,detect_plane_flag):
    ret = 0   #disabled
    if tracker_flag:
        ret = 2 #tracker
    elif (detect_light_flag or detect_plane_flag):
        ret = 1  #direct
    return ret  
        
def main():
    root=tk.Tk()
    visualizer=CarDataVisualizer()
    detect_light_flag=False     #detect light from landing gear
    tracker_flag=True           #tracker the line
    detect_plane_flag=False     #detect the plane
    is_holding_flag=False       #whether robort arm is holding the airpl

    Finish_flag=False


    TOF_THRESHOLD_MM = 300
    
    
    ports = Communication.detect_ports()
    port = ports["CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller"]

    comm = Communication(port)
    comm.start()
    visualizer.set_communication(comm)
    state = "IDLE"
    visualizer.update_state(state)
    detect_plane = detectPlane()

    detect_light=MecanumLightSeeker()

    state_opmode_map = { "IDLE": 0, "FOLLOW": 2, "CV": 1, "GRAB": 1, "BACKTOLINE": 1, "HOMING": 2 }

    update_t = time.time()

    try:
        visualizer_thread=threading.Thread(target=lambda:root.mainloop())
        visualizer_thread.start()
        while True:
            current_distance = comm.sensor_info.distance_mm
            detect_light.set_distance(current_distance)
            detect_light.set_angleZ(comm.sensor_info.angleZ)
            #light_seeker = MecanumLightSeeker()
            #line trackingcv2.VideoCapture(0)
            visualizer.update_state(state)
            # if xy[0] == None and communication.sensorInfo.is_holding==False and detect_plane_flag==False:
            comm.chassis_cmd.op_mode = state_opmode_map[state]
            
            vx, vy, wz = 0, 0, 0

            ra_op_mode = 0
            ra_height = 0
            ra_angle = 0

            if state == "IDLE":
                #IF UI button is  
                ra_op_mode = 0
                if True:
                    state = "FOLLOW"
                    detect_plane.start()
                    
            elif state == "FOLLOW":
                ra_op_mode = 1
                print("no airplane founded")

                tracker_flag=True
                detect_light_flag=False
                
                vx = 40
                
                if detect_plane.get_found():
                    detect_plane.stop()
                    detect_light.start()
                    
                    print("Airplane detected, heading to the airplane")
                    state = "CV"

            # elif xy[0] != 0 and communication.sensorInfo.is_holding == False and communication.sensorInfo.distance_mm>=1000:
            elif state == "CV":
                ra_op_mode = 2
                ra_angle = 0
                ra_height = 148
 

                detect_light.set_distance(comm.sensor_info.distance_mm)
                detect_light.set_angleZ(comm.sensor_info.angleX)    # tbd

                detect_plane_flag = True
                tracker_flag = False

                mvvct = detect_light.get_vector()
                print("mvvct =", mvvct)
                if mvvct is not None:
                    vx = mvvct[0] * 200.0
                    vy = mvvct[1] * 200.0
                    wz = mvvct[2] * 120.0
                else:
                    vx, vy, wz = 0, 0, 0

                print(f"TOF = {comm.sensor_info.distance_mm} mm")
                if comm.sensor_info.distance_mm < TOF_THRESHOLD_MM:
                    print("start grabbing!!")
                    state = "GRAB"

            elif state == "GRAB":
                ra_op_mode = (1 << 7) | (2)
                ra_angle = 0
                ra_height = 148

                tracker_flag=False
                detect_light_flag= True
                
                # if detect_light_flag:
                #     vx,vy,wz=detect_light.get_vector()
                vx, vy, wz = 0, 0, 0

                if comm.sensor_info.is_holding:
                    detect_light.stop()
                    print("Finish grabbing. back to the line!")
                    state = "BACKTOLINE"
                
            elif state == "BACKTOLINE":
                ra_op_mode = (1 << 7) | (2)
                ra_angle = 0
                ra_height = 148

                print("successfully hold the airplane, return back")
                detect_plane.stop()
                detect_light_flag=False
                tracker_flag=False
                if comm.chassis_info.follow_state != 1:
                    # tracker_flag=True
                    print("Successfully bck to the line, back to home!")
                    state = "HOMING"
                
            elif state == "HOMING":
                ra_op_mode = (1 << 7) | (2)
                ra_angle = 0
                ra_height = 148
 
                tracker_flag=True
                detect_light_flag=False
                if (comm.chassis_info.finish_state):
                    ra_op_mode = 1
                    state = "IDLE"
            
            opmode=get_opmode(detect_light_flag,tracker_flag,detect_plane_flag)
            print("CHASSIS SEND:", opmode, vx, vy, wz)
            comm.chassis_cmd.op_mode = int(opmode)
            comm.chassis_cmd.vx = int(vx)
            comm.chassis_cmd.vy = int(vy)
            comm.chassis_cmd.wz = int(wz)
            comm.send_chassis()

            # print(comm.chassis_cmd.op_mode, comm.chassis_cmd.vx, comm.chassis_cmd.vy, comm.chassis_cmd.wz)
            comm.roboarm_cmd.op_mode = ra_op_mode
            comm.roboarm_cmd.height = ra_height
            comm.roboarm_cmd.angle = ra_angle
            comm.send_roboarm()
            print("ROBOARM SEND:", ra_op_mode, ra_height, ra_angle)

            sleep_t = update_t + 0.1 - time.time()
            if (sleep_t > 0):
                time.sleep(sleep_t)
            update_t = time.time()

    except KeyboardInterrupt:
        visualizer.stop_program()
        detect_plane.stop()
        detect_light.stop()
        comm.stop()



if __name__ == "__main__":
    main()
