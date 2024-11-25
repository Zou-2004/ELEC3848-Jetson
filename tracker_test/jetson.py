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

def run_main_loop(root, visualizer, comm, detect_plane, detect_light):
    detect_light_flag=False     
    tracker_flag=True           
    detect_plane_flag=False     
    is_holding_flag=False       
    Finish_flag=False
    TOF_THRESHOLD_MM = 300
    
    state = "IDLE"
    visualizer.update_state(state)
    state_opmode_map = { "IDLE": 0, "FOLLOW": 2, "CV": 1, "GRAB": 1, "BACKTOLINE": 1, "HOMING": 2 }
    update_t = time.time()

    while root.winfo_exists():  # Check if window is still open
        current_distance = comm.sensor_info.distance_mm
        detect_light.set_distance(current_distance)
        detect_light.set_angleZ(comm.sensor_info.angleZ)
        
        visualizer.update_state(state)
        comm.chassis_cmd.op_mode = state_opmode_map[state]
        
        vx, vy, wz = 0, 0, 0
        ra_op_mode = 0
        ra_height = 0
        ra_angle = 0

        if state == "IDLE":
            if visualizer.is_running():
                print("###")
                print("start running")
                print("###")
                ra_op_mode = 0
                state = "FOLLOW"
                detect_plane.start()
                
        # Your existing state machine code...
        # (Keep all the elif blocks the same)

        opmode=get_opmode(detect_light_flag,tracker_flag,detect_plane_flag)
        comm.chassis_cmd.op_mode = int(opmode)
        comm.chassis_cmd.vx = int(vx)
        comm.chassis_cmd.vy = int(vy)
        comm.chassis_cmd.wz = int(wz)
        comm.send_chassis()

        comm.roboarm_cmd.op_mode = ra_op_mode
        comm.roboarm_cmd.height = ra_height
        comm.roboarm_cmd.angle = ra_angle
        comm.send_roboarm()

        sleep_t = update_t + 0.1 - time.time()
        if (sleep_t > 0):
            time.sleep(sleep_t)
        update_t = time.time()

def main():
    root = tk.Tk()
    visualizer = CarDataVisualizer(root)
    
    ports = Communication.detect_ports()
    port = ports["CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller"]

    comm = Communication(port)
    comm.start()
    visualizer.set_communication(comm)
    
    detect_plane = detectPlane()
    detect_light = MecanumLightSeeker()

    try:
        # Start the main loop in a separate thread
        main_thread = threading.Thread(
            target=run_main_loop,
            args=(root, visualizer, comm, detect_plane, detect_light),
            daemon=True
        )
        main_thread.start()

        # Run GUI in the main thread
        root.mainloop()

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        visualizer.stop_program()
        detect_plane.stop()
        detect_light.stop()
        comm.stop()
        root.quit()

if __name__ == "__main__":
    main()
