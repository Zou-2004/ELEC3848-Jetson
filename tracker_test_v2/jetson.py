import cv2
import threading 
import numpy as np
import serial
import time
import communication
import math
from detectnet import detectPlane
import argparse
import sys
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, Log

US_OFFSET = 25

"""
#parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, 
                                 epilog=detectNet.Usage() + videoSource.Usage() + videoOutput.Usage() + Log.Usage())

parser.add_argument("input", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

try:
	args = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help
	sys.exit(0)

# create video sources and outputs
input = videoSource(args.input, argv=sys.argv)
output = videoOutput(args.output, argv=sys.argv)
	
# load the object detection network
net = detectNet(args.network, sys.argv, args.threshold)
"""
class Communicate:
    def __init__(self):
        self.ports=communication.Communication.detect_ports()
        self.main_port = self.ports["CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller"]
        self.gyro_port = self.ports["USB Serial"]
        self.comm= communication.Communication(self.main_port, self.gyro_port)
    def initialize_comm(self):
        # Target parameters                                                                                   # detect port
        if lens(self.ports) != 2:
            print("Error: num of ports= ",len(self.ports))
            print(self.ports)
            exit(1)
        main_port = ports["CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller"]
        gyro_port=ports["USB Serial"]
        self.comm = communication.Communication(main_port, gyro_port)
    def send_chassis(self,vx,vy,wz,op_mode):
        self.comm.chassis_cmd.vx = int(vx * 200)
        self.comm.chassis_cmd.vy = int(vy * 200)
        self.comm.chassis_cmd.wz = int(omega * 120)
        self.comm.chassis_cmd.op_mode=op_mode
        self.comm.send()
        
    @staticmethod
    def get_opmode(detetct_light_flag,tracker_flag,detect_plane_flag):
        ret = 0
        if tracker_flag:
            ret = 2;
        elif (detect_light_flag or detect_plane_flag):
            ret = 1
        return ret
        
        
        
    def start(self):
        self.comm.start()

    

class MecanumLightSeeker:
    print_time = 0
    def __init__(self,running):
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        
        time.sleep(2)  
        self.running=running
        # Light detection parameters
        self.light_threshold = 200
        self.min_light_area = 100
        self.max_light_area = 10000
        

        # Target parameters
        self.target_distance = 3.5 + US_OFFSET  # cm
        
        # PID parameters for each movement component
        self.kp_forward = 0.07    # Forward/backward
        self.kp_lateral = 1.5    # Left/right
        self.kp_rotation = 1.5   # Rotation
        
        #self.running=True
        # Movement limits
        # self.max_speed = 255
        # self.min_speed = 50

        #self.initialize_comm()
        time.sleep(1)

    def start(self):
        self.t=threading.Thread(target=self.run)
        self.t.start()
    def stop(self):
        self.t.join()

    """
    def initialize_comm(self):
        # detect port
        ports = communication.Communication.detect_ports()
        if (len(ports) != 2):
            print("ERR: number of ports =", len(ports))
            print(ports)
            exit(1)

        print(ports)
        main_port = ports["CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller"]
        gyro_port = ports["USB Serial"]
        print(main_port, gyro_port)
        self.comm = communication.Communication(main_port, gyro_port)
        self.comm.start()
    """
    def cleanup(self):
        """Clean up resources"""
        self.cap.release()
        cv2.destroyAllWindows()

    def detect_light(self, frame):
        """
        Detect the brightest spot in the frame
        Returns: (x,y) coordinates of brightest spot and its area
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Threshold the image
        _, thresh = cv2.threshold(gray, self.light_threshold, 255, cv2.THRESH_BINARY)
        
        # Find contours
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, 
                                     cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0
        
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.min_light_area:
            return None, 0
            
        # Get the centroid
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None, 0
            
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        return (cx, cy), area

    def get_distance(self):
        """
        Get distance reading from Arduino
        Returns: distance in cm or None if no valid reading
        """

        #ret = self.comm.chassis_info.distance
        if (abs(ret) < 3):
            return None
        return ret

    yahoo_flag = False

    def calculate_movement_vectors(self, frame_width, frame_height, 
                                 light_area, current_distance):
        vx = 0
        vy = 0
        omega = 0

        if light_pos != None:
            center_x_error = light_pos[0] - frame_width // 2
        else:
            center_x_error = 0

            # if (current_distance is not None and current_distance > 20):
            #     omega = -self.kp_rotation * center_x_error / (frame_width // 2)
            #     vy = 0
            # elif current_distance is not None and current_distance > 8:
            #     omega = 0
            #     vy = -self.kp_lateral * center_x_error / (frame_width // 2)
            # else:
            #     omega = 0
            #     vy = 0
        

        distance_error = None
        if (current_distance != None):
            distance_error = current_distance - self.target_distance

        # if (self.dir_flag):
        #     vx, vy= 0, 0
        #     if (abs(self.comm.gyro_info.angleZ) < 10):
        #         self.dir_flag = False
        #         self.continue_flag = False
        #         omega = 0
        #     else:
        #         omega = -self.comm.gyro_info.angleZ / 45.0
            
        # if (selpython run command in parallelf.yonly_flag):
        #     vx, omega = 0, 0
        #     if (abs(center_x_error / (frame_width // 2)) < 0.2 or time.time() - self.yonly_timer > 10):
        #         self.yonly_flag = False
        #         self.y_continue_flag = False
        #         vy = 0
        #     else:
        #         vy = self.kp_lateral * center_x_error / (frame_width // 2)

        if (distance_error == None):
            vx, vy, omega = 0, 0, 0
        elif (distance_error > 40):
            vx = self.kp_forward * distance_error 
            vy = 0
            if (light_pos):
                omega = - self.kp_rotation * center_x_error / (frame_width // 2)
            else:
                omega = 0
        # elif (distance_error > 8):
        elif (distance_error > 0):
            if (abs(self.comm.gyro_info.angleZ) > 10):
                vx, vy = 0, 0
                omega = -self.comm.gyro_info.angleZ / 45.0

            elif (abs(center_x_error / (frame_width // 2)) > 0.1):
                vx, omega = 0, 0
                vy = -self.kp_lateral * center_x_error / (frame_width // 2)
            else:
                vx = self.kp_forward * distance_error 
                vy = - self.kp_lateral * center_x_error / (frame_width // 2)
                omega = 0
        # elif (distance_error > 0):
        #     vx = self.kp_forward * distance_error 
        #     vy, omega = 0, 0
        else:
            vx, vy, omega = 0, 0, 0
            self.yahoo_flag = True
        
        # Clamp values between -1 and 1
        vx = max(min(vx, 1), -1)
        vy = max(min(vy, 1), -1)
        omega = max(min(omega, 1), -1)
        
        return vx, vy, omega

    def run(self):
        try:
            while not self.yahoo_flag:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                height, width = frame.shape[:2]
                
                # Detect light
                light_pos, light_area = self.detect_light(frame)
                
                # Get current distance
                current_distance = self.get_distance()
                
                vx, vy, omega = self.calculate_movement_vectors(
                    light_pos, width, height, light_area, current_distance)
                
                # Convert to wheel speeds
                # fl, fr, bl, br = self.calculate_mecanum_wheel_speeds(vx, vy, omega)
                                    # Send commands to Arduino
                # self.send_mecanum_command(fl, fr, bl, br)
                """
                # Visual feedback
                cv2.circle(frame, light_pos, 10, (0, 255, 0), -1)
                cv2.putText(frame, f"Area: {light_area}", (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"vx: {vx:.2f} vy: {vy:.2f} w: {omega:.2f}", 
                          (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                """
                """
                self.comm.chassis_cmd.vx = int(vx * 200)
                self.comm.chassis_cmd.vy = int(vy * 200)
                self.comm.chassis_cmd.wz = int(omega * 120)
                self.comm.send()
                """
 
                # Display distance if avail self.comm.chassis_cmd.vx = int(vx * 200)able
                """
                if current_distance is not None:
                    cv2.putText(frame, f"Distance: {current_distance}cm", (10, 90),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                """
                # cv2.imshow('Light Seeking', frame)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break
                time.sleep(0.1)

                """
                if (time.time() - self.print_time > 1):
                    print("fdbk", self.comm.chassis_info.distance,
                            self.comm.chassis_cmd.vx, self.comm.chassis_cmd.vy, self.comm.chassis_cmd.wz  )
                    print("gyro", self.comm.gyro_info.angleX,self.comm.gyro_info.angleY, self.comm.gyro_info.angleZ,  )
                    print("pos ", light_pos)
                    # print(self.dir_flag, self.continue_flag, self.yonly_flag, self.y_continue_flag)
                    self.print_time = time.time()
                """
            print("Yahoo!")

        finally:
            self.cleanup()
ports = communication.Communication.detect_ports()
detect_light_flag=False     #detect light from landing gear
tracker_flag=True           #tracker the line
detect_plane_flag=False     #detect the plane
is_holding_flag=False       #whether robort arm is holding the airpl
Finish_flag=False



def main():
    Comm= Communicate()
    Comm.start()
    dn = detectPlane()
    dn.start()
    detect_light=MecanumLightSeeker(detect_light_flag)

    try:
        while True:
            xy = dn.get_xy()
                #light_seeker = MecanumLightSeeker()
            #line trackingcv2.VideoCapture(0)
            if xy[0]==None and comm.Comm.sensorInfo.is_holding==False and detect_plane_flag==False:
                print("no airplane founded")
                tracker_flag=True
                #line trackingcommunicate
            elif xy[0] !=0 and comm.Comm.sensorInfo.is_holding == False and Comm.comm.sensorInfo.distance>=1000:
                detect_plane_flag==True
                tracker_flag=False
                print("Airplane detected, heading to the airplane")
                while xy[0]!=None:
                    latest_xy=xy
                    ret, frame = cv2.VideoCapture(0).read()
                    height, width = frame.shape[:2]
                    current_distance=Comm.comm.sensorInfo.distance
                    vx,vy,wz=detect_light.calculate_movement_vectors(latest_xy, width,height, current_distance)

                    Comm.send_Mecanum(vx,vy,wz)
            elif Comm.comm.sensoInfo.distance<1000:
                detect_light_flag= True
                detect_light.start()

                #comm 
            else if is_holding:
                print("successfully hold the airplane, return back")
                dn.stop()
                tracker_flag=True
            #else if T shaped line detected:
                #finish_flag=True
                #print("Done!")
                #exit(0)
            
            comm.Comm.get_opmode(detetct_light_flag,tracker_flag,detect_plane_flag)
            
        



            #print(xy)
            #light_seeker.run()
            time.sleep(0.5)
    except KeyboardInterrupt:
        dn.stop()

if __name__ == "__main__":
    main()
