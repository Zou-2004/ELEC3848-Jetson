#!/usr/bin/env python3
#
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import sys
import argparse
import time

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, Log

import threading


DP_PERIOD = 0.1
DP_SW_LEN = 0.5

class detectPlane:
    def __init__(self, found_prob=0.1):
        self.FOUND_PROB = found_prob

        self.net = detectNet("ssd-mobilenet-v2", None, 0.5 )        

        self.input = None
        self.output = None
        self.running = None

        self.avg_left = -1
        self.avg_right = -1
        self.avg_top = -1
        self.avg_bottom = -1

        self.plane_found = False
	
    def start(self):
        self.input = videoSource("/dev/video0")
        self.output = videoOutput("")

        self.running = True
        self.t = threading.Thread(target = self.detect_airplane)
        self.t.start()

    def stop(self):
        self.running = False
        self.t.join()

        print("Thread joint.")
        time.sleep(1)

        self.input.Close()
        self.output.Close()

        time.sleep(1)
        print("IS STREAMING =", self.input.IsStreaming())

    def get_xy(self):
        if self.plane_found:
            return (self.avg_left + self.avg_right) / 2, (self.avg_top + self.avg_bottom) / 2
        return None
    
    def get_found(self)->bool:
        return self.plane_found 

    # def calculate_movement_vectors(self):
    #     pass

    def detect_airplane(self):
        detect_data=[]
        # detect_data["Top"],detect_data["Left"],detect_data["Right"],detect_data["Bottom"]=0,0,0,0
        # detect_data["Top_average"],detect_data["Left average"],detect_data["Right_average"],detect_data["Bottom_average"]=0,0,0,0        
        while self.running:
            tt = time.time()

            # capture the next image
            img = self.input.Capture()
            # print("\n\n\n\n\n", img.height, img.width)
            # print("\n\n\n\n\n")
            # exit()

            
            #delete timeout info
            # detect objects in the image (with overlay)
            pop_idx = []
            for i, dd in enumerate(detect_data):
                if time.time() - dd[0] > DP_SW_LEN:
                    pop_idx.append(i)
                else:
                    break
            for i in pop_idx[::-1]:
                detect_data.pop(i)
            #delete timeout info

            detections = self.net.Detect(img, overlay="box,labels,conf")

            # print the detections
            # print("detected {:d} objects in image".format(len(detections)))
            # count_start=1
            for detection in detections:
                if detection.ClassID == 5:# detect airplane
                    detect_data.append((time.time(), detection.Top, detection.Left, detection.Right, detection.Bottom, True))
                    break
            else:
                detect_data.append((time.time(), 0, 0, 0, 0, False))
                

            eff_dd = [d for d in detect_data if d[5]]
            self.plane_found = (len(eff_dd) / len(detect_data) > self.FOUND_PROB)
            if (len(eff_dd) > 0):
                self.avg_top = sum([d[1] for d in eff_dd]) / len(detect_data)
                self.avg_left = sum([d[2] for d in eff_dd]) / len(detect_data)
                self.avg_right = sum([d[3] for d in eff_dd]) / len(detect_data)
                self.avg_bottom = sum([d[4] for d in eff_dd]) / len(detect_data)

            self.output.Render(img)
            
                    # if current_time-start_time <= 0.5:
                        # detect_data["Top_average"]=detect_data["Top"]/count_start
                        # detect_data["Left_average"]=detect_data["Left"]/count_start
                        # detect_data["Right_average"]=detect_data["Right"]/count_start
                        # detect_data["Bottom_average"]=detect_data["Bottom"]/count_start
                    # else:detect_light
                        # detect_data={}
                        # count_start=1
                        # return detect_data["Top_average"],detect_data["Left_average"],detect_data["Right_average"],detect_data["Bottom_average"]      
            
            sleep_t = DP_PERIOD - (time.time() - tt)
            if sleep_t > 0:
                time.sleep(sleep_t)

        # render the image

        # update the title bar
        #output.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))

        # print out performance info
        #net.PrintProfilerTimes()

        # # exit on input/output EOS
        # if not input.IsStreaming() or not output.IsStreaming():
        #     return
