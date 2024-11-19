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
# 
# print(sys.argv);
# exit()
# # parse the command line
# parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 # formatter_class=argparse.RawTextHelpFormatter, 
                                 # epilog=detectNet.Usage() + videoSource.Usage() + videoOutput.Usage() + Log.Usage())
# 
# parser.add_argument("input", type=str, default="", nargs='?', help="URI of the input stream")
# parser.add_argument("output", type=str, default="", nargs='?', help="URI of the output stream")
# parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
# parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
# parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 
# 
# try:
	# args = parser.parse_known_args()[0]
# except:
	# print("")
	# parser.print_help()
	# sys.exit(0)

# create video sources and outputs
# input = videoSource(args.input, argv=sys.argv)
# output = videoOutput(args.output, argv=sys.argv)
	
# load the object detection network
# net = detectNet(args.network, sys.argv, args.threshold)

# note: to hard-code the paths to load a model, the following API can be used:
#
# net = detectNet(model="model/ssd-mobilenet.onnx", labels="model/labels.txt", 
#                 input_blob="input_0", output_cvg="scores", output_bbox="boxes", 
#                 threshold=args.threshold)

# process frames until EOS or the user exits

class detectPlane:
    def __init__(self):
        self.net = detectNet("ssd-mobilenet-v2", None, 0.5 )
        self.input = videoSource("/dev/video0")
        self.output = videoOutput("")

        self.running = True

        self.avg_left = -1
        self.avg_right = -1
        self.avg_top = -1
        self.avg_bottom = -1

        self.plane_found = False
	
    def start(self):
        self.t = threading.Thread(target = self.detect_airplane)
        self.running = True
        self.t.start()

    def stop(self):
        self.running = False
        self.t.join()

    def get_xy(self):
        if self.plane_found:
            return (self.avg_left + self.avg_right) / 2, (self.avg_top + self.avg_bottom) / 2
        return -1, -1

    def detect_airplane(self):
        detect_data=[]
        # detect_data["Top"],detect_data["Left"],detect_data["Right"],detect_data["Bottom"]=0,0,0,0
        # detect_data["Top_average"],detect_data["Left average"],detect_data["Right_average"],detect_data["Bottom_average"]=0,0,0,0
        while self.running:
            # capture the next image
            img = self.input.Capture()

            
            #delete timeout info
            # detect objects in the image (with overlay)
            pop_idx = []
            for i, dd in enumerate(detect_data):
                if time.time() - dd[0] > 0.5:
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
            self.plane_found = False
            for detection in detections:
                
                if detection.ClassID == 5:# detect airplane
                    # if count_start==1:
                        # start_time=time.time() #start timer
                    # count_start=1
                    detect_data.append((time.time(), detection.Top, detection.Left, detection.Right, detection.Bottom))
                    
                    self.avg_top = sum([d[1] for d in detect_data]) / len(detect_data)
                    self.avg_left = sum([d[2] for d in detect_data]) / len(detect_data)
                    self.avg_right = sum([d[3] for d in detect_data]) / len(detect_data)
                    self.avg_bottom = sum([d[4] for d in detect_data]) / len(detect_data)
                    self.plane_found = True

            self.output.Render(img)

                    # if current_time-start_time <= 0.5:
                        # detect_data["Top_average"]=detect_data["Top"]/count_start
                        # detect_data["Left_average"]=detect_data["Left"]/count_start
                        # detect_data["Right_average"]=detect_data["Right"]/count_start
                        # detect_data["Bottom_average"]=detect_data["Bottom"]/count_start
                    # else:
                        # detect_data={}
                        # count_start=1
                        # return detect_data["Top_average"],detect_data["Left_average"],detect_data["Right_average"],detect_data["Bottom_average"]      
                     
                    

        # render the image

        # update the title bar
        #output.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))

        # print out performance info
        #net.PrintProfilerTimes()

        # # exit on input/output EOS
        # if not input.IsStreaming() or not output.IsStreaming():
        #     return
