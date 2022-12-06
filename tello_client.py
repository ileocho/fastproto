# -*- coding: utf-8 -*-
"""
Created on Fri Dec  2 09:42:48 2022

@author: natha
"""

import socket
import threading
import time
import path_design as pathDesign
import cv2
import datetime
import numpy as np

from djitellopy import Tello

def commandDroneUDP(sock, address, commands):
        
    #Send takeoff command
    print("send takeoff ....")
    sock.sendto("takeoff".encode(), address)
    
    #Start command process
    for command in commands:
        print(command + " ....")
        sock.sendto(command.encode(), address)
 
        time.sleep(3)
       
    #Send land command
    print("send land ....")
    sock.sendto("land".encode(), address)
    
    return 0
      

def commandWithTello(commands, tello):
    
    tello.takeoff()
    
    for command in commands:
        command_array = command.split(" ")
        x = int(command_array[0])
        y = int(command_array[1])
        z = int(command_array[2])
        speed = int(command_array[3])
        
        if command_array[0] == "go" :
            tello.go_xyz_speed(x,y,z,speed)
        elif command_array[0] == "curve" : 
            tello.curve_xyz_speed(x,y,z,speed)
        
    tello.land()
    
    return 0
            

def receiveVideoStream(sock, tello_cmd_address):
    
    print("send command ....")    
    sock.sendto("command".encode('ascii'),tello_cmd_address)
    print("send streamon ....")
    sock.sendto("streamon".encode('ascii'),tello_cmd_address)
 
    #Send streamon request

    cap = cv2.VideoCapture("udp://@:11111")
    
    # Get fps, width and height via the video capture property.
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)    
    
    print("source format :",fps,"fps",width,"x",height)
    #tello : 25.0 fps 960.0 x 720.0
    #mbp cam : 29.97002997002997 fps 1280.0 x 720.0
    
    dt = str(datetime.datetime.now().replace(microsecond=0))
    dt = dt.replace(":","")
    dt = dt.replace("-","")
    dt = dt.replace(" ","_")
    
    ww = int(width * 1)
    wh = int(height * 1)
   
    writer= cv2.VideoWriter('./video_'+dt+'.mkv', cv2.VideoWriter_fourcc(*'X264'), 25, (ww,wh))
 
    while keepRecording:
        
        # Video
        ret, frame = cap.read()
        print(ret, frame)
        
        dst = frame

        ### AFFICHAGE
        cv2.imshow("Drone",dst)    
        
        writer.write(dst)
    
        #use esc key to exit
        c = cv2.waitKey(1)
        if c == 27:
            break
    
    print("finishing...(ESC key recieved)")
    
    print("closing video display...")
    cv2.destroyAllWindows()
    
   # e.clear()
    #t1.join()
    print("drone data thread ended")

    cap.release()

    print("end of command thread...")
    #t2.join()

    print("closing video save file...")
    writer.release()


    print("send streamof command and closing drone communication...")
    time.sleep(1)
   # drone.streamoff()
    sock.sendto("streamoff".encode('ascii'),tello_cmd_address)
    sockinfo.close()

    print("-fin-")
    
    return

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video_2.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))

    while keepRecording:
        video.write(frame_read.frame)
        time.sleep(1 / 30)

    video.release()


def getInfo(sock,e,l,log=False):
    global tello_info         # should use locks on it
    global LOGDRONETEXT
    if LOGDRONETEXT :
        dt = str(datetime.datetime.now().replace(microsecond=0))
        dt = dt.replace(":","")
        dt = dt.replace("-","")
        dt = dt.replace(" ","_")
        
        try :
            f = open('./data_'+dt+'.csv','w')
        except :
            print('pb creating data log file')
            LOGDRONETEXT = False
    while e.isSet():
        info,addr = sock.recvfrom(1024)
        l.acquire()
        tello_info = info.decode('ascii')
        l.release()
        if LOGDRONETEXT :
            f.write('date:'+str(time.time())+';'+tello_info)
        #print(tello_info)
   
    print("closing data log file")
    f.close()
        
    return

#%% Method UDP 

###### SEND COMMAND & RECEIVE RESPONSE ######
# IP and Port  
localIP = "127.0.0.1"
droneIP = "192.168.10.1"

UDPCommandport = 8889
UDPStatePort = 8890
UDPVideoPort = 11111

# Number of bytes to get
numBytes2Get = 1024
        
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sockinfo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

tello_cmd_address =(droneIP, UDPCommandport)
tello_info_address = ('192.168.10.2', UDPStatePort)

sockinfo.bind(tello_info_address)

#%% djitellopy packet

#for the function buildPath()
x = [100] 
y = [30]
z = [100]
       
commands, err = pathDesign.buildPath(x,y,z)
print(commands)
 
### Start connect, command and stream with tello ###

tello = Tello()

tello.connect(False)

keepRecording = True
tello.streamon()
frame_read = tello.get_frame_read()

recorder = threading.Thread(target=videoRecorder)
recorder.start()

###### END #######

### Stream with UDP method ###

#recorder = threading.Thread(target=receiveVideoStream, args(sock_video, tello_cmd_address))
#recorder.start()

### END ###

commandWithTello(commands, tello)
commandDroneUDP(sock_cmd, tello_cmd_address, commands)

keepRecording = False
recorder.join()

## ADD function getInfo()


