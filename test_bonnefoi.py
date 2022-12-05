#from https://subscription.packtpub.com/book/application_development/9781785283932/3/ch03lvl1sec28/accessing-the-webcam
#https://stackoverflow.com/questions/33239669/opencv-how-to-merge-two-images

import cv2      # for video processing
import numpy as np  # for pip and multiple video
import time, datetime
import socket   # for tello cmd sending
import threading # for data read and joystick
import imutils 

#####

#JOY_SCRIPT = 'mylibs/TelloCtrl_joyFree.py'
JOY_SCRIPT = 'mylibs/TelloCtrl_joySnes1.py'

#####
TELLO = False                # activate tello
JOYSTICK = False             # activate joystick
SAVEVID  = False             # save video
FILTER = True              # activate opencv video filters
VIDEOTEXT = True            # display date fps resolution
DRONETEXT = True           # display drone data : acc yaw etc.
LOGDRONETEXT = False         #activate drone data log to file
PREVIEW = False             # display video /= recorded video

reduction_factor = 1      # video resolution reduction : 0.5 = half
tello_info = ""

print("Tello : ", TELLO, "\nJoystick: ",JOYSTICK, "\nsave video: ", SAVEVID)

if SAVEVID or  LOGDRONETEXT:
    dt = str(datetime.datetime.now().replace(microsecond=0))
    dt = dt.replace(":","")
    dt = dt.replace("-","")
    dt = dt.replace(" ","_")

if TELLO :
    ##### TELLO ##########
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockinfo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tello_cmd_address = ('192.168.10.1', 8889)
    #tello_info_address = ('192.168.10.255', 8890)
    tello_info_address = ('192.168.10.2', 8890)
    #tello_info_address = ('192.168.10.3', 8890)
    #local_address = ('192.168.10.2', 8889)

    sockinfo.bind(tello_info_address)

    sock.sendto("command".encode('ascii'),tello_cmd_address)
    sock.sendto("streamon".encode('ascii'),tello_cmd_address)

if TELLO and (DRONETEXT or LOGDRONETEXT) :
    def get_info(sock,e,l,log=False):
        global tello_info         # should use locks on it
        global LOGDRONETEXT
        if LOGDRONETEXT :
            try :
                f = open('./videos/data_'+dt+'.csv','w')
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
        if LOGDRONETEXT :
            print("closing data log file")
            f.close()


    e = threading.Event()
    l = threading.Lock()
    e.set()
    t1 = threading.Thread(target = get_info , args=(sockinfo,e,l,))
    t1.start()

if JOYSTICK :
    print("starting joystick")
    import subprocess # to execute joystick control
    pjoy = None   # process
    def joycontrol():
        global pjoy
        # for linux
        #pjoy = subprocess.Popen(['/usr/bin/python3.8',
        #                         'mylibs/TelloCtrl_joyFree.py'],stdout=subprocess.PIPE ,stderr=subprocess.PIPE)
        #macos
        pjoy = subprocess.Popen(['/Library/Frameworks/Python.framework/Versions/3.9/bin/python3.9',
                                JOY_SCRIPT],stdout=subprocess.PIPE ,stderr=subprocess.PIPE)
        ##
        ## with Popen(["ifconfig"], stdout=PIPE) as proc:
        print(pjoy.stdout.read())
        print(pjoy.stderr.read())
    t2 = threading.Thread(target = joycontrol ,args=())
    t2.start()

###### VIDEO #################
if TELLO :
    cap = cv2.VideoCapture("udp://@:11111")    #  tello edu
else :
    cap = cv2.VideoCapture(0)                   #  webcam

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

# Get fps, width and height via the video capture property.
fps = cap.get(cv2.CAP_PROP_FPS)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

print("source format :",fps,"fps",width,"x",height)
#tello : 25.0 fps 960.0 x 720.0
#mbp cam : 29.97002997002997 fps 1280.0 x 720.0

ww = int(width * reduction_factor)
wh = int(height * reduction_factor)


if SAVEVID :
    writer= cv2.VideoWriter('./videos/video_'+dt+'.mkv', cv2.VideoWriter_fourcc(*'X264'), 25, (ww,wh))
    #writer= cv2.VideoWriter('video.avi',cv2.VideoWriter_fourcc(*'jpeg'), 25, (ww,wh))
    #writer= cv2.VideoWriter('found my codec test', -1,30, (200,200))
    #writer= cv2.VideoWriter('video.mp4',cv2.VideoWriter_fourcc(*'MJPG'), 25, (ww,wh))
    #writer= cv2.VideoWriter('video.avi',cv2.VideoWriter_fourcc(*'DIVX'), 25, (200,200))

if VIDEOTEXT or DRONETEXT :
    #font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
    #font = 0 # cv2.FONT_HERSHEY_SIMPLEX
    font = 1 # cv2.FONT_HERSHEY_PLAIN


if VIDEOTEXT :
    # for fps
    prev_frame_time = 0
    new_frame_time = 0
    firsttime = time.time()

if FILTER :
    piph = wh // 4
    pipw = ww // 4

    multi = np.zeros((wh+piph, ww+pipw, 3), dtype="uint8")


def get_colored_objects(frame):
    width = int(cap.get(3))
    height = int(cap.get(4))

    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    blue = np.uint8([[[255, 0, 0]]]) #here insert the bgr values which you want to convert to hsv
    hsvBlue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
    print(hsvBlue)

    lower_blue = hsvBlue[0][0][0] - 10, 100, 100
    upper_blue = hsvBlue[0][0][0] + 10, 255, 255
    lower_blue = np.array(lower_blue, dtype=np.uint8)
    upper_blue = np.array(upper_blue, dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        area = cv2.contourArea(c)
        if area > 200:
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (36, 255, 12), 3)

    return frame

print("starting video loop")
while True:

    # Video
    ret, frame = cap.read()

    # if reduct_factor not 1
    #frame = cv2.resize(frame,(ww,wh), interpolation=cv2.INTER_AREA)
    #frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    if FILTER :
        edges = cv2.Canny(frame,200,200)
        edges = cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)

        #combine
        alpha = 0.5
        beta = 1.0 - alpha
        combined = cv2.addWeighted( frame, alpha, edges, 1, 0,3);
        pipedges = cv2.resize(edges,(pipw,piph), interpolation=cv2.INTER_AREA)
        frame = get_colored_objects(frame)
        pipframe = cv2.resize(frame,(pipw,piph), interpolation=cv2.INTER_AREA)
        #pipcombined = cv2.resize(combined,(pipw,piph), interpolation=cv2.INTER_AREA)
        #dst=combined
        dst=combined
        dst[wh-piph:,ww-pipw:]=pipedges
        #dst[wh-piph*2:wh-piph,ww-pipw:]=pipcombined
        dst[wh-piph*2:wh-piph,ww-pipw:]=pipframe

        if VIDEOTEXT :
            dst = cv2.putText(dst,"CANNY Filter",
                          (ww-pipw,wh-piph+13),font, 1,(140, 255, 50),1)
            #dst = cv2.putText(dst,"COMBINE",
            #              (ww-pipw,wh-piph*2+13),font, 1,(140, 255, 50),1)
            dst = cv2.putText(dst,"CAMERA",
                          (ww-pipw,wh-piph*2+13),font, 1,(140, 255, 50),1)

        #roi = cv2.rectangle((0,0), (edges.shape[1], edges.shape[0]),(1,1,1),2)
        #subview = frame(roi)
        #edges.copyto(subview)

        #pipedges.copyto(dst.rowRange(1, 1+pipw).colRawh-piphnge(3, 3+piph))
    else:
        dst = frame

    if VIDEOTEXT :
        # Get date and time and
        # save it inside a variable
        new_frame_time = time.time() #datetime.datetime.now().replace(microsecond=0)
        #dt = str(datetime.date.fromtimestamp(new_frame_time)) #.replace(microsecond=0)
        dt = str(datetime.datetime.now().replace(microsecond=0))
        # Calculating the fps
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        # converting the fps into integer
        fps = int(fps)
        fps = str(fps)
        # putting the FPS count on the frame
        #cv2.putText(frame, , (7, 70), font, 3, (140, 255,120), 3, cv2.LINE_AA)    # put the dt variable over the
        # video frame
        videotext = dt+"    "+str(ww)+"x"+str(wh)+" "+str(fps)+"fps "
        if SAVEVID :
            videotext += str(int(new_frame_time-firsttime))+"s"
        dst = cv2.putText(dst, videotext,
                                (10,wh- 10),font, 1,(140, 255, 50),1)
    if TELLO & DRONETEXT:
        teinfol = tello_info.split(";")

        l.acquire()
        dst = cv2.putText(dst,"TELLO",
                          (10,15),font, 1,(140, 255, 50),1)
        dst = cv2.putText(dst,teinfol[-4]+" "+teinfol[-3]+" "+teinfol[-2]+" "+teinfol[-6],
                          (10,28),font, 1,(140, 255, 50),1)

        dst = cv2.putText(dst,teinfol[5]+" "+teinfol[6]+" "+teinfol[7],
                          (10,41),font, 1,(140, 255, 50),1)
        #bat
        dst = cv2.putText(dst,teinfol[-7]+" "+teinfol[11]+" "+teinfol[12],
                          (10,55),font, 1,(140, 255, 50),1)

        dst = cv2.putText(dst,teinfol[0]+" "+teinfol[4]+" "+teinfol[-8]+" "+teinfol[-9],
                          (10,68),font, 1,(140, 255, 50),1)
        dst = cv2.putText(dst,teinfol[8]+" "+teinfol[9]+" "+teinfol[10],
                          (10,81),font, 1,(140, 255, 50),1)
        dst = cv2.putText(dst,teinfol[1]+" "+teinfol[2]+" "+teinfol[3],
                          (10,94),font, 1,(140, 255, 50),1)
        l.release()


    ### AFFICHAGE
    if PREVIEW :
        cv2.imshow('Drone', pipframe)
    else :
        #cv2.imshow('Drone', frame)
        #cv2.imshow('Drone edges', edges)
        cv2.imshow('Drone add', dst)
        #cv2.imshow('Drone pip mix', pipcombined)

    if SAVEVID :
        writer.write(dst)

    #use esc key to exit
    c = cv2.waitKey(1)
    if c == 27:
        break


print("finishing...(ESC key recieved)")

if TELLO and DRONETEXT:
    e.clear()
    t1.join()
    print("drone data thread ended")

print("closing video display...")
cv2.destroyAllWindows()

print("release video capture")
cap.release()

if JOYSTICK :
    print("end of joystick thread...")
    pjoy.send_signal(subprocess.signal.SIGINT)
    t2.join()

if SAVEVID :
    print("closing video save file...")
    writer.release()

if TELLO :
    print("send streamof command and closing drone communication...")
    time.sleep(1)
    sock.sendto("streamoff".encode('ascii'),tello_cmd_address)
    sock.close()
    sockinfo.close()

print("-fin-")