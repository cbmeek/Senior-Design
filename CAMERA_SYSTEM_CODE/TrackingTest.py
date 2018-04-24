import numpy as np
import cv2
import boto3
import json
import sys
from time import monotonic
import serial
ser = serial.Serial('/dev/ttyACM0',9600)

#initilize timer so I dont spend a fortune on AWS
time = 0
lastTime = 0

# Create AWS client
client=boto3.client('rekognition','us-west-2')

# Create Tracker 
tracker = cv2.TrackerMedianFlow_create()
# Read video
video = cv2.VideoCapture(0)
     
# Exit if video not opened.
if not video.isOpened():
    print ("Could not open video")
    sys.exit()
 
# Read first frame.
ok, frame = video.read()
if not ok:
    print ('Cannot read video file')
    sys.exit()
    
#Deinterlace frame    
frame[1:-1:2] = frame[0:-2:2]/2 + frame[2::2]/2
        
# Get frame Dimensions
frameHeight,frameWidth,dim=frame.shape

#Convert frame into a usable data type for AWS Rekognition
result,encimg=cv2.imencode('.jpg',frame,[int(cv2.IMWRITE_JPEG_QUALITY),100])
if False==result:
    print("could not encode image!")
    quit()
imgbytes = encimg.tostring()

#send frame to AWS Rekognition
response = client.detect_faces(Image={'Bytes': imgbytes}, Attributes=['DEFAULT'])
for faceDetail in response['FaceDetails']:
    #Getting bbox from AWS Rekoginiton
    try:
        rectVert1 = (int(frameWidth*faceDetail['BoundingBox']['Left']),int(frameHeight*faceDetail['BoundingBox']['Top']))
        rectVert2 = (int(frameWidth*faceDetail['BoundingBox']['Width']),int(frameHeight*faceDetail['BoundingBox']['Height']))
        # Define an initial bounding box
        bbox = rectVert1+rectVert2
        print(bbox)
    except:
        print("No face detected using default roi")
        bbox = (287, 23, 86, 320)

# Initialize tracker with first frame and bounding box
ok = tracker.init(frame, bbox)
 
while True:
    print(ser.readline().decode().strip())
    time = monotonic()   
    # Read a new frame
    ok, frame = video.read()
    frame[1:-1:2] = frame[0:-2:2]/2 + frame[2::2]/2
    if not ok:
        break
         
    # Start timer
    timer = cv2.getTickCount()
    
    # Update tracker
    ok, bbox = tracker.update(frame)
 
    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
 
    # Draw bounding box
    if ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        centerX = int((bbox[0] + bbox[2]/2))
        centerY = int((bbox[1] + bbox[3]/2))
        cv2.rectangle(frame, p1, p2, (0,255,0), 3)
        ser.write("<"+ str(centerX) + "," + str(centerY)+ ">")
        cv2.circle(frame,(centerX,centerY), 3, (0,0,255), 0)
    else :
        # Tracking failure
         cv2.putText(frame, "Tracking failure detected. Looking for new target.",
                     (25,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
        #if 5 seconds have passed since the last attempt to find a new target then try again
         if (time-lastTime)>5:
             #Convert frame into a usable data type for AWS Rekognition
             result,encimg=cv2.imencode('.jpg',frame,[int(cv2.IMWRITE_JPEG_QUALITY),100])
             if False==result:
                 print("could not encode image!")
                 quit()
             imgbytes = encimg.tostring()
             response = client.detect_faces(Image={'Bytes': imgbytes}, Attributes=['DEFAULT'])
             for faceDetail in response['FaceDetails']:
                 #Getting bbox from AWS Rekoginiton
                 try:
                     rectVert1 = (int(frameWidth*faceDetail['BoundingBox']['Left']),int(frameHeight*faceDetail['BoundingBox']['Top']))
                     rectVert2 = (int(frameWidth*faceDetail['BoundingBox']['Width']),int(frameHeightqq*faceDetail['BoundingBox']['Height']))
                     bbox = rectVert1+rectVert2
                     ok = tracker.init(frame, bbox)
                 except:
                     cv2.putText(frame,"No face detected",(25,130), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
             lastTime = monotonic()
             
    # Display FPS on frame
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
 
    # Display result
    cv2.imshow("Tracking", frame)
 
    # Exit if ESC pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
