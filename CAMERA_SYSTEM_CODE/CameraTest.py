import numpy as np
import cv2
import boto3
import json


s3 = boto3.resource('s3')
client=boto3.client('rekognition','us-west-2')
cap = cv2.VideoCapture(0)


while(True):

    # Capture frame-by-frame
    ret, frame = cap.read()
    
    #deinterlace frame
    frame[1:-1:2] = frame[0:-2:2]/2 + frame[2::2]/2

    #get frame dimenstions
    frameHeight,frameWidth,dim=frame.shape

    #Convert frame into a usable data type for AWS Rekognition
    result,encimg=cv2.imencode('.jpg',frame,[int(cv2.IMWRITE_JPEG_QUALITY),100])
    if False==result:
        print("could not encode image!")
        quit()
    imgbytes = encimg.tostring()
   
    #send frame to AWS
    response = client.detect_faces(Image={'Bytes': imgbytes}, Attributes=['DEFAULT'])
    for faceDetail in response['FaceDetails']:
        print("<" + str(faceDetail['BoundingBox']['Height']) + "," +
              str(faceDetail['BoundingBox']['Left']) + "," +
              str(faceDetail['BoundingBox']['Top']) + "," +
              str(faceDetail['BoundingBox']['Width']) + ">")

    rectVert1 = (int(frameWidth*faceDetail['BoundingBox']['Left']),int(frameHeight*faceDetail['BoundingBox']['Top']))
    rectVert2 = (int(frameWidth*(faceDetail['BoundingBox']['Width']+faceDetail['BoundingBox']['Left'])),int(frameHeight*(faceDetail['BoundingBox']['Height']+faceDetail['BoundingBox']['Top'])))
                 
    print("rectVert1: " +str(rectVert1))
    print("rectVert2: " +str(rectVert2))
                 
    newframe = cv2.rectangle(frame,rectVert1,rectVert2,(0,255,0),3)
    newframe = cv2.rectangle(frame,(0,0),(1,1),(0,255,0),3)
    t=False
    # Display the resulting frame
    cv2.imshow('I See You', newframe)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break   
    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

