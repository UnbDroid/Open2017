import numpy as np
import cv2

cap = cv2.VideoCapture(1)

# Define the codec and create VideoWriter object
fourcc = cv2.CV_FOURCC('D', 'I', 'V', 'X')
out = cv2.VideoWriter('Videos/CupVid.avi',fourcc, 24.0, (640,480), 1)
key = cv2.waitKey(1)
while(key != 27):
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    key = cv2.waitKey(1)
    while ((ret==True)and (key == 13)):
    	ret, frame = cap.read()
    	cv2.imshow('frame',frame)
        # write the flipped frame
        out.write(frame)
    	keyAux = cv2.waitKey(1)
    	if keyAux == 13:
    		key=0

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()