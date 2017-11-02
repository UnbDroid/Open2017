import cv2
import numpy as np

# Create a VideoCapture object
cap = cv2.VideoCapture('o3.avi')

# Check if camera opened successfully
if (cap.isOpened() == False):
  print("Unable to read camera feed")

while(True):
  print '1'
  ret, frame = cap.read()
  if ret == True:
    lower_red = np.array([0,0,85])
    upper_red = np.array([80,80,220])
    mask = cv2.inRange(frame, lower_red, upper_red)
    cv2.imshow('mask',mask)
    frame2 = frame.copy()
    print '2'
    contours,hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    maiorContorno = 0
    maiorArea = 640*480
    print '3'
    for i in range (0, len(contours)):
        cnt = contours[i]
        areaEmTeste = cv2.contourArea(cnt)
        if areaEmTeste > maiorArea:
            maiorArea = areaEmTeste
            maiorContorno = i
    print '4'
    cnt = contours[maiorContorno]
    cv2.drawContours(frame, [cnt], 0, (255,0,0), 3)
    print '5'
    cv2.drawContours(frame2, contours, -1, (0,255,0), 1)
    print '6'

    cv2.imshow('contours',frame2)
    # Display the resulting frame
    cv2.imshow('frame',frame)

    # Press Q on keyboard to stop recording
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  # Break the loop
  else:
    break

# When everything done, release the video capture and video write objects
cap.release()

# Closes all the frames
cv2.destroyAllWindows()
