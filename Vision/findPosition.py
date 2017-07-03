import numpy as np
import cv2

#Function and it's macros to be used in the oficial code

#Macros:
FOCAL_DIST = 602.1197
X_ERROR_MI = 0.129152		#The x axis position of the analised line have an error that grows linearly (approximately) with the distance in the z axis
X_ERROR_CONST = -0.6332		#Mathematically: error = z1*X_ERROR_MI + X_ERROR_CONSTANT

def findPos(line1size, line2size, realHeight1, realHeight2, px1, px2, xcenter):		#Begining a endings of known vertical lines from the cow, real measures of them, position in the x axis in pixel level, and center of the image
	z1 = (FOCAL_DIST*realHeight1)/line1size
	z2 = (FOCAL_DIST*realHeight2)/line2size
	X1_CENTER_ERROR = X_ERROR_MI*z1 + X_ERROR_CONST
	X2_CENTER_ERROR = X_ERROR_MI*z2 + X_ERROR_CONST
	x1 = (z1*(px1-xcenter)/FOCAL_DIST)-X1_CENTER_ERROR
	x2 = (z2*(px2-xcenter)/FOCAL_DIST)-X2_CENTER_ERROR
	dist = pow( pow(z1-z2, 2) + pow(x1-x2, 2) , 0.5)
	return z1, z2, x1, x2, dist 		#It returns the position of both vertical lines in the world (in depth z and in the horizontal x axis), and the distance between those lines


#________________________________________________________________________________________________________________________________________

#From here, all the algorithm are written just to test the function findPos, it does NOT go to the oficial code

#Storing of the begining and ending of the lines
count=0
pointx = []
pointy = []

#Mouse function to select the lines
def mouse(e, x, y, param, flag):
	global count
	global pointx
	global pointy
	if(e == cv2.EVENT_LBUTTONDOWN):
		pointx.append(x)
		pointy.append(y)
		count+=1
		print x, y


def main():
	#Capturing the camera video
	cap = cv2.VideoCapture(1)
	cv2.namedWindow("video", cv2.WINDOW_NORMAL)
	cv2.setMouseCallback("video", mouse)
	key = cv2.waitKey(1)
	while(key!=27):		#End the code by pressing esc
		key = cv2.waitKey(1)
		ret, frame = cap.read()
		cv2.imshow("video", frame)
		if key == 13:	#Press enter to stop the video in the desired frame, and analise that position of the cow
			cv2.waitKey(0)
			if count%4==0:		#Test the funtion findPos every time there's 4 points selected (2 lines)
				z1, z2, x1, x2, dist = findPos(pow(pow(pointy[count-4]-pointy[count-3], 2) + pow(pointx[count-4]-pointx[count-3], 2), 0.5), pow(pow(pointy[count-2]-pointy[count-1], 2) + pow(pointx[count-2]-pointx[count-1], 2), 0.5), 10, 10, (pointx[count-3]+pointx[count-4])/2, (pointx[count-1]+pointx[count-2])/2, frame.shape[0]/2)
				print z1, x1, z2, x2
				print dist
main()