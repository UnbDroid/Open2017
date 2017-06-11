#Code to generate Ground Truth images - Cow chalenge

import cv2
import numpy as np


#_______________________________________________________________________________________________________________________________

def mouse(e, x, y, flags, param):
	global GT
	

	if e == cv2.EVENT_LBUTTONDOWN:
		print ("X = %d  Y = %d"%(x, y))
		GT[y, x] = [255, 255, 255]				#Points in the cow body are in white	
		cv2.imshow("%03dGT" %(num), GT)

	elif e == cv2.EVENT_RBUTTONDOWN:
		print ("X = %d  Y = %d"%(x, y))
		GT[y, x] = [0, 0, 255]					#Points in the cow legs are in red
		cv2.imshow("%03dGT" %(num), GT)

#________________________________________________________________________________________________________________________
	#Function to isolate the name passed (taking off any .jpg, or address before '/')
def GetRawName(name):
	bar = 1
	dot = 1
	while ((name[-bar:-bar+1] != '/')and(bar<len(name)+1)):
		bar = bar+1

	while ((name[-dot:-dot+1] != '.')and(dot<len(name))):
		dot = dot+1
	return (name[-bar+1:-dot])

#________________________________________________________________________________________________________________________

def main():
	for num in range(input("First picture: "), input("Last picture: ") +1):

        #Image reading
        #imgName = raw_input("Image name: ")
        img = cv2.imread("src/%03d.jpg" %(num))
        #name = GetRawName(imgName)

        #Ground Truth image
        height = img.shape[0]
        width = img.shape[1]
        GT = np.zeros((height, width, 3))

        #Windows
	cv2.namedWindow("%03dGT" %(num))
	cv2.imshow("%03dGT" %(num), GT)

	cv2.namedWindow("%03d"%(num), cv2.WINDOW_NORMAL)
	cv2.imshow("%03d"%(num), img)
	cv2.setMouseCallback("%03d"%(num), mouse)


        #Analysis
	print("Left mouse button for cow body points, right mouse button for cow legs points.\nPress 'esc' to end the process, and any other button to go to next image")
	key = cv2.waitKey(0)	
	if key == 27:  #esc
		break

	cv2.imwrite('GT/%03d_GT.png' %(num), GT)
	cv2.destroyAllWindows()

main()
