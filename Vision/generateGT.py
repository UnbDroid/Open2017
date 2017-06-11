#Code to generate Ground Truth images - Cow chalenge

import cv2
import numpy as np


#_______________________________________________________________________________________________________________________________
def mouse(e, x, y, flags, param):
	global key
	global GT

	if e == cv2.EVENT_LBUTTONDOWN:
		print ("X = %d  Y = %d"%(x, y))
		if key == 98:
			GT[y, x] = [255, 255, 255]			#Points in the cow body are in white
		else:
			GT[y, x] = [0, 0, 255]				#Points in the cow legs are in red
		cv2.imshow("%03dGT" % num, GT)

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
	for num in range(input("Primeira foto: "), input("Ultima foto: ") +1):

		#Image Read
		#imgName = raw_input("Nome da imagem: ")
		img = cv2.imread("src/%d.jpg" %(num))
		#name = GetRawName(imgName)

		#Ground Truth Image
		height = img.shape[0]
		width = img.shape[1]
		GT = np.zeros((height, width, 3))

		#Create windows
		cv2.namedWindow("%dGT" %(num))
		cv2.imshow("%dGT" %(num), GT)

		cv2.namedWindow("%d"%(num), cv2.WINDOW_NORMAL)
		cv2.imshow("%d"%(num), img)
		cv2.setMouseCallback("%d"%(num), mouse)


		#Analysis
		print("Press 'l' to analyze the legs, 'b' to analyze the body, 'enter' to end analyzing the image, and 'esc' to end the process")
		key = 98
		key = cv2.waitKey(0)
		print key
		
		if key == 27:  #esc
			break

		while ((key==98)or(key==108)):
			key = cv2.waitKey(0)
			print key
			
		if key == 27:  #esc
			break

		cv2.imwrite('GT/%s_GT.png' %(num), GT)
		cv2.destroyAllWindows()

main()