import cv2
import numpy as np
import sys

def nothing(x):
	pass

def createTemplate(templateSize):
	size = templateSize/2

	plus = np.ones((size, size), dtype=np.float32)
	minus = -np.ones((size, size), dtype=np.float32)

	tmp = np.append(plus, minus, 0)

	template = np.append(tmp, cv2.flip(tmp, 0), 1)

	return template

def integralImage(src):
	h, w = src.shape
	intImg = np.zeros((h+1, w+1), dtype = np.float32)

	for i in xrange(0, h):
		for j in xrange(0, w):
			intImg[i+1, j+1] = intImg[i, j] + np.sum(src[i, 0:j+1]) + np.sum(src[0:i, j])
			#intImg[i+1, j+1] = np.sum(src[0:i+1, 0:j+1])

	return intImg

def findChess(intImg, templateSize):
	h, w = intImg.shape

	start = templateSize-1
	stoph = (h-1) - templateSize/2
	stopw = (w-1) - templateSize/2

	chess = np.zeros((h-1, w-1), dtype = np.float32)

	tmp = templateSize/2

	for i in xrange(start, stoph):
		for j in xrange(start, stopw):
			chess[i, j] += intImg[i+1, j+1] - intImg[i+1-tmp, j+1] - intImg[i+1, j+1-tmp] + intImg[i+1-tmp, j+1-tmp]
			chess[i, j] += intImg[i+1+tmp, j+1+tmp] - intImg[i+1+tmp, j+1] - intImg[i+1, j+1+tmp] + intImg[i+1, j+1]
			chess[i, j] -= intImg[i+1, j+1+tmp] - intImg[i+1, j+1] - intImg[i+1-tmp, j+1+tmp] + intImg[i+1-tmp, j+1]
			chess[i, j] -= intImg[i+1+tmp, j+1] - intImg[i+1, j+1] - intImg[i+1+tmp, j+1-tmp] + intImg[i+1, j+1-tmp]

	return chess

def main():
	img = cv2.imread('src/' + sys.argv[1] + '.jpg', cv2.CV_LOAD_IMAGE_GRAYSCALE)

	if(img is None):
		print 'Error loading image'
		print 'Use $ python findChess.py <image_number>'
		print 'Example: $ python findChess.py 001'
		return -1

	cv2.namedWindow('Parameters')
	cv2.createTrackbar('Threshold', 'Parameters', 850, 1000, nothing)
	cv2.createTrackbar('Template Size', 'Parameters', 4, 20, nothing)

	while(True):
		templateSize = cv2.getTrackbarPos('Template Size', 'Parameters')

		if(templateSize % 2 != 0):
			templateSize -= 1
		if(templateSize == 0):
			templateSize = 2

		template1 = createTemplate(templateSize)
		template2 = cv2.flip(template1, 0)

		chess1 = cv2.matchTemplate(img.astype(np.float32), template1, cv2.TM_CCORR)
		chess2 = cv2.matchTemplate(img.astype(np.float32), template2, cv2.TM_CCORR)

		dil1 = cv2.dilate(chess1, np.ones((3,3)))
		dil2 = cv2.dilate(chess2, np.ones((3,3)))

		localMax1 = cv2.compare(chess1, dil1, cv2.CMP_EQ)
		localMax2 = cv2.compare(chess2, dil2, cv2.CMP_EQ)

		minVal, maxVal1, minLoc, maxLoc = cv2.minMaxLoc(chess1)
		minVal, maxVal2, minLoc, maxLoc = cv2.minMaxLoc(chess2)

		thresh = 0.001*cv2.getTrackbarPos('Threshold', 'Parameters')
		thresh1 = thresh*maxVal1
		thresh2 = thresh*maxVal2
		
		ret, binarychess1 = cv2.threshold(chess1, thresh1, 255, cv2.THRESH_BINARY)
		ret, binarychess2 = cv2.threshold(chess2, thresh2, 255, cv2.THRESH_BINARY)
		
		binarychess1 = np.uint8(binarychess1)
		binarychess2 = np.uint8(binarychess2)

		hits1 = cv2.bitwise_and(binarychess1, localMax1)
		hits2 = cv2.bitwise_and(binarychess2, localMax2)

		disp = cv2.bitwise_or(hits1, hits2)

		cv2.imshow('src', img)
		cv2.imshow('chess1', cv2.convertScaleAbs(chess1))
		cv2.imshow('chess2', cv2.convertScaleAbs(chess2))
		cv2.imshow('hits', disp)

		if cv2.waitKey(20) & 0xFF == 27:
			break

main()