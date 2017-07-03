import cv2
import numpy as np
import sys
import os

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
	src = cv2.imread('src/' + sys.argv[1] + '.png')

	if(src is None):
		print 'Error loading image'
		print 'Use $ python findChess.py <image_number>'
		print 'Example: $ python findChess.py 001'
		return -1

	img = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

	cv2.namedWindow('Parameters')
	cv2.createTrackbar('Threshold', 'Parameters', 750, 1000, nothing)
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

		points_mtx = cv2.bitwise_or(hits1, hits2)

		points_found = np.where(points_mtx)

		npoints = np.shape(points_found)[1]

		print 'Points found:'
		print npoints

		disp = np.copy(src)
		disp_filtered = np.copy(src)

		for i in xrange(npoints):
			cv2.circle(disp, (points_found[1][i], points_found[0][i]), 10, (0, 0, 255), thickness = 2)

		done = 0
		for i in xrange(npoints):
			if(done == 0):
				angcoef = np.zeros(npoints-1)
				distance = np.zeros(npoints-1)
				point1 = [points_found[0][i], points_found[1][i]]
				for j in xrange(npoints):
					if(i != j):
						point2 = [points_found[0][j], points_found[1][j]]

						deltay = float(point2[0]-point1[0])
						deltax = float(point2[1]-point1[1])
						
						if(deltax != 0):
							m = deltay/deltax
						else:
							m = 1.63317787e+16

						if(j > i):
							angcoef[j-1] = m
						else:
							angcoef[j] = m

				for k in xrange(npoints-1):
					epsilon = np.absolute(angcoef - angcoef[k])
					pointsaligned = (epsilon < 0.05)
					score = np.sum(pointsaligned)

					candidatepointsindex = np.where(pointsaligned)[0]
					candidatepointsindex[candidatepointsindex >= i] += 1
					candidatepointsindex = np.append(i, candidatepointsindex)
					candidatepoints = [points_found[0][candidatepointsindex], points_found[1][candidatepointsindex]]
					
					ncandidates = np.size(candidatepointsindex)
					mindist = np.zeros(ncandidates)
					maxdist = np.zeros(ncandidates)

					for t in xrange(ncandidates):
						dist = ((candidatepoints[1]-candidatepoints[1][t])**2+(candidatepoints[0]-candidatepoints[0][t])**2)**0.5
						mindist[t] = np.amin(dist[dist != 0])
						maxdist[t] = np.amax(dist)

					line_width = np.amax(maxdist)
					square_width = np.mean(mindist)

					ratio = line_width/square_width

					if(score >= 4 and np.isclose(ratio, 4, atol=0.1)):
						truepoints = [points_found[0][candidatepointsindex], points_found[1][candidatepointsindex]]

						print 'Distances to the nearest neighbour point:'
						print mindist
						print 'Distances to the farthest point:'
						print maxdist
						print 'All Points Checked:'
						print truepoints[1]
						print truepoints[0]

						npointsfiltered = np.shape(truepoints)[1]
						for w in xrange(npointsfiltered):
							cv2.circle(disp_filtered, (truepoints[1][w], truepoints[0][w]), 10, (0, 0, 255), thickness = 2)
						
						done = 1
						break
					

		#cv2.imshow('src', src)
		#cv2.imshow('chess1', cv2.convertScaleAbs(chess1))
		#cv2.imshow('chess2', cv2.convertScaleAbs(chess2))
		cv2.imshow('hits', disp)
		cv2.imshow('hits filtered', disp_filtered)

		os.system('clear')

		if cv2.waitKey(20) & 0xFF == 27:
			break



main()