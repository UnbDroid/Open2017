import cv2
import numpy as np

def drawGT(src, GT):
	ret = np.copy(src)

	h = src.shape[0]
	w = src.shape[1]

	for x in xrange(0, w):
		for y in xrange(0, h):
			if(np.all(GT[y, x] == [255, 255, 255])):
				cv2.circle(ret, (x, y), 10, (255, 255, 255), thickness = 2)

			if(np.all(GT[y, x] == [0, 0, 255])):
				cv2.circle(ret, (x, y), 10, (0, 0, 255), thickness = 2)

	return ret

def main():
	n = 1

	src = cv2.imread('src/%03d.png' % n)
	gt = cv2.imread('GT/%03d_GT.png' % n)

	while(src is not None or gt is not None):
		img = drawGT(src, gt)
		cv2.imshow('Image %03d View GT' % n, img)
		cv2.imshow('Image %03d Ground Truth' % n, gt)
		cv2.imshow('Image %03d Source' % n, src)

		while(True):
			key = cv2.waitKey(1)

			if(key & 0xFF == 102):
				n = n + 1
				cv2.destroyAllWindows()
				src = cv2.imread('src/%03d.png' % n)
				gt = cv2.imread('GT/%03d_GT.png' % n)
				break
			if(key & 0xFF == 27):
				src = None
				gt = None
				break

	cv2.destroyAllWindows()


main()