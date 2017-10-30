import numpy as np
import cv2

def printPixel(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print '\nCor do pixel: '
        print x
        print y
        print frame[y, x]


cap = cv2.VideoCapture('o1.avi')
# Capture frame-by-frame
mediaCorEsq = 0
mediaCorDir = 0
cv2.namedWindow('frame', cv2.WINDOW_OPENGL)
cv2.setMouseCallback('frame',printPixel)

for x in range(0, 30):
    print x
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    x1Esq = 410
    x2Esq = 490
    y1Esq = 230
    y2Esq = 320
    copoEsquerda = frame [y1Esq:y2Esq, x1Esq:x2Esq]
    #copoEsquerda = frame [200:300, 200:300]
    cv2.imshow('copoEsquerda', copoEsquerda)

    mediaCorE_coluna = np.average(copoEsquerda, axis=0)
    mediaCorEsq += np.average(mediaCorE_coluna, axis=0)
    #print(mediaCorEsq)

    x1Dir = 400
    x2Dir = 500
    y1Dir = 300
    y2Dir = 400
    copoDireita = frame [y1Dir:y2Dir, x1Dir:x2Dir]
    cv2.imshow('copoDireita', copoDireita)

    mediaCorD_coluna = np.average(copoDireita, axis=0)
    mediaCorDir += np.average(mediaCorD_coluna, axis=0)
    #print(mediaCorDir)
mediaCorEsq = mediaCorEsq/30
mediaCorDir = mediaCorDir/30
delay = 100
while(cap.isOpened()):
#while(1):
    try:
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        x1Esq = 210
        x2Esq = 290
        y1Esq = 230
        y2Esq = 320

        copoEsquerda = frame [y1Esq:y2Esq, x1Esq:x2Esq]
        #copoEsquerda = frame [200:300, 200:300]
        cv2.namedWindow('copoEsquerda', cv2.WND_PROP_OPENGL)
        cv2.imshow('copoEsquerda', copoEsquerda)

        mediaCorE_coluna = np.average(copoEsquerda, axis=0)
        corEsq= np.average(mediaCorE_coluna, axis=0) - mediaCorDir
        corEsqNorm = cv2.norm (corEsq)

        #print(corEsqNorm)
        #print(corEsq[2])


        x1Dir = 365
        x2Dir = 445
        y1Dir = 350
        y2Dir = 450

        copoDireita = frame [y1Dir:y2Dir, x1Dir:x2Dir]
        cv2.imshow('copoDireita', copoDireita)

        mediaCorD_coluna = np.average(copoDireita, axis=0)
        corDir = np.average(mediaCorD_coluna, axis=0)
        corDir -= mediaCorDir
        corDirNorm = cv2.norm (corDir) - mediaCorDir

        print('\nEsq: Dir:')
        print(mediaCorDir)
        print(corDirNorm)


        char = cv2.waitKey(delay)


        #hsv = cv2.cvtColor(copoDireita, cv2.COLOR_BGR2HSV)
        hsframe,sframe,vframe = cv2.split(copoDireita)
        ret,thresh1 = cv2.threshold(hsframe,100,255, cv2.THRESH_BINARY)

        cv2.namedWindow('Hsvframe', cv2.WINDOW_OPENGL)
        cv2.imshow('Hsvframe',hsframe)

        cv2.namedWindow('thresh1', cv2.WINDOW_OPENGL)
        cv2.imshow('thresh1',thresh1)
        #cv2.namedWindow('hSvframe', cv2.WINDOW_OPENGL)
        #cv2.imshow('hSvframe',sframe)
        #cv2.namedWindow('hsVframe', cv2.WINDOW_OPENGL)
        #cv2.imshow('hsVframe',vframe)

        lower_red = np.array([110,190,230])
        upper_red = np.array([120,130,255])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(copoDireita, lower_red, upper_red)

        #res = cv2.bitwise_and(copoDireita,copoDireita, mask= mask)
        #cv2.imshow('res',res)
        cv2.imshow('mask',mask)

        #cv2.namedWindow('frame', cv2.WINDOW_OPENGL)
        #cv2.imshow('luv', luv)
        #cv2.imshow('lab', lab)
        cv2.imshow('frame', frame)
        if char == ord(','):
            delay = delay*2
        if char == ord('.'):
            delay = delay/2
        if char == ord('q'):
            break
        if char == ord('p'):
            cv2.waitKey ()
    except Exception as e:
        print 'Erro'
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
