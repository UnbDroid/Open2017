import numpy as np
import cv2
import serial
import time

def corrigePosGarra (posicao_garra):
    #lobal arduino
    #arduino.write('m')
    #arduino.write(chr (posicao_garra))

    time.sleep(1)
    return 0

def andaRobo ():
    return 0

def identificaCopo (imagem):
    area = cv2.sumElems (imagem)
    cv2.namedWindow('ide', cv2.WND_PROP_OPENGL)
    cv2.imshow('ide', imagem)
    print (area[0])
    if area[0] > 1:
        return 1
    return 0

def copoFora (posGarra, cap):
    pos_real = 0
    posicao_garra = posGarra
    delay = 2000

    while(1):
        try:
            #for i in range (0, 19):
        	    #retr = cap.grab()
    	        #retr, frame = cap.retrieve(retr)

            ret, frame = cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            x1Dir = posicao_garra + 25#365
            x2Dir = x1Dir + 80#445
            y1Dir = 420
            y2Dir = 475
            imgCopoDireita = frame [y1Dir:y2Dir, x1Dir:x2Dir]

            frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
            cv2.rectangle(frame,(x1Dir,y1Dir),(x2Dir,y2Dir),(0,255,0),3)
            cv2.namedWindow('frame', cv2.WND_PROP_OPENGL)
            cv2.imshow('frame', frame)

            dirH,dirS,dirV = cv2.split(imgCopoDireita)
            ret, binDirH = cv2.threshold(dirH,60,1, cv2.THRESH_BINARY_INV)

            copoNaDireita= identificaCopo (binDirH)

            #char = cv2.waitKey(delay)
            #cv2.namedWindow('imgCopoDireita', cv2.WND_PROP_OPENGL)
            #cv2.imshow('imgCopoDireita', imgCopoDireita)

            #cv2.namedWindow('Hsvframe', cv2.WINDOW_OPENGL)
            #cv2.imshow('Hsvframe',dirH)

            cv2.namedWindow('thresh1', cv2.WINDOW_OPENGL)
            bin2 = binDirH*255
            cv2.imshow('thresh1',bin2)

            if copoNaDireita == 0:# and copoNaEsquerda == 0:
                if posicao_garra == posGarra:
                    posicao_garra += 35
                    pos_real += 30
                    corrigePosGarra (pos_real)
                    time.sleep(2)
                    #cv2.waitKey ()
                else:
                    pos_real += 5
                    corrigePosGarra (pos_real)
                    return 1

            elif copoNaDireita == 1:
                posicao_garra += 9
                pos_real += 11
                corrigePosGarra (pos_real)

        except Exception as e:
            print 'Erro'
            return 0


def copoDentro (posGarra):
    cap = cv2.VideoCapture(0)
    posicao_garra = posGarra
    while(1):
        try:
            #for i in range (0, 19):
        	#    retr = cap.grab()
    	    #    retr, frame = cap.retrieve(retr)

            ret, frame = cap.read()

    	    cv2.namedWindow('frame', cv2.WND_PROP_OPENGL)
            cv2.imshow('frame', frame)

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            x2 = posicao_garra-30 #210
            x1 = posicao_garra+60 #290
            y1 = 415
            y2 = 450
            imgCopoDentro = frame [y1:y2, x1:x2]
    	    print ('5')

    	    cv2.namedWindow('imgCopo', cv2.WND_PROP_OPENGL)
                cv2.imshow('imgCopo', imgCopoDentro)
    	    print ('6')

                canalH,canalS,canalV = cv2.split(frame)
    	    print ('1')
                ret, binH = cv2.threshold(canalH,60,1, cv2.THRESH_BINARY_INV)
    	    print ('4')

                copoDentro = identificaCopo (binH)
    	    print ('2')

                if copoDentro == 0:
                    return 1

                else:
    		print ("aqui")
    		cv2.waitKey()
                    #andaRobo ()
    	    print ('3')

        except Exception as e:
            print 'Erro'
            return 0

#use ttyUSB0 se for o caso, ou se for Windows, COMx.
#arduino = serial.Serial('/dev/ttyACM1',9600,timeout=2)
#a = arduino.read()
#time.sleep(3)
cap = cv2.VideoCapture(0)
#cv2.waitKey()
a = copoFora (250, cap)

#arduino.write('f')
#b = copoDentro (296)
#cap.release()
cv2.destroyAllWindows()
