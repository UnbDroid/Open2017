import numpy as np
import cv2
import serial
import time

def corrigePosGarra (posicao_garra):
    global arduino
    arduino.write('m')
    arduino.write(chr (posicao_garra))

    time.sleep(0.1)
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

def copoFora (posGarra):
    pos_real = 0
    posicao_garra = posGarra
    delay = 2000

    while(1):
        try:
            cap = cv2.VideoCapture(1)
            ret, frame = cap.read()
            ret, frame = cap.read()
            cap.release()
            del cap
            cv2.namedWindow('frame', cv2.WND_PROP_OPENGL)
            cv2.imshow('frame', frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


            x2Esq = posicao_garra - 30 #210
            x1Esq = x2Esq - 55 #290
            y1Esq = 420
            y2Esq = 475

            #imgcopoEsquerda = frame [y1Esq:y2Esq, x1Esq:x2Esq]
            #cv2.namedWindow('imgcopoEsquerda', cv2.WND_PROP_OPENGL)
            #cv2.imshow('imgcopoEsquerda', imgcopoEsquerda)

            #esqH,esqS,esqV = cv2.split(imgcopoEsquerda)
            #ret, binEsqH = cv2.threshold(esqH,60,1, cv2.THRESH_BINARY_INV)

            #copoNaEsquerda = identificaCopo (binEsqH)

            x1Dir = posicao_garra + 25#365
            x2Dir = x1Dir + 75#445
            y1Dir = 420
            y2Dir = 475
            imgCopoDireita = frame [y1Dir:y2Dir, x1Dir:x2Dir]
            dirH,dirS,dirV = cv2.split(imgCopoDireita)
            ret, binDirH = cv2.threshold(dirH,60,1, cv2.THRESH_BINARY_INV)

            copoNaDireita= identificaCopo (binDirH)

            char = cv2.waitKey(delay)
            cv2.namedWindow('imgCopoDireita', cv2.WND_PROP_OPENGL)
            cv2.imshow('imgCopoDireita', imgCopoDireita)

            cv2.namedWindow('Hsvframe', cv2.WINDOW_OPENGL)
            cv2.imshow('Hsvframe',dirH)

            cv2.namedWindow('thresh1', cv2.WINDOW_OPENGL)
            bin2 = binDirH*255
            cv2.imshow('thresh1',bin2)

            cv2.namedWindow('thresh3', cv2.WINDOW_OPENGL)
            bin3 = binEsqH*255
            cv2.imshow('thresh3',bin3)
            print ('a4')
            if copoNaDireita == 0:# and copoNaEsquerda == 0:
                if posicao_garra == posGarra:
                    print ('a1')
                    posicao_garra += 24
                    print ('a2')
                    pos_real += 30
                    print ('a3')
                    corrigePosGarra (pos_real)
                    #cv2.waitKey ()
                else:
                    print ('a12312')
                    return 1

            elif copoNaDireita == 1:
                print ('a6')
                posicao_garra += 8
                pos_real += 10
                corrigePosGarra (pos_real)

            #elif copoNaEsquerda == 1:
                #print ('a7')
                #posicao_garra -= 5
                #pos_real -= 12
                #corrigePosGarra (pos_real)
            print ('a5')
            if char == ord(','):
                delay = delay*2
            if char == ord('.'):
                delay = delay/2
            if char == ord('q'):
                break
            if char == ord('p'):
                cv2.waitKey ()
            #cv2.waitKey ()
        except Exception as e:
            print 'Erro'
            return 0
    # When everything done, release the capture


def copoDentro (posicao_garra, cap):
    posicao_garra = posGarra
    while(1):
        try:
            ret, frame = cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            x2 = posicao_garra-25 #210
            x1 = posicao_garra+55 #290
            y1 = 370
            y2 = 450

            imgCopoDentro = frame [y1:y2, x1:x2]

            canalH,canalS,canalV = cv2.split(imgCopoDentro)
            ret, binH = cv2.threshold(canalH,100,1, cv2.THRESH_BINARY_INV)

            copoDentro = identificaCopo (binH)

            if copoDentro == 0:
                return 1

            else:
                andaRobo ()

        except Exception as e:
            print 'Erro'
            return 0

#use ttyUSB0 se for o caso, ou se for Windows, COMx.
arduino = serial.Serial('/dev/ttyACM0',9600,timeout=2)
a = arduino.read()
time.sleep(3)
cv2.waitKey()
#cap = cv2.VideoCapture(1)
a = copoFora (250)

arduino.write('f')
    #b = copoDentro (cap)
#cap.release()
cv2.destroyAllWindows()
