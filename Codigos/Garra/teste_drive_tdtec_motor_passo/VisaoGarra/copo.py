import numpy as np
import cv2

def corrigePosGarra (posicao_garra):
    pass

def andaRobo ():
    pass

def identificaCopo (imagem):
    area = cv2.sumElems(imagem)
    #cv2.namedWindow('ide', cv2.WND_PROP_OPENGL)
    cv2.imshow('ide', imagem)
    print (area[0])
    if area[0] > 10:
        return 1
    return 0

def copoFora (posGarra, cap):
    posicao_garra = posGarra
    delay = 2000
    cv2.waitKey()
    cv2.waitKey()
    while(cap.isOpened()):
        try:
            ret, frame = cap.read()
            #cv2.namedWindow('frame', cv2.WND_PROP_OPENGL)
            cv2.imshow('frame', frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            x2Esq = posicao_garra #210
            x1Esq = x2Esq - 80 #290
            y1Esq = 350
            y2Esq = 450

            imgcopoEsquerda = frame [y1Esq:y2Esq, x1Esq:x2Esq]
            #cv2.namedWindow('imgcopoEsquerda', cv2.WND_PROP_OPENGL)
            cv2.imshow('imgcopoEsquerda', imgcopoEsquerda)

            esqH,esqS,esqV = cv2.split(imgcopoEsquerda)
            ret, binEsqH = cv2.threshold(esqH,100,1, cv2.THRESH_BINARY_INV)

            copoNaEsquerda = identificaCopo (binEsqH)

            x1Dir = posicao_garra + 80#365
            x2Dir = x1Dir+ 80#445
            y1Dir = 350
            y2Dir = 450

            imgCopoDireita = frame [y1Dir:y2Dir, x1Dir:x2Dir]

            dirH,dirS,dirV = cv2.split(imgCopoDireita)
            ret, binDirH = cv2.threshold(dirH,100,1, cv2.THRESH_BINARY_INV)

            copoNaDireita= identificaCopo(binDirH)

            char = cv2.waitKey(delay)

            #cv2.namedWindow('imgCopoDireita', cv2.WND_PROP_OPENGL)
            cv2.imshow('imgCopoDireita', imgCopoDireita)

            #cv2.namedWindow('Hsvframe', cv2.WINDOW_OPENGL)
            cv2.imshow('Hsvframe',dirH)

            #cv2.namedWindow('thresh1', cv2.WINDOW_OPENGL)
            bin2 = binDirH*255
            cv2.imshow('thresh1',bin2)

            #cv2.namedWindow('thresh3', cv2.WINDOW_OPENGL)
            bin3 = binEsqH*255
            cv2.imshow('thresh3',bin3)

            if copoNaDireita == 0 and copoNaEsquerda == 0:
                return 1

            elif copoNaDireita == 1:
                posicao_garra += 10
                corrigePosGarra (posicao_garra)

            elif copoNaEsquerda == 1:
                posicao_garra -= 10
                corrigePosGarra (posicao_garra)


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
            return 0
    # When everything done, release the capture


def copoDentro (posicao_garra, cap):
    posicao_garra = posGarra
    while(cap.isOpened()):
        try:
            ret, frame = cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            x2 = posicao_garra-20 #210
            x1 = posicao_garra+20 #290
            y1 = 400
            y2 = 500

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

cap = cv2.VideoCapture('video.mp4')

if(cap.isOpened()):
    a = copoFora (285, cap)
    if (a==1):
        b = copoDentro (cap)
else:
    print('Could not open VideoCapture')

cap.release()
cv2.destroyAllWindows()
