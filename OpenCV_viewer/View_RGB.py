import numpy as np
import cv2

#cap = cv2.VideoCapture('http://root:root@192.168.70.52/mjpg/1/video.mjpg')
cap = cv2.VideoCapture(2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)

def decode(frame):
    left = np.zeros((800,1264,3), np.uint8)
    right = np.zeros((800,1264,3), np.uint8)
    
    for i in range(800):
        left[i] = frame[i, 64: 1280 + 48] 
        right[i] = frame[i, 1280 + 48: 1280 + 48 + 1264] 
    
    return (left, right)

while(True):

    ret, frame = cap.read()
    #cv2.normalize(frame, frame, 0, 255, cv2.NORM_MINMAX)

    left, right = decode(frame)

    cv2.imshow('left',left)
    cv2.imshow('right',right)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
