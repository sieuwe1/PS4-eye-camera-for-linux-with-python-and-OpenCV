import numpy as np
import cv2

#cap = cv2.VideoCapture('http://root:root@192.168.70.52/mjpg/1/video.mjpg')
cap = cv2.VideoCapture(0)

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

    left, right = decode(frame)
    left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    cv2.imshow('left',left)

    stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
    disparity = stereo.compute(left,right)

    norm_image = cv2.normalize(disparity, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

    cv2.imshow("depth", norm_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()