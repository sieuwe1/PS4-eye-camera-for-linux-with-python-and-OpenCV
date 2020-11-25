import numpy as np
import cv2
from matplotlib import pyplot as plt

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

out_full = cv2.VideoWriter('full.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(3448,808))
out_left = cv2.VideoWriter('left.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))
out_right = cv2.VideoWriter('right.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))

while(1):

    ret, frame = cap.read()
    right, left = decode(frame)

    out_full.write(frame)
    out_left.write(left)
    out_right.write(right)

    cv2.imshow('left', left)
    cv2.imshow('right', right)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
