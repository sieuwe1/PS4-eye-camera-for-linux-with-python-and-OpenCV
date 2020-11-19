import numpy as np
import cv2

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

print("to perform calibration make sure you have a chessboard printed and pisitoned ata clear white wall for the best results.")
print("We will take 40 pictures from diffrent angles. Each time press enter to take a picture.")
print("So let's get started by making the first picture")

for i in range (40):
    input("press enter to make picture")
    ret, frame = cap.read()
    left, right = decode(frame)
    cv2.imwrite("pic" + str(i)+ ".jpg", left)

print("done!")

cap.release()
