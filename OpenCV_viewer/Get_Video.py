import numpy as np
import cv2
from matplotlib import pyplot as plt

#Load camera parameters
ret = np.load('param_ret.npy')
K = np.load('param_K.npy')
dist = np.load('param_dist.npy')
h,w = (800, 1264)
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(K,dist,(w,h),1,(w,h))

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)

kernel= np.ones((20,20),np.uint8)

#Note: disparity range is tuned according to specific parameters obtained through trial and error. 
win_size = 5
min_disp = 16
max_disp = min_disp * 9
num_disp = max_disp - min_disp # Needs to be divisible by 16
stereo = cv2.StereoSGBM_create(minDisparity= min_disp,
numDisparities = num_disp,
blockSize = 25,
uniquenessRatio = 5,
speckleWindowSize = 50,
speckleRange = 5,
disp12MaxDiff = 6,
P1 = 8*3*win_size**2,#8*3*win_size**2,
P2 =32*3*win_size**2) #32*3*win_size**2)

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

    #cv2.imwrite("left.png", left)
    #cv2.imwrite("right.png", right)

    #Undistort images
    #img_1_undistorted = cv2.undistort(left, K, dist, None, new_camera_matrix)
    #img_2_undistorted = cv2.undistort(right, K, dist, None, new_camera_matrix)

    #img_1_downsampled = cv2.pyrDown(cv2.cvtColor(img_1_undistorted, cv2.COLOR_BGR2GRAY))
    #img_2_downsampled = cv2.pyrDown(cv2.cvtColor(img_2_undistorted, cv2.COLOR_BGR2GRAY))

    #disparity_map = stereo.compute(img_1_downsampled,img_2_downsampled)
    #disparity_u8 = np.uint8(disparity_map)

    #disparity_color = cv2.applyColorMap(disparity_u8, cv2.COLORMAP_JET)
    
    #cv2.imshow('disparity', np.uint8(disparity_color))

    #denoised= cv2.morphologyEx(disparity_color,cv2.MORPH_CLOSE, kernel)
    
    cv2.imshow('left', left)
    cv2.imshow('right', right)

    #cv2.imshow('color', denoised)    


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
