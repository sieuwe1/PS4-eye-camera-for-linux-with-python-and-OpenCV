import numpy as np
import cv2
from matplotlib import pyplot as plt

#Load camera parameters
ret = np.load('param_ret.npy')
K = np.load('param_K.npy')
dist = np.load('param_dist.npy')
h,w = (800, 1264)
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(K,dist,(w,h),1,(w,h))

cap = cv2.VideoCapture("/home/sieuwe/PS4-eye-camera-for-linux-with-python-and-OpenCV/OpenCV_viewer/full.avi")
#cap = cv2.VideoCapture("/home/sieuwe/PS4-eye-camera-for-linux-with-python-and-OpenCV/OpenCV_viewer/full_3.avi")
#cap = cv2.VideoCapture("/home/sieuwe/PS4-eye-camera-for-linux-with-python-and-OpenCV/OpenCV_viewer/full_1.avi")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)

kernel= np.ones((13,13),np.uint8)

#Note: disparity range is tuned according to specific parameters obtained through trial and error. 
win_size = 5
min_disp = 10
max_disp = 16 * 2 + 10
num_disp = max_disp - min_disp # Needs to be divisible by 16
stereo = cv2.StereoSGBM_create(minDisparity= min_disp,
numDisparities = num_disp,
blockSize = 5,
uniquenessRatio = 10,
speckleWindowSize = 1000,
speckleRange = 10,
disp12MaxDiff = 25,
P1 = 8*3*win_size**2,#8*3*win_size**2,
P2 =32*3*win_size**2) #32*3*win_size**2)

def decode(frame):
    left = np.zeros((800,1264,3), np.uint8)
    right = np.zeros((800,1264,3), np.uint8)
    
    for i in range(800):
        left[i] = frame[i, 64: 1280 + 48] 
        right[i] = frame[i, 1280 + 48: 1280 + 48 + 1264] 
    
    return (left, right)

def get_distance(d):
    return 30 * 10/d

while(1):

    ret, frame = cap.read()

    right, left = decode(frame)

    #Undistort images
    img_1_undistorted = cv2.undistort(left, K, dist, None, new_camera_matrix)
    img_2_undistorted = cv2.undistort(right, K, dist, None, new_camera_matrix)

    #downsample image for higher speed
    img_1_downsampled = cv2.pyrDown(cv2.cvtColor(img_1_undistorted, cv2.COLOR_BGR2GRAY))
    img_2_downsampled = cv2.pyrDown(cv2.cvtColor(img_2_undistorted, cv2.COLOR_BGR2GRAY))

    #compute stereo
    disp = stereo.compute(img_1_downsampled,img_2_downsampled)
    
    #denoise step 1
    denoised = ((disp.astype(np.float32)/ 16)-min_disp)/num_disp
    dispc= (denoised-denoised.min())*255
    dispC= dispc.astype(np.uint8)     
    
    #denoise step 2
    denoised= cv2.morphologyEx(dispC,cv2.MORPH_CLOSE, kernel)
    
    #apply color map
    disp_Color= cv2.applyColorMap(denoised,cv2.COLORMAP_OCEAN)
    
    #visualize
    cv2.imshow("Depth", disp_Color)
    left = cv2.resize(left,(632, 400))
    color_depth = cv2.addWeighted(left,0.4,disp_Color,0.4,0)
    cv2.imshow("color & Depth", color_depth)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
