import numpy as np
import cv2
from matplotlib import pyplot as plt
import time

#Load camera parameters
ret = np.load('param_ret.npy')
K = np.load('param_K.npy')
dist = np.load('param_dist.npy')
h,w = (800, 1264)
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(K,dist,(w,h),1,(w,h))

mapx, mapy = cv2.initUndistortRectifyMap(K,dist, None ,new_camera_matrix,(w, h),cv2.CV_16SC2)

#cap = cv2.VideoCapture("/home/sieuwe/Desktop/vidz/full_2.avi")
#cap = cv2.VideoCapture("/home/sieuwe/Desktop/vidz/full_3.avi")
#cap = cv2.VideoCapture("/home/sieuwe/Desktop/vidz/full_1.avi")
#cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)

kernel= np.ones((13,13),np.uint8)

#Stereo matcher settings
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

    start = time.time()

    right, left = decode(frame)

    #Undistort images
    #img_1_undistorted = cv2.undistort(left, K, dist, None, new_camera_matrix)
    #img_2_undistorted = cv2.undistort(right, K, dist, None, new_camera_matrix)
    #img_1_undistorted= cv2.remap(left,mapx,mapy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)  
    #img_2_undistorted= cv2.remap(right,mapx,mapy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    #downsample image for higher speed
    img_1_downsampled = cv2.pyrDown(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY))
    img_2_downsampled = cv2.pyrDown(cv2.cvtColor(right, cv2.COLOR_BGR2GRAY))

    new_w, new_h = img_1_downsampled.shape

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
    
    f = 0.3*w                          # 30cm focal length
    Q = np.float32([[1, 0, 0, -0.5*new_w],
                    [0,-1, 0,  0.5*new_h], # turn points 180 deg around x-axis,
                    [0, 0, 0,      f], # so that y-axis looks up
                    [0, 0, 1,      0]])
    points = cv2.reprojectImageTo3D(disp, Q)

    z_values = points[:,:,2]
    z_values = z_values.flatten()
    indices = z_values.argsort()

    precentage = 25280
    min_distance = np.mean(np.take(z_values,indices[0:precentage]))                             # takes the 30% lowest measuerements and gets the average distance from these.
    avg_distance = np.mean(z_values)                                                           # averages all distances
    max_distance = np.mean(np.take(z_values,indices[z_values.shape[0]-precentage:z_values.shape[0]])) # takes the 30% highest measuerements and gets the average distance from these.
    #print(np.take(z_values,indices[z_values.shape[0]-precentage:z_values.shape[0]]))
    #print(np.take(z_values,indices[:-100]))

    #visualize
    cv2.imshow("Depth", disp_Color)
    left = cv2.resize(left,(632, 400))
    color_depth = cv2.addWeighted(left,0.4,disp_Color,0.4,0)

    end = time.time()
    fps = 1 / (end-start)

    cv2.putText(color_depth, "minimum: " + str(round(min_distance,1)),(5, 20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(color_depth, "average: " + str(round(avg_distance,1)),(5, 40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(color_depth, "maximum: " + str(round(max_distance,1)),(5, 60),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(color_depth, "FPS: " + str(round(fps)),(5, 80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)

    cv2.imshow("color & Depth", color_depth)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
