import numpy as np
import cv2 as cv
import glob

def rescaleFrame(frame, scale=0.25): # scale value
    # works for Images, Videos, Live Video
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale) # [0] is height, [1] is width
    dimensions = (width, height)
    
    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA) # interpolate area based on new dimensions

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
 
print(objp)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
images = glob.glob('C:/Users/User/Documents/PlatformIO/Projects/Visual_Odometry/NewRes/*.jpg')
 
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9, 6), None)
 
    # If found, add object points, image points (after refining them)
    if ret == True:
        #print(img.shape)
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
 
        # Draw and display the corners
        cv.drawChessboardCorners(img, (9, 6), corners2, ret)
        img = rescaleFrame(img, 0.25)
        cv.imshow('Success', img)
        cv.waitKey(0)
    else:
        img = rescaleFrame(img, 0.25)
        cv.imshow('Failed', img)
        cv.waitKey(0)
        print("Failed :c")

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
#print(rvecs)
print(mtx)
cv.destroyAllWindows()