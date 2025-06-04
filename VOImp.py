from picamera2 import Picamera2
import time
import numpy as np
import cv2

K = np.array([[1.31411739e+03, 0.00000000e+00, 3.08535247e+02],
 [0.00000000e+00, 1.31523328e+03, 2.55907773e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) # Camera Intrinsic Matrix
picam2 = Picamera2()
fps = 10
camera_config = picam2.create_still_configuration(main={"size": (640, 480)}, lores={"size": (640, 480)}, display="lores")
picam2.configure(camera_config)
picam2.set_controls({"FrameDurationLimits": [int(1_000_000/fps), int(1_000_000/fps)]})  # change to 1FPS
picam2.start()
time.sleep(2)


def rescaleFrame(frame, scale=0.25): # scale value
    # works for Images, Videos, Live Video
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale) # [0] is height, [1] is width
    dimensions = (width, height)

    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA) # interpolate area based on new dimensions

# function to create mask of matched pixels
def show_pixels(matched_points, frame):
    blank = np.full_like(frame, 255)
    for m in matched_points:
        y = int(round(m[0]))
        x = int(round(m[1]))
        cv2.circle(blank, (y, x), radius=1, color=frame[x, y].tolist(), thickness=-1)
        # print(f"Drawing point at ({x}, {y})")
    return blank

def run_ORB(frame1, frame2):   
    # Convert it to grayscale
    frame1_bw = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
    frame2_bw = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Initialize the ORB detector algorithm
    orb = cv2.ORB_create()
    
    # Now detect the keypoints and compute
    # the descriptors for the query image
    # and train image
    queryKeypoints, queryDescriptors = orb.detectAndCompute(frame1_bw,None)
    trainKeypoints, trainDescriptors = orb.detectAndCompute(frame2_bw,None)

    #print(queryKeypoints)
    #print(trainKeypoints)
    # Initialize the Matcher for matching
    # the keypoints and then match the
    # keypoints
    matcher = cv2.BFMatcher() # Brute Force Matcher
    matches = matcher.match(queryDescriptors,trainDescriptors)

    matches = sorted(matches, key=lambda x: x.distance) # sort by distance
    matched_points_frame1 = [queryKeypoints[m.queryIdx].pt for m in matches]
    matched_points_frame2 = [trainKeypoints[m.trainIdx].pt for m in matches]
    final_img = cv2.drawMatches(frame1, queryKeypoints, frame2, trainKeypoints, matches[:50],None)
    final_img = cv2.resize(final_img, (1000,650))

    return (matched_points_frame1, matched_points_frame2, final_img)

def get_P(m_frame1, m_frame2, camMat):
    E, mask = cv2.findEssentialMat(m_frame1, m_frame2, camMat)
    retval, R, t, mask = cv2.recoverPose(E, m_frame1, m_frame2, camMat)
    P = np.hstack((R, t))
    return P

def VO(frame1, frame2):
    ORB_out = run_ORB(frame1, frame2)

    matched_points_frame1 = ORB_out[0]
    matched_points_frame2 = ORB_out[1]
    pts1 = np.array(matched_points_frame1)
    pts2 = np.array(matched_points_frame2)
    print(pts1.shape) # (500, 2) means 500 x/y coords 

    #final_img = ORB_out[2]
    #mask_img = show_pixels(matched_points_frame1, frame1)
    P2 = get_P(pts1, pts2, K) # Projection matrix of frame 2
    P2 = K @ P2
    P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1)))) # Projection  matrix of frame  - assume no rotation/translation

    points_3D = cv2.triangulatePoints(P1 , P2, np.array(pts1).T, np.array(pts2).T) # gives 4D(??) coords of each point in frame 1 relative to frame 2
    #cv2.imshow("Matches", final_img)
    return points_3D

k=0
frame1 = picam2.capture_array()
frame1 = rescaleFrame(frame1)
time.sleep(1/fps)

while True:
    frame2 = picam2.capture_array()
    frame2 = rescaleFrame(frame2)
    pixels = VO(frame1, frame2)
    print(pixels.T[0])
    frame1 = frame2
    k += 1
    time.sleep(1/fps)