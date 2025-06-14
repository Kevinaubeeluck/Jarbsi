import numpy as np
import cv2

#K = np.array([[1100, 0, 960],
#     [0, 1100, 540],
#     [0,    0,   1]]) # typical High res matrix

K = np.array([[2.49958316e+03, 0.00000000e+00,  9.92922387e+02],
 [0.00000000e+00, 2.50636951e+03, 7.42220253e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) # High Res Camera Intrinsic Matrix

#K = np.array([[1.25048727e+03, 0.00000000e+00, 3.20442247e+02],
# [0.00000000e+00, 1.25140777e+03, 1.86992519e+02],
# [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) # Low Res Camera Intrinsic Matrix

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

    #print(np.array(queryKeypoints).shape)
    #print(np.array(trainKeypoints).shape)
    # Initialize the Matcher for matching
    # the keypoints and then match the
    # keypoints
    matcher = cv2.BFMatcher() # Brute Force Matcher
    #matches = matcher.match(queryDescriptors,trainDescriptors)

    matches = matcher.knnMatch(queryDescriptors, trainDescriptors, k=2) 
    #print(np.array(matches).shape)  
    good_matches = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good_matches.append(m)  
    
    matches = sorted(good_matches[:], key=lambda x: x.distance) # sort by distance
    matched_points_frame1 = [queryKeypoints[m.queryIdx].pt for m in matches]
    matched_points_frame2 = [trainKeypoints[m.trainIdx].pt for m in matches]
    final_img = cv2.drawMatches(frame1, queryKeypoints, frame2, trainKeypoints, good_matches[:],None)
    final_img = cv2.resize(final_img, (1000,650))

    return (matched_points_frame1, matched_points_frame2, final_img)

def get_P(m_frame1, m_frame2, camMat):
    E, mask = cv2.findEssentialMat(m_frame1, m_frame2, camMat, method=cv2.RANSAC, prob = 0.999, threshold =1.0)
    inlier_1 = m_frame1[mask.ravel() == 1]
    inlier_2 = m_frame2[mask.ravel() == 1]
    #inlier_1, inlier_2 = m_frame1, m_frame2
    #print(len(inlier_1))
    retval, R, t, mask_recoverPose = cv2.recoverPose(E, inlier_1, inlier_2, camMat)
    P = np.hstack((R, t))
    print(f"P: {P}")
    return P, inlier_1, inlier_2

def VO(frame1, frame2):
    ORB_out = run_ORB(frame1, frame2)

    matched_points_frame1 = ORB_out[0]
    matched_points_frame2 = ORB_out[1]
    pts1 = np.array(matched_points_frame1)
    pts2 = np.array(matched_points_frame2)
    print(f"initial shape: {pts1.shape}") # (500, 2) means 500 x/y coords 

    final_img = ORB_out[2]
    mask_img = show_pixels(matched_points_frame1, frame1)
    P2, inlier_1, inlier_2 = get_P(pts1, pts2, K) # Extrinsic matrix of frame 2
    P = P2
    P2 = K @ P2 # Projection matrix
    P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1)))) # Projection  matrix of frame  - assume no rotation/translation

    points_4D = cv2.triangulatePoints(P1 , P2, np.array(inlier_1).T, np.array(inlier_2).T) # gives 4D coords of each point in frame 1 relative to frame 2
    cv2.imshow("Matches", final_img)
    cv2.imwrite("Matches.jpg", final_img)
    points_3D = points_4D.T[:,0:3]/points_4D.T[:,3:]
    return points_3D, P


def seperate_by_angle(points_3D):
    thresh = 10
    angles = np.arctan(np.divide(points_3D.T[0], points_3D.T[2]))
    angles = angles * 180/3.14
    print(f"Angles: {angles}")
    #print(points_3D.T[0])
    #print(points_3D.T[2])
    print(f"First Point: {points_3D[0]}")
    left, center, right = [], [], []
    for i in range(len(angles)):
        if abs(angles[i]) < thresh:
            center.append(points_3D[i])
        elif angles[i] > thresh:
            right.append(points_3D[i])
        else:
            left.append(points_3D[i])
    return np.array(center), np.array(right), np.array(left)

def seperate_by_distance(points_3D, distance_travelled, P):
    t_Z = P[2, 3]
    scale = distance_travelled/t_Z
    distance = []
    for i in range(len(points_3D)):
        distance.append(np.linalg.norm(points_3D[i]))
    distance_scaled = np.multiply(distance, scale)
    distance_sorted = sorted(distance_scaled)
    return distance_sorted



#frame1_raw = cv2.imread('C:/Users/User/Pictures/Saved Pictures/Test1.jpeg')
#frame2_raw = cv2.imread('C:/Users/User/Pictures/Saved Pictures/Test2.jpeg')

frame1_raw = cv2.imread('C:/Users/User/Documents/PlatformIO/Projects/Visual_Odometry/key_close.jpg')
frame2_raw = cv2.imread('C:/Users/User/Documents/PlatformIO/Projects/Visual_Odometry/key_far.jpg')
#frame1 = rescaleFrame(frame1_raw)
#frame2 = rescaleFrame(frame2_raw)
points, P = VO(frame1_raw, frame2_raw)
center, right, left = seperate_by_angle(points)
print(f"L, C, R: {len(left),len(center),len(right)}")


distance = seperate_by_distance(points, 10, P)
print(f"Closest: {distance[0]} cm")
#print(len(points))



cv2.waitKey(0)