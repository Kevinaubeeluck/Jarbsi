from picamera2 import Picamera2
import time
import numpy as np
import cv2
import io
import requests

K = np.array([[2.49958316e+03, 0.00000000e+00,  9.92922387e+02],
 [0.00000000e+00, 2.50636951e+03, 7.42220253e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) # High Res Camera Intrinsic Matrix

#K = np.array([[1.25048727e+03, 0.00000000e+00, 3.20442247e+02],
# [0.00000000e+00, 1.25140777e+03, 1.86992519e+02],
# [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) # Low Res Camera Intrinsic Matrix

FLASK_SERVER_URL = "http://192.168.92.215:8000/api/upload_frame"
picam2 = Picamera2()
fps = 1
resolution = (1920, 1080)
#camera_config = picam2.create_still_configuration(main={"size": resolution}, lores={"size": resolution}, display="lores", transform = {"scale" =})
camera_config = picam2.create_video_configuration(
    main={"size": resolution},
    lores={"size": resolution},
    display="lores"
)
picam2.configure(camera_config)
picam2.set_controls({"FrameDurationLimits": [int(1000000/fps), int(1000000/fps)]})  # change to 1FPS
picam2.start()
time.sleep(2)

def send_frame(image_stream): # GPT'D PLEASE CHANGE
    """Sends a single frame to the Flask server."""
    image_stream.seek(0)
    files = {'frame': ('image.jpg', image_stream, 'image/jpeg')}
    try:
        response = requests.post(FLASK_SERVER_URL, files=files)
        response.raise_for_status()  # Raise an exception for bad status codes
        print("Frame sent successfully")
    except requests.exceptions.RequestException as e:
        print(f"Error sending frame: {e}")

def send_numpy_frame(numpy_image, server_url): # GPT'D PLEASE CHANGE
    """
    Encodes a NumPy array (BGR image) to JPEG format in memory 
    and sends it to a web server.
    """
    
    if numpy_image is None:
        print("No matches")
        return
    
    numpy_image = cv2.cvtColor(numpy_image, cv2.COLOR_BGR2RGB)
    is_success, buffer = cv2.imencode(".jpg", numpy_image)
    if not is_success:
        print("Couldn't convert to jpg")
        return

    files = {'frame': ('image.jpg', buffer.tobytes(), 'image/jpeg')}
    try:
        response = requests.post(server_url, files=files, timeout=5)
        response.raise_for_status()
        print("Frame sent successfully")
    except requests.exceptions.RequestException as e:
        print(f"Error sending frame: {e}")

def radial_expansion_normalized(matched1, matched2, frame_shape):
    cx, cy = frame_shape[1] // 2, frame_shape[0] // 2
    expansion_ratios = []

    for (x1, y1), (x2, y2) in zip(matched1, matched2):
        r1 = np.linalg.norm([x1 - cx, y1 - cy])
        r2 = np.linalg.norm([x2 - cx, y2 - cy])
        if r1 > 1:  # avoid division by zero
            expansion_ratios.append((r2 - r1) / r1)

    return np.array(expansion_ratios)

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

    queryDescriptors = np.array(queryDescriptors).astype(np.float32)
    trainDescriptors = np.array(trainDescriptors).astype(np.float32)
    #print(queryKeypoints)
    #print(trainKeypoints)
    # Initialize the Matcher for matching
    # the keypoints and then match the
    # keypoints
    matcher = cv2.BFMatcher() # Brute Force Matcher
    #print(trainDescriptors.shape)
    try:
        if queryDescriptors.shape[1] == trainDescriptors.shape[1]:  
            #matches = matcher.match(queryDescriptors, trainDescriptors)
            matches = matcher.knnMatch(queryDescriptors, trainDescriptors, k=2) 
            if np.array(matches).shape[1] == 2:
                good_matches = []
                for m, n in matches:
                    if m.distance < 0.8 * n.distance:
                        good_matches.append(m)  
                
                matches = sorted(good_matches[:50], key=lambda x: x.distance) # sort by distance
                matched_points_frame1 = [queryKeypoints[m.queryIdx].pt for m in matches]
                matched_points_frame2 = [trainKeypoints[m.trainIdx].pt for m in matches]
                final_img = cv2.drawMatches(frame1, queryKeypoints, frame2, trainKeypoints, good_matches[:50],None)
                final_img = cv2.resize(final_img, (1000,650))

                return (matched_points_frame1, matched_points_frame2, final_img)
            else:
                return ((0,0), (0,0), None)
        else:
                return ((0,0), (0,0), None)
    except IndexError:
        return ((0,0), (0,0), None)


def get_P(m_frame1, m_frame2, camMat):
    E, mask = cv2.findEssentialMat(m_frame1, m_frame2, camMat, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    try:
        if E.any() != None and E.shape==(3, 3):
            #add inlier filtering
            inlier_1 = m_frame1[mask.ravel() == 1]
            inlier_2 = m_frame2[mask.ravel() == 1]
            #inlier_1, inlier_2 = m_frame1, m_frame2
            #print(len(inlier_1))
            retval, R, t, mask_recoverPose = cv2.recoverPose(E, inlier_1, inlier_2, camMat)
            P = np.hstack((R, t))
            print(f"P: {P}")
            return True, P, inlier_1, inlier_2
        else:
            print('E 0x0')
            return False, np.hstack((np.eye(3), np.zeros((3, 1)))), m_frame1, m_frame2
    except AttributeError:
        print('E none')
        return False, np.hstack((np.eye(3), np.zeros((3, 1)))), m_frame1, m_frame2

def VO(matched_points_frame1, matched_points_frame2):
    pts1 = np.array(matched_points_frame1)
    pts2 = np.array(matched_points_frame2)
    print(pts1.shape[0]) # (500, 2) means 500 x/y coords 
    
    if pts1.shape[0] > 2:
        #final_img = ORB_out[2]
        Error, P2, inlier_1, inlier_2 = get_P(pts1, pts2, K) # Projection matrix of frame 2
        P = P2
        P2 = K @ P2
        P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1)))) # Projection  matrix of frame  - assume no rotation/translation

        points_4D = cv2.triangulatePoints(P1 , P2, np.array(inlier_1).T, np.array(inlier_2).T) # gives 4D(??) coords of each point in frame 1 relative to frame 2
        #cv2.imshow("Matches", final_img)
        points_3D = points_4D.T[:,0:3]/points_4D.T[:,3:]
        expansion_ratios = radial_expansion_normalized(inlier_1, inlier_2, sorted(resolution))
        return Error, points_3D, P, expansion_ratios
    else:
        return False, np.zeros((3, 1)), np.zeros((4, 3)), []

def seperate_by_angle(points_3D):
    thresh = 7.5
    angles = np.arctan(np.divide(points_3D.T[0], points_3D.T[2]))
    angles = angles * 180/3.14
    #print(f"Angles: {angles}")
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
    return angles, np.array(center), np.array(right), np.array(left)

frame_far = picam2.capture_array()
#time.sleep(1/fps)

while True:
    frame_close = picam2.capture_array()
    matched_points_frame1, matched_points_frame2, match_img = run_ORB(frame_close, frame_far)
    found, points_3D, P, expansion_ratios = VO(matched_points_frame1, matched_points_frame2)
    if found:
        angles, C, R, L = seperate_by_angle(points_3D)
        print(f"L, C, R: {len(L)}, {len(C)}, {len(R)}")
        center, left, right = 0, 0, 0
        approaching_angles = angles[expansion_ratios.ravel() < -0.1]
        approaching_angles = np.array(approaching_angles)
        center = len(approaching_angles[(approaching_angles.ravel() < 7.5) & (approaching_angles.ravel() > -7.5)])
        right = len(approaching_angles[approaching_angles.ravel() > 7.5])
        left = len(approaching_angles[approaching_angles.ravel() < -7.5])
        print(f"Obst Left:{left}, Obst Center:{center}, Obst Right:{right}")
    else:
        print("Failed :c")
    send_numpy_frame(match_img, FLASK_SERVER_URL)
    frame_far = frame_close
    #time.sleep(1/fps)