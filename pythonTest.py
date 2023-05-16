import cv2
import numpy as np

# Initialize the camera
cap = cv2.VideoCapture(0)


# Load the Aruco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
markerLength = 100

# Initialize the Aruco detector parameters
aruco_params = cv2.aruco.DetectorParameters()
cameraMatrix = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape((3, 3))
distortionCoeffs = np.array([0, 0, 0, 0, 0])

# Define the ID of the Aruco marker to detect
marker_id = 3

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Detect the Aruco marker
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

    # If the Aruco marker is detected, estimate its pose
    if ids is not None and marker_id in ids:
        ret = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distortionCoeffs)

        rvecs = ret[0]
        print(rvecs)
        tvecs = ret[1]
        print(tvecs)
        # Calculate the rotation matrix and translation vector of the camera
        rmat, _ = cv2.Rodrigues(rvecs)
        tvec = np.transpose(tvecs)
        inv_rmat = np.transpose(rmat)
        print(inv_rmat)
        print(-tvec)
        print(tvec.size)
        inv_tvec = np.matmul(inv_rmat, -tvec)

        # Calculate the relative rotation and translation between the camera and the Aruco marker
        rel_rmat, rel_tvec = cv2.composeRT(rmat, tvec, inv_rmat, inv_tvec)

        # Calculate the distance and angle between the camera and the Aruco marker
        dist = np.linalg.norm(rel_tvec)
        angle = np.arctan2(rel_tvec[0][0], rel_tvec[2][0]) * 180 / np.pi

        # Output the distance and angle to the user
        print("Distance to marker:", dist)
        print("Angle to marker:", angle)

    # Display the image
    cv2.imshow("Frame", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
