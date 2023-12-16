#####################################################################################
#                This program is used to calibrate the drones camera                #
# This will probably not work on the drone but it can be run on a laptop running the#
# latest python3 version                                                            #
#               Run this program before using the GetEnemies program                #
#     Hold the provided 7x10 checker board in front of the camera and hold it at    #
#     different positions and orientations                                          #
#  Copy the printed results and initialize the  cameraMatrix and distortionCoef     # 
#  variables in the GetEnemies program with the copied results.                     #
#   This program will not run on the Jetson. Instead connect the drone camera to a  #
#   laptop and run the program.                                                     #
#####################################################################################


import cv2
import numpy as np

frameI = 0
captureRate = 5

# Set the following variable to the number corners on the checker board not including
#  the outer corner
# In other words set patter_size = (w-1, h-1) where w is the number of white and black
#  squares along the width and h is number white and black square along the height.
pattern_size = (9, 6) 

# Array to store 3d object points from all calibration images
obj_points = []
# Array to store 2d object points from all calibration images
img_points = []

# Prepare grid and object points
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)

# Create video capture object for the default camera 0
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    # If the chessboard was found proceed
    if ret:
        # This adds a delay between updates for obj_points and img_points to decrease
        # the calculation time and ensure more photos from different angles and
        # distances can be taken without slowing the computer.
        # Increase the captureRate to decrease the computation time and decrease the sample rate.
        # Decrease the captureRate to increase the computation time and increase the sample rate. 
        if frameI >= captureRate:
            obj_points.append(objp)
            img_points.append(corners)
            frameI = -1
        frameI += 1

        # Draw and display the corners
        cv2.drawChessboardCorners(frame, pattern_size, corners, ret)
        cv2.imshow("Calibration", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

# Release the camera
cap.release()
cv2.destroyAllWindows()

print("Getting results")
print(len(obj_points))
print(len(img_points))
# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray.shape[::-1], None, None
)

# Print the camera matrix and distortion coefficients
# Copy these results and initialize cameraMatrix and distortionCoef with these results

print("Camera Matrix:")
print("np.array([", end="")
for i in range(len(mtx)):
    print("[", end="")
    print(mtx[i][0], end="")
    for j in range(1, len(mtx[i])):
        print(", ", end=str(mtx[i][j]))
    if i == len(mtx) - 1:
        print("]", end="")
    else:
        print("], ", end="")
print("])")

print("\nDistortion Coefficients:")
print("np.array([[", end="")
print(dist[0][0], end="")
for j in range(1, len(dist[0])):
    print(", ", end=str(dist[0][j]))
print("]])")
