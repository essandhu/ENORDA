import cv2
import numpy as np

# Set the number of inner corners on your calibration pattern (e.g., chessboard)
pattern_size = (9, 6)  # Change this according to your calibration pattern

# Arrays to store object points and image points from all calibration images
obj_points = []  # 3D points in real world space
img_points = []  # 2D points in image plane

# Prepare grid and object points
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

# Create video capture object
cap = cv2.VideoCapture(0)  # Use 0 for the default camera, adjust if needed

while True:
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if ret:
        obj_points.append(objp)
        img_points.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(frame, pattern_size, corners, ret)
        cv2.imshow('Calibration', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the camera
cap.release()
cv2.destroyAllWindows()

print("Getting results")
print(len(obj_points))
print(len(img_points))
print(gray.shape[::-1])
# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

# Print the camera matrix and distortion coefficients
print("Camera Matrix:")
print(mtx)
print("\nDistortion Coefficients:")
print(dist)
