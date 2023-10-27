import cv2
import cv2.aruco as aruco
import numpy as np

# Start the video stream
cap = cv2.VideoCapture(0)

# Get the pixel coordinate range
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Get the center of the camera coordinates
midX = width // 2
midY = width // 2

# List of friendly aruco markers
goodGuys = [18, 5]

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

aruco_parameters = cv2.aruco.DetectorParameters()

# Loop until escape is pressed or the camera is disconnected.
while True:
    ret, frame = cap.read()
    # If the camera disconnects break out of the loop
    if not ret:
        break

    # Convert the video stream to gray image.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Get find the aruco markers.
    corners, ids, rejected = aruco.detectMarkers(
        gray, aruco_dict, parameters=aruco_parameters
    )

    if ids is not None:
        enemyCorners = []
        enemyIds = []
        enemyPixDist = {}
        # Search through each aruco marker if it is an enemy store the details in the lists above.
        for i in range(len(ids)):
            if ids[i][0] not in goodGuys:
                enemyCorners.append(corners[i])
                enemyIds.append([ids[i][0]])
                # Get the pixel distance of the enemy to the center of the camera.
                boxCenterX = (corners[i][0][0][0] + corners[i][0][3][0]) / 2
                boxCenterY = (corners[i][0][0][1] + corners[i][0][3][1]) / 2
                enemyPixDist[ids[i][0]] = (
                    (boxCenterX - midX) ** 2 + (boxCenterY - midY) ** 2
                ) ** (1 / 2)
        # Draw boxes around the enemies and display their ids.
        enemyIds = np.array(enemyIds, dtype=np.int32)
        aruco.drawDetectedMarkers(frame, enemyCorners, enemyIds)

        # Get the enemy that is the closest.
        min_id, min_distances = min(enemyPixDist.items(), key=lambda item: item[1])
        print(max_key)

    cv2.imshow("ArUco Marker Detection", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
