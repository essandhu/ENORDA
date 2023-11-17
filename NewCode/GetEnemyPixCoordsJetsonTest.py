# Import Open CV for Computer Vision
import cv2
import cv2.aruco as aruco
import time 


class GetEnemy:
    # # The dimensions of the camera
    # width = None
    # height = None

    # # The camera
    # cap = None

    # # The pixel dimensions of the camera
    # midX = None
    # midY = None

    # # A list of our team's rovers
    # goodGuys = None

<<<<<<< Updated upstream:NewCode/GetEnemyPixCoords.py
    # The dictionary for the aruco markers
    aruco_dict = None

    aruco_parameters = aruco.DetectorParameters_create()

    # A dictionary of the pixel coordinates of each marker.
    # The aruco marker id is the key
    # The aruco coordinates are the items.
    # The coordinates are stored in the following format to form the bounding box:
    # [[x0, y0], [x1, y1], [x2, y2], [x3, y3]]
    pixCoords = {}
=======
    # # The dictionary for the aruco markers
    # aruco_dict = None
>>>>>>> Stashed changes:NewCode/GetEnemyPixCoordsJetsonTest.py

    def __init__(self, camIndex=1, goodGuys=[18, 5], dictionary=aruco.DICT_4X4_50):
        # Start the video stream
        self.cap = cv2.VideoCapture(camIndex)

        # Get the pixel coordinate range
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Get the center of the camera coordinates
        self.midX = self.width // 2
        self.midY = self.height // 2

        # List of friendly aruco markers
        self.goodGuys = goodGuys

        # Set the aruco marker dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary)

        # Set the aruco parameters
        self.aruco_parameters = aruco.DetectorParameters()

    # display will show the camera and mark all the aruco markers and label them with the ID.
    # this should only be used for testing.
    def display(self):
        while True:
            ret, frame = self.cap.read()

            # If the camera disconnects break out of the loop
            if not ret:
                break

            # Convert the video stream to gray image.
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Find the aruco markers.
            corners, ids, rejected = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_parameters
            )

            # Draw boxes around the enemies and display their IDs.
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow("ArUco Marker Detection", frame)

            # Press escape to exit.
            if cv2.waitKey(1) & 0xFF == 27:
                break

    # getMarkers will find all the aruco markers and add them to the pixCoords dictionary
    # getMarkers will return a boolean variable to indicate if pixCoords was updated or if it was not changed
    def getMarkers(self):
        ret, frame = self.cap.read()

        # If the camera disconnects return False to indicate it was not updated.
        if not ret:
            return False

        # Convert the video stream to gray image.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the aruco markers.
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_parameters
        )
        if ids is not None:
            # Reset pixCoords
            self.pixCoords = {}
            # Search through each aruco marker and store it in pixCoords
            for i in range(len(ids)):
                self.pixCoords[ids[i][0]] = corners[i][0]
            return True
        # If no markers were found return False to indicate there was no update.
        return False

    # getEnemies finds all the markers that are not identified as our own markers.
    # getEnemies returns a boolean variable to indicate if pixCoords was updated
    # getEnemies also returns a list filled with the IDs of the enemy markers found.
    def getEnemies(self):
        # update pixCoords with the marker location
        updated = self.getMarkers()

        # if any markers are found in pixCoords continue.
        if self.pixCoords is not None:
            enemies = []
            # Search through each aruco marker if it is not one of our own store the IDs in the enemies list.
            for k in self.pixCoords.keys():
                if k not in self.goodGuys:
                    enemies.append(k)
        return updated, enemies

    # getClosestEnemy finds the enemy marker that is the closest to the center of the camera.
    def getClosestEnemy(self):
        # find all the enemy markers.
        updated, enemies = self.getEnemies()
        # This will store the enemy ID's as the key and the distance of the enemy to the center to the camera as the item.
        enemyPixDist = {}
        # Search through each enemy and store it to the enemyPixDist dictionary.
        for k in enemies:
            # Get the pixel distance of the enemy to the center of the camera.
            enemyPixDist[k] = self.pixDist(self.pixCoords[k])

            # Get the enemy that is the closest.
            if len(enemyPixDist.items()) > 0:
                min_id, min_distances = min(
                    enemyPixDist.items(), key=lambda item: item[1]
                )
                return updated, min_id, min_distances
        # If no enemy was found return the following.
        return updated, None, None

    # getVectorToMarker returns the x and y values from the center of the camera to the center of the aruco marker box.
    def getVectorToMarker(self, markerID):
        if markerID != None:
            # Get the pixel coordinates of the marker.
            coord = self.pixCoords[markerID]
            # Calculate x and y
            boxCenterX = (coord[0][0] + coord[3][0]) / 2
            boxCenterY = (coord[0][1] + coord[3][1]) / 2
            return (boxCenterX - self.midX), (boxCenterY - self.midY)
        # If an empty marker was submitted return the following.
        return None, None

    # getVectorToClosestEnemy returns a boolean variable indicating if pixCoords was updated.
    # getVectorToClosestEnemy then returns the vector from the center of the camera to the center of the box in the for of a list containing x, y.
    # getVectorToClosestEnemy then returns the distance from the center of the camera to the center of the box
    def getVectorToClosestEnemy(self):
        # get the ID of the closest enemy.
        updated, enemyID, enemyDist = self.getClosestEnemy()
        # return the boolean updated and the x, y values for the vector.
        return updated, self.getVectorToMarker(enemyID), enemyDist

    # pixDist returns the distance from the center of the camera to the center of the aruco box.
    def pixDist(self, corners):
        # calculate the center of the box
        boxCenterX = (corners[0][0] + corners[3][0]) / 2
        boxCenterY = (corners[0][1] + corners[3][1]) / 2
        # return the distance.
        return ((boxCenterX - self.midX) ** 2 + (boxCenterY - self.midY) ** 2) ** (
            1 / 2
        )

    def getPixCoords(self):
        return self.pixCoords

    # When finished run this to close the camera.
    def close(self):
        self.cap.release()