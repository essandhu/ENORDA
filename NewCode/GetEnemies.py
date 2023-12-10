# Import Open CV for Computer Vision
import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation
import time


class GetEnemy:
    # The dimensions of the camera
    width = None
    height = None

    # The camera
    cap = None

    # The pixel dimensions of the camera
    midX = None
    midY = None

    # A list of our team's rovers
    goodGuys = None

    # The dictionary for the aruco markers
    aruco_dict = None

    aruco_parameters = cv2.aruco.DetectorParameters()

    # A dictionary of the pixel coordinates of each marker.
    # The aruco marker id is the key.
    # The aruco coordinates are the items.
    # The coordinates are stored in the following format to form the bounding box:
    # [[x0, y0], [x1, y1], [x2, y2], [x3, y3]]
    pixCoords = {}

    # A dictionary of the coordinates and orientation of each marker relative to the camera in meters.
    # The aruco marker id is the key.
    # The aruco coordinates and orientation are the items.
    camCoordinates = {}

    # A dictionary of the global coordinates of each marker in meters.
    # The aruco marker id is the key.
    # The aruco coordinates are the items.
    globalCoords = {}

    # A dictionary which stores if each marker was updates the last time the camera was checked.
    # The aruco marker id is the key.
    # A boolean of if the markers are updated are the items.
    updates = {}

    # This is the width of the aruco marker in meters.
    # This is used to determine the distance to the marker.
    markerWidth = None

    # A dictionary of the necesary information for calculating the velocity of each marker.
    # The aruco marker id is the key.
    # The items include the following:
    # The global coordinates that were last recorded when the velocity of the marker was calculated.
    # The time in seconds when the last velocity was calculated for the marker.
    # The magnitude of the velocity when the last velocity was calculated for the marker.
    # The velocity when the last velocity was calculated for the marker.
    # The change in time between when the second to last velocity was calculated and the last velocity was calculated.
    # The items are stored in a list in the following format.
    # [globalCoordinates, time, speed, velocity, deltaTime]
    lastVelocity = {}

    # Use camera calibration program to get the cameraMatrix and the distortionCoef.
    cameraMatrix = np.array([[544.2900538666688, 0.0, 318.2822285358641], [0.0, 545.8874796323947, 254.04757006428002], [0.0, 0.0, 1.0]])

    distortionCoef = np.array([[-0.29295405447282924, 3.71618606448295, 0.0021801641569121222, 0.012314010977921836, -12.551057852648873]])

    def __init__(
        self,
        camIndex=0,
        goodGuys=[18, 5],
        dictionary=aruco.DICT_4X4_50,
        markerWidth=0.0508,
        cameraMatrix=None,
        distortionCoef=None,
    ):
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

        # Set the width of the aruco marker
        self.markerWidth = markerWidth

        # Set the cameraMatrix
        if cameraMatrix != None:
            self.cameraMatrix = cameraMatrix

        # Set the distortionCoef
        if distortionCoef != None:
            self.distortionCoef = distortionCoef

    # display will show the camera and mark all the aruco markers and label them with the ID.
    # This should only be used for testing and it is not capable of performing any other action until this is terminated.
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

            # Draw boxes around the enemies and display their ids.
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow("ArUco Marker Detection", frame)
            if ids is not None:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.markerWidth, self.cameraMatrix, self.distortionCoef
                )
                camI = [[np.pi, 0, 0], [0, 0, 0]]
                _, id, coord = self.getClosestEnemyGlobal(camI)
                _, speed, vel, _ = self.getVelocity(id, camI)
                if speed != None and speed > 0.1:
                    print("\nMarker ID")
                    print(id)
                    print("Camera Intrinsics in Meters")
                    print(coord)
                    print("Speed and Velocity")
                    print(speed, "m/s", vel)

            # Press escape to exit.
            if cv2.waitKey(1) & 0xFF == 27:
                break

    # updateMarkers will find all the aruco markers and add them to the pixCoords dictionary
    # updateMarkers will return a boolean variable to indicate if pixCoords was updated or if it was not changed
    def updateMarkers(self):
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
            for i in self.updates:
                self.updates[i] = False
            # Reset pixCoords
            self.pixCoords = {}
            self.camCoordinates = {}
            self.globalCoords = {}
            # Search through each aruco marker and store it in pixCoords
            for i in range(len(ids)):
                self.updates[ids[i][0]] = True
                self.pixCoords[ids[i][0]] = corners[i][0]
                #This gets the coordinates of the marker relative to the camera and the orientation of the marker relative to the camera.
                self.camCoordinates[ids[i][0]] = aruco.estimatePoseSingleMarkers(
                    corners[i], self.markerWidth, self.cameraMatrix, self.distortionCoef
                )
            return True
        # If no markers were found return False to indicate there was no update.
        return False

    # getEnemies finds all the markers that are not identified as our own markers.
    # getEnemies returns a boolean variable to indicate if the location of the marker was updated.
    # getEnemies also returns a list filled with the IDs of the enemy markers found.
    def getEnemies(self):
        # update pixCoords with the marker location
        updated = self.updateMarkers()

        # if any markers are found in pixCoords continue.
        if self.pixCoords is not None:
            enemies = []
            # Search through each aruco marker if it is not one of our own store the IDs in the enemies list.
            for k in self.pixCoords.keys():
                if k not in self.goodGuys:
                    enemies.append(k)
        return updated, enemies

    # getClosestEnemy2D finds the enemy marker that is the closest to the center of the camera based on pixel coordinates.
    # The first element returned is a boolean variable to indicate if the location of the marker was updated.
    # The second element returned is the id of the closest aruco marker based on the pixel coordinates.
    # The final element returned is a dictionary of the pixel distance from the center of the camera to each enemy aruco marker. The aruco marker is the key and the pixel distance is the item.
    def getClosestEnemy2D(self):
        # find all the enemy markers.
        _, enemies = self.getEnemies()
        # This will store the enemy ID's as the key and the distance of the enemy to the center to the camera as the item.
        enemyPixDist = {}
        # Search through each enemy and store the pixel distance to the enemyPixDist dictionary.
        for k in enemies:
            # Get the pixel distance of the enemy to the center of the camera.
            enemyPixDist[k] = self.pixDist(self.pixCoords[k])

        # Get the enemy that is the closest.
        if len(enemyPixDist.items()) > 0:
            min_id, min_distances = min(
                enemyPixDist.items(), key=lambda item: item[1]
            )
            return self.updates[min_id], min_id, min_distances
        # If no enemy was found return the following.
        return False, None, min_distances

    # getClosestEnemy finds the enemy marker that is the closest to the center camera using the coordinates relative to the camera.
    # The first element returned is a boolean variable to indicate if the location of the marker was updated.
    # The second element returned is the id of the closest aruco marker.
    # The third element returned is the coordinates relative to the camera of the closest aruco marker.
    # The final element returned is a dictionary of the pixel distance from the center of the camera to each enemy aruco marker. The aruco marker is the key and the distance in meters is the item.
    def getClosestEnemy(self):
        # find all the enemy markers.
        _, enemies = self.getEnemies()
        # This will store the enemy ID's as the key and the distance in meters of the enemy to the center to the camera as the item.
        enemiesDist = {}
        # Search through each enemy and store the distance in meters to the enemiesDist dictionary.
        for e in enemies:
            # Get the distance in meters of the enemy to the center of the camera.
            enemiesDist[e] = self.getDist(self.camCoordinates[e])

        # Get the enemy that is the closest.
        if len(enemiesDist.items()) > 0:
            min_id, min_dist = min(enemiesDist.items(), key=lambda item: item[1])
            return self.updates[min_id], min_id, min_dist, enemiesDist
        
        # If no enemy was found return the following.
        return False, None, None, enemiesDist
    

    # getClosestEnemyGlobal finds the enemy marker that is the closest to the center camera using the coordinates relative to the camera and returns the global coordinates of the marker.
    # getClosestEnemyGlobal requires a parameter including the camera intrinsics.
    # The first element in the camIntrinsics list should be the orientation of the camera in radians.
    # The second element in the camIntrinsics list should be the location of the camera in meters.
    # The first element returned is a boolean variable to indicate if the location of the marker was updated.
    # The second element returned is the id of the closest aruco marker.
    # The third element returned is the global coordinates of the closest aruco marker.
    def getClosestEnemyGlobal(self, camI):
        updated, id, _, _ = self.getClosestEnemy()
        _, coord = self.getGlobalCoordinates(id, camI)
        return updated, id, coord

    # getPixVectorToMarker returns the x and y values from the center of the camera to the center of the aruco marker box based on the pixel coordinates.
    # It first requires an aruco marker id as input.
    # The first element returned is the x value based on the pixel coordinates.
    # The second element returned is the y value based on the pixel coordinates.
    def getPixVectorToMarker(self, markerID):
        if markerID != None:
            # Get the pixel coordinates of the marker.
            coord = self.pixCoords[markerID]
            # Calculate x and y
            boxCenterX = (coord[0][0] + coord[3][0]) / 2
            boxCenterY = (coord[0][1] + coord[3][1]) / 2
            return (boxCenterX - self.midX), (boxCenterY - self.midY)
        # If an empty marker was submitted return the following.
        return None, None

    # getCameraCoordinatesForMarker is given an aruco marker id and returns the camera coordinates of the id.
    # The first element returned indicates if the marker was updated.
    # The final element returned is the camera coordinates of the marker.
    def getCameraCoordinatesForMarker(self, enemyID):
        #This gets the locations of the aruco markers.
        _ = self.updateMarkers()
        return self.updates.get(enemyID, False), self.camCoordinates.get(enemyID, None)

    # getGlobalCoordinates is returns the global coordinates of an aruco marker.
    # The id of the marker is required for the first parameter.
    # The camera intrisics are required in the form of a list for the second parameter.
    # The first element in the camIntrinsics list should be the orientation of the camera in radians.
    # The second element in the camIntrinsics list should be the location of the camera in meters.
    # The first element returned is a boolean variable to indicate if the location of the marker was updated.
    # The second element returned is a list containing the global location of the marker in meters.
    def getGlobalCoordinates(self, ID, camIntrinsics):
        # Check the camera and get the camera coordinates of the for the marker.
        updated, objCoord = self.getCameraCoordinatesForMarker(ID)

        # If the marker is found procede.
        if objCoord != None:
            #Get the coordinates of the marker relative to the camera.
            X = objCoord[1][0][0]

            #Get the global coordinates of the marker using the formula RX + t
            globalCoord = (
                np.dot(Rotation.from_rotvec(camIntrinsics[0]).as_matrix(), X)
                + camIntrinsics[1]
            )
            return updated, globalCoord
        return None, None

    # getVectorToClosestEnemy returns a boolean variable indicating if pixCoords was updated.
    # getVectorToClosestEnemy then returns the vector from the center of the camera to the center of the box in the for of a list containing x, y.
    def get2DVectorToClosestEnemy(self):
        # get the ID of the closest enemy.
        updated, enemyID, _ = self.getClosestEnemy2D()
        # return the boolean updated and the x, y values for the vector.
        return updated, self.getVectorToMarker(enemyID)
    
    # getVelocity gets the speed and velocity of an aruco marker.
    # An id of the marker is required for the first parameter.
    # The camera intrisics are required in the form of a list for the second parameter.
    # The first element in the camIntrinsics list should be the orientation of the camera in radians.
    # The second element in the camIntrinsics list should be the location of the camera in meters.
    # The first element returned is a boolean variable to indicate if the location of the marker was updated.
    # The second element returned is the magnitude of the velocity.
    # The third element returned is the velocity in m/s.
    # The final element returned is the delta time in seconds.
    def getVelocity(self, ID, camIntrinsics):
        # Get the global coordinates of the marker.
        updated, newCoord = self.getGlobalCoordinates(ID, camIntrinsics)
        # If the velocity of the marker has not already been recorded store the time and coordinates of the marker in the lastVelocity dictionary.
        if ID not in self.lastVelocity:
            if updated != None:
                self.lastVelocity[ID] = (newCoord, time.perf_counter(), None, None, None)
            return None, None, None, None
        # If the velocity of the marker has been stored previously procede.
        elif updated == True:
            # Get the data relative to the marker that was stored in the lastVelocity dictionary.
            lastVelocity = self.lastVelocity[ID]
            # Get the time.
            newTime = time.perf_counter()
            # Calculate the delta time.
            deltaTime = newTime - lastVelocity[1]
            # Get the distance vector between the last recorded position and the current recorded position.
            dif = newCoord-lastVelocity[0]
            # Calculate the velocity.
            newVelocity = dif/deltaTime
            # Calculate the distance between the last recorded position and the current recorded position.
            dist = ((dif[0]**2)+(dif[1]**2)+dif[2]**2)**0.5
            # Calculate the speed (aka the magnitude of the velocity)
            newSpeed = dist / deltaTime
            # Store the new data in the lastVelocity dictionary.
            self.lastVelocity[ID] = (newCoord, newTime, newSpeed, newVelocity, deltaTime)
            
            return updated, newSpeed, newVelocity, deltaTime
        else:
            #Return this if the data was not updated.
            return updated, self.lastVelocity[ID][2], self.lastVelocity[ID][3], self.lastVelocity[ID][4]
    
    # getVelTimer gets the speed and velocity of an aruco marker and sets a specified amount for the delta time.
    # The actual delta time may be larger than the requested delta time based on the amount time it takes to make the calculation.
    # An id of the marker is required for the first parameter.
    # The camera intrisics are required in the form of a list for the second parameter.
    # The first element in the camIntrinsics list should be the orientation of the camera in radians.
    # The second element in the camIntrinsics list should be the location of the camera in meters.
    # The final parameter is the specified amount of time that should pass between the first location being noted and the last location being noted for calculating the velocity.
    # The first element returned is a boolean variable to indicate if the location of the marker was updated.
    # The second element returned is the magnitude of the velocity.
    # The third element returned is the velocity in m/s.
    # The final element returned is the delta time in seconds.
    def getVelTimer(self, ID, camIntrinsics, timer):
        while ID not in self.lastVelocity:
            _, _, _, _  = self.getVelocity(ID, camIntrinsics)
        time.sleep(timer)
        return self.getVelocity(ID, camIntrinsics)

    # pixDist returns the distance from the center of the camera to the center of the aruco box.
    def pixDist(self, corners):
        # calculate the center of the box
        boxCenterX = (corners[0][0] + corners[3][0]) / 2
        boxCenterY = (corners[0][1] + corners[3][1]) / 2
        # return the distance.
        return ((boxCenterX - self.midX) ** 2 + (boxCenterY - self.midY) ** 2) ** (
            1 / 2
        )
    
    # This calculates the magnitude of a 3D vetor.
    def getDist(self, intrinsics):
        return (
            (intrinsics[1][0][0][0] ** 2)
            + (intrinsics[1][0][0][1] ** 2)
            + (intrinsics[1][0][0][2] ** 2)
        ) ** 0.5

    # This returns the pixel coordinates.
    def getPixCoords(self):
        return self.pixCoords
    
    # This returns the a list of coordinates for each marker relative to the camera.
    def getCamCoordinates(self):
        return self.camCoordinates
    
    # This returns the global coordinates.
    def getCoords(self):
        return self.globalCoords
    
    # This returns the boolean values that indicate if the marker was updated the last time the camera was checked.
    def getUpdates(self):
        return self.updates
    
    # When finished run this to close the camera.
    def close(self):
        self.cap.release()
