#This is a junk program to test the computer vision programs

import cv2
import cv2.aruco as aruco
import numpy as np

# from GetEnemies import GetEnemy
from GetEnemies import GetEnemy

ge = GetEnemy()
ge.display()
# camI = [[np.pi, 0, 0], [0, 0, 0]]
# while True:
    # _, id, coord = ge.getClosestEnemyGlobal(camI)
    # _, speed, vel, _ = ge.getVelocity(42, camI)
    # _, vel, time = ge.getVelTimer(42, camI, 0.25)
    # print(speed)
    # if speed != None and speed > 0.1:
        # print(speed, vel)
#     # print(ge.getCameraIntrinsicsForMarker(5)[1][1][0][0])
#     print(min_id, ge.getCameraIntrinsicsForMarker(min_id))
# print(ge.getPixCoords()[min_id][0])
