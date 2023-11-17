import cv2
import cv2.aruco as aruco
import numpy as np

from GetEnemies import GetEnemy

ge = GetEnemy()
# ge.display()
camI = [[np.pi, 0, 0], [0, 0, 0]]
while True:
    ge.getGlobalCoordinates(42, camI)
#     # print(ge.getCameraIntrinsicsForMarker(5)[1][1][0][0])
#     print(min_id, ge.getCameraIntrinsicsForMarker(min_id))
# print(ge.getPixCoords()[min_id][0])
