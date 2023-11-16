import cv2
import cv2.aruco as aruco

from GetEnemyPixCoords import GetEnemy

ge = GetEnemy()
ge.display()

while True:
    updated, min_id, distance = ge.getClosestEnemy()
    print(ge.getVectorToClosestEnemy())
    # print(ge.getPixCoords()[min_id][0])
