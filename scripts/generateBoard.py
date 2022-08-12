import cv2
from cv2 import aruco

# Create Aruco Board
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
aruco_dict.bytesList=aruco_dict.bytesList[0:,:,:]
board = aruco.GridBoard_create(7, 5, 0.025, 0.005, aruco_dict)

imgBoard = board.draw((1100, 850))
cv2.imwrite("resources/boards/ArucoBoard_7x5_DICT_5X5.png", imgBoard)

# Create CharucoBoard
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_dict.bytesList=aruco_dict.bytesList[0:,:,:]
board = aruco.CharucoBoard_create(7, 5, 0.1, 0.075, aruco_dict)

imgBoard = board.draw((1100, 850))
cv2.imwrite("resources/boards/CharucoBoard_7x5_DICT_6X6.png", imgBoard)