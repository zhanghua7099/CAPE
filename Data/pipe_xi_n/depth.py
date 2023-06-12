import cv2

img = cv2.imread("depth_0.png")
img = 100*img
cv2.imwrite("1.png", img)