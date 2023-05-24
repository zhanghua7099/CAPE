import matplotlib.pyplot as plt
import cv2

img = cv2.imread("depth_0.png", cv2.IMREAD_ANYDEPTH)
plt.imshow(img)
plt.show()

