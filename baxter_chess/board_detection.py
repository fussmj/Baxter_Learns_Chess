#!/usr/bin/python2
import cv2
import numpy as np

image = cv2.imread('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/real_before.png')
#print(image.shape) # Print image shape
cv2.imshow("original", image)

height, width = image.shape[:2]
center = (width/2, height/2)
rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=1.5, scale=1)
rotated_image = cv2.warpAffine(src=image, M=rotate_matrix, dsize=(width, height))

cv2.imshow("rotated", rotated_image)

# Cropping an image
cropped_image = rotated_image[104:394, 210:509]

# Display cropped image
cv2.imshow("cropped", cropped_image)

# Save the cropped image
cv2.imwrite('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/crop.png', cropped_image)

cv2.waitKey(0)
cv2.destroyAllWindows()