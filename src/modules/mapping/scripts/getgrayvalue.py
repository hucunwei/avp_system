import cv2
import numpy as np

# Load grayscale image (CV_8UC1)
gray_img = cv2.imread("label_54.png", cv2.IMREAD_GRAYSCALE)

# Convert to color image so we can draw colored text on it
overlay_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)

# Set font and size
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.4
color = (0, 255, 0)  # Green text
thickness = 1

# Sample every N pixels (for readability)
step = 10  # Change this to control text density

for y in range(0, gray_img.shape[0], step):
    for x in range(0, gray_img.shape[1], step):
        val = int(gray_img[y, x])  # Grayscale value
        text = str(val)
        if val > 150 and val < 155:
            cv2.putText(overlay_img, text, (x, y), font, font_scale, color, thickness)

# Display the image
cv2.imshow("Gray Image with Values", overlay_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
