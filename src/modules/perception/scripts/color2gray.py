import numpy as np

# RGB color table
rgb_table = np.array([
    [0, 0, 0],         # Gray 0
    [123, 64, 132],    # Gray 1
    [34, 134, 136],    # Gray 2
    [77, 128, 254],    # Gray 3
    [113, 193, 46],    # Gray 4
    [192, 192, 0],     # Gray 5
    [254, 254, 0],     # Gray 6
    [255, 255, 255]    # Gray 7
], dtype=np.float32)

# RGB to Grayscale conversion weights
weights = np.array([0.299, 0.587, 0.114])

# Compute grayscale values
gray_values = np.dot(rgb_table, weights).round().astype(np.uint8)

print("Grayscale values:", gray_values.tolist())
