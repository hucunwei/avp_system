import os
import numpy as np
from PIL import Image
from sklearn.cluster import KMeans
# import matplotlib.pyplot as plt

# Step 1: Load the first 100 PNG images from the directory
image_dir = './datasets/SUPS/data/labels-scaled/'
image_files = [f for f in os.listdir(image_dir) if f.endswith('.png')]

# Limit to first 100 files
image_files = image_files[:200]

# Step 2: Convert each image to grayscale and collect all pixel values
all_pixels = []

for image_file in image_files:
    image_path = os.path.join(image_dir, image_file)
    img = Image.open(image_path).convert('L')  # Convert to grayscale ('L' mode)
    img_array = np.array(img).flatten()  # Flatten the image to a 1D array of pixel values
    img_array = img_array[(img_array > 0) & (img_array < 255)]
    all_pixels.append(img_array)

# Convert all pixels into a single numpy array
# # 💡 Remove near-black and near-white values

all_pixels = np.concatenate(all_pixels)


# Step 3: Apply k-means clustering to the pixel values
kmeans = KMeans(n_clusters=8, random_state=42)
kmeans.fit(all_pixels.reshape(-1, 1))  # Reshape to fit KMeans model

# Step 4: Get the centroids (gray scale values)
centroids = np.sort(kmeans.cluster_centers_.flatten())

# Output the centroids (sorted grayscale values)
print("Grayscale Centroids (8 clusters):", centroids)

# # Optional: Visualize the centroids as grayscale values
# plt.figure(figsize=(8, 1))
# plt.imshow([centroids], cmap='gray', aspect='auto')
# plt.axis('off')
# plt.show()
