import cv2
import numpy as np
from glob import glob
from sklearn.cluster import KMeans

def extract_grayscale_values(png_files, max_files=200):
    """Extract all grayscale pixel values from images"""
    all_gray_values = []
    for i, file in enumerate(png_files[:max_files]):
        img = cv2.imread(file)
        if img is None:
            continue

        # Convert to grayscale using perceptual weights
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        grayscale = np.dot(img_rgb, [0.299, 0.587, 0.114]).flatten()
        all_gray_values.extend(grayscale)

    return np.array(all_gray_values)

def cluster_grayscale_values(gray_values, n_clusters=8):
    """Cluster grayscale values into n classes using K-means"""
    if len(gray_values) == 0:
        return np.array([]), np.array([])

    # Reshape for K-means (samples must be 2D)
    X = gray_values.reshape(-1, 1)

    # Perform K-means clustering
    kmeans = KMeans(n_clusters=min(n_clusters, len(gray_values)), n_init=20)
    kmeans.fit(X)

    # Get cluster centers and labels
    centers = kmeans.cluster_centers_.flatten()
    labels = kmeans.labels_

    return centers, labels

def main():
    # 1. Get all grayscale values from images
    png_files = glob("./datasets/SUPS/data/labels-scaled/*.png")
    gray_values = extract_grayscale_values(png_files, max_files=200)

    if len(gray_values) == 0:
        print("No valid images processed!")
        return

    # # 💡 Remove near-black and near-white values
    # gray_values = gray_values[(gray_values > 5) & (gray_values < 250)]

    if len(gray_values) == 0:
        print("No valid grayscale values after filtering!")
        return

    print(f"Collected {len(gray_values)} grayscale values from images")

    # 2. Cluster into 8 classes
    centers, labels = cluster_grayscale_values(gray_values, n_clusters=8)

    # 3. Print results
    print("\nCluster centers (grayscale values):")
    print(np.sort(centers.astype(int)))

    # Optional: Print distribution
    unique, counts = np.unique(labels, return_counts=True)
    print("\nCluster distribution:")
    for center, count in zip(centers[unique], counts):
        print(f"Class {np.where(centers == center)[0][0]}: {count} pixels (center: {int(center)})")

if __name__ == "__main__":
    main()