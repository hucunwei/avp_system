import cv2
import numpy as np
from glob import glob
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score

def auto_cluster_grayscale(gray_values):
    """Automatically determine optimal clusters and return centers"""
    X = gray_values.reshape(-1, 1)

    # Try both methods
    elbow_k, _ = find_optimal_clusters(X)
    silh_k, _ = silhouette_analysis(X)

    # Use consensus (or pick one method)
    optimal_k = max(3, min(elbow_k, silh_k))  # Ensure at least 3 clusters

    print(f"Elbow method suggests: {elbow_k} clusters")
    print(f"Silhouette suggests: {silh_k} clusters")
    print(f"Using: {optimal_k} clusters")

    # Final clustering
    kmeans = KMeans(n_clusters=optimal_k, n_init=10)
    kmeans.fit(X)
    return np.sort(kmeans.cluster_centers_.flatten())

def main():
    png_files = glob("./datasets/SUPS/data/labels-scaled/*.png")[:150]
    gray_values = np.concatenate([
        np.dot(cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2RGB),
               [0.299, 0.587, 0.114]).flatten()
        for f in png_files if cv2.imread(f) is not None
    ])

    centers = auto_cluster_grayscale(gray_values)
    print("Automatic cluster centers:", centers.astype(int))