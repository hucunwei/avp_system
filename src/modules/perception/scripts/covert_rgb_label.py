import cv2
import numpy as np
from glob import glob
import os

def rgb_to_class_map(rgb_image, class_centers):
    """
    Convert RGB image to class-indexed image (0-7)
    Args:
        rgb_image: Input RGB image (H,W,3)
        class_centers: Precomputed grayscale class centers from previous step
    Returns:
        class_map: Output image (H,W) with values 0-7
    """
    # Convert to grayscale using perceptual weights
    grayscale = np.dot(rgb_image, [0.299, 0.587, 0.114])

    # Find closest class center for each pixel
    distances = np.abs(grayscale[..., np.newaxis] - class_centers)
    class_map = np.argmin(distances, axis=2)

    return class_map.astype(np.uint8)

if __name__ == "__main__":
    # Create output directory
    output_dir = "./datasets/SUPS/data/labels-test"
    os.makedirs(output_dir, exist_ok=True)

    png_files = glob("./datasets/SUPS/data/labels-scaled/*.png")
    # class_centers = np.array([15, 47, 79, 111, 143, 175, 207, 239])  # Example centers
    class_centers = np.array([0, 89, 104, 127, 152, 170, 225, 254])  # Example centers
    png_files = png_files[:4]
    for file in png_files:  # Process first 20 files
        # Load image
        input_image = cv2.imread(file)
        if input_image is None:
            print(f"Warning: Could not read {file}, skipping")
            continue

        rgb_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)

        # Convert to class map
        class_map = rgb_to_class_map(rgb_image, class_centers)

        # Save results
        output_path = os.path.join(output_dir, os.path.basename(file))

        # Option 1: Save raw class indices (0-7)
        cv2.imwrite(output_path, class_map)  # Will be very dark
        # # Option 2: Save scaled version for visualization
        cv2.imwrite(output_path.replace(".png", "_scaled.png"), class_map * 32)
