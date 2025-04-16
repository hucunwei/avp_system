import os
import csv

# Set your folder paths
images_folder = './datasets/SUPS/data/images'
labels_folder = './datasets/SUPS/data/labels'
output_csv = 'file_list.csv'

# Get all file names in images folder
file_names = [f for f in os.listdir(images_folder) if os.path.isfile(os.path.join(images_folder, f))]

# Optional: sort file names
file_names.sort()

# Write to CSV with both image and label paths
with open(output_csv, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for file in file_names:
        image_path = os.path.join(images_folder, file)
        label_path = os.path.join(labels_folder, file)
        writer.writerow([image_path, label_path])

print(f"Saved {len(file_names)} image-label pairs to {output_csv}")
