import os
import csv

images_folder = './datasets/SUPS/data/images'
labels_folder = './datasets/SUPS/data/labels'
output_csv = 'common_files.csv'

# Get base filenames (without extensions if needed)
image_files = set(os.listdir(images_folder))
label_files = set(os.listdir(labels_folder))

# Find common files
common_files = sorted(image_files & label_files)

# Write to CSV
with open(output_csv, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for filename in common_files:
        writer.writerow([filename])

print(f"Saved {len(common_files)} common file names to {output_csv}")
