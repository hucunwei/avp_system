perception功能包
-- 实现感知功能，如：车道线识别/地面箭头识别

1. Receive 4 camera images at the "same" timestamp to form ipm_raw (using ipm.cc)
2. Receive 4 correspondent segmentation images to form ipm_seg, and then segment into ipm_label (refer to the next section for details)
3. Create many ipm_raw ipm_label pairs to input Bisenet model to train.
4. Use the training model to inference semantic segmentation for each online coming in ipm_raw

ipm_label is generated as following:
1. form ipm_seg by fusing 4 image segs
2. convert ipm_seg into gray images (scripts/color2gray.py)
3. collect all pixel values from about 200 gray images
4. apply k-means to cluster to 8 classes, get 8 centroids (scripts/extract_rgb_kmeans.py)
5. convert each ipm_seg into gray image (scripts/color2gray.py), and then use the 8 centroids to segment each ipm_seg into 0~7 ipm_label

For training data preparation, please check out the branch:
https://github.com/hucunwei/avp_system/tree/prepare-train-data
it includes python scripts to form train.csv, label images, etc as the above.

usage:
roslaunch perception perception_node

rosbag play path/to/ros.bag or run simulator.