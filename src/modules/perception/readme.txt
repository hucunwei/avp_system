perception功能包
-- 实现感知功能，如：车道线识别/地面箭头识别

1. Receive 4 camera images at the "same" timestamp to form ipm_raw
2. Receive 4 correspondent label images to form ipm_label
3. Create many ipm_raw ipm_label pairs to input Bisenet model to train.
4. Use the training model to inference semantic segmentation for each online coming in ipm_raw

ipm_label is generated as following:
1. form ipm_seg by fusing 4 image segs
2. convert ipm_seg into gray images
3. collect all pixel values from about 200 gray images
4. apply k-means to cluster to 8 classes, get 8 centroids
5. use this 8 centroids to segment each ipm_seg into 0~7 ipm_label
will add python scripts for ipm_label
