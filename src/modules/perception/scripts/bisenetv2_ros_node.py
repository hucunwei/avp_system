#!/usr/bin/env python
import os
import sys
import rospy
import torch
import math
import torch.nn.functional as F
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Add both package root and lib directory
pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
lib_path = os.path.join(pkg_root, 'lib')

for path in [pkg_root, lib_path]:
    if path not in sys.path:
        sys.path.insert(0, path)

import lib.data.transform_cv2 as T
from lib.models import model_factory
from config import set_cfg_from_file

class BiSeNetV2Segmenter:
    def __init__(self):
        rospy.init_node('bisenetv2_segmenter')

        # Initialize attributes
        self.cfg = None
        self.palette = None
        self.net = None
        self.to_tensor = None
        self.device = None
        self.bridge = None

        # Setup components
        self.setup_model()
        self.setup_ros_components()

        rospy.loginfo("BiSeNetV2 Segmenter initialized and ready")

    def setup_model(self):
        """Initialize the BiSeNetV2 model with configuration"""
        try:
            import rospkg
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('perception')

            # Device setup
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            torch.set_grad_enabled(False)

            # Get parameters with fallback
            config_file = rospy.get_param('~config_file', os.path.join(pkg_path, 'config', 'bisenetv1_SUPS.py'))
            weight_path = rospy.get_param('~weight_path', os.path.join(pkg_path, 'models', 'model_final.pth'))

            rospy.loginfo(f"config: {config_file}")
            rospy.loginfo(f"config: {weight_path}")

            if not os.path.exists(weight_path):
                raise FileNotFoundError(f"Model weights not found at {weight_path}")

            # Load model config and weights
            self.cfg = set_cfg_from_file(config_file)
            self.palette = np.random.randint(0, 256, (256, 3), dtype=np.uint8)

            self.net = model_factory[self.cfg.model_type](self.cfg.n_cats, aux_mode='eval')

            try:
                state_dict = torch.load(weight_path, map_location='cpu', weights_only=True)
            except TypeError:
                # fallback for older PyTorch
                state_dict = torch.load(weight_path, map_location='cpu')

            self.net.load_state_dict(state_dict, strict=False)
            self.net.eval().to(self.device)

            # Image transform
            self.to_tensor = T.ToTensor(
                mean=(0.3257, 0.3690, 0.3223),
                std=(0.2112, 0.2148, 0.2115),
            )

            rospy.loginfo(f"Model loaded on device: {self.device}")

        except Exception as e:
            rospy.logerr(f"Failed to initialize model: {str(e)}")
            rospy.signal_shutdown("Model initialization failed")

    def setup_ros_components(self):
        """Initialize ROS communication components"""
        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = rospy.Subscriber(
            rospy.get_param('~input_topic', '/ipm_raw'),
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24  # Large buffer for high-res images
        )

        # Publisher
        self.seg_pub = rospy.Publisher(
            rospy.get_param('~output_topic', '/ipm_seg'),
            Image,
            queue_size=1,
        )

    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS Image to OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Convert to RGB and preprocess
            im = cv_image[:, :, ::-1]  # BGR to RGB
            im = self.to_tensor(dict(im=im, lb=None))['im'].unsqueeze(0).to(self.device)

            # Get original and padded sizes
            org_size = im.size()[2:]
            new_size = [math.ceil(el / 32) * 32 for el in im.size()[2:]]

            # Inference
            im = F.interpolate(im, size=new_size,
                               align_corners=False, mode='bilinear')
            out = self.net(im)[0]
            out = F.interpolate(out, size=org_size,
                                align_corners=False, mode='bilinear')
            out = out.argmax(dim=1)

            # Convert to color segmentation map
            out = out.squeeze().detach().cpu().numpy()
            pred = self.palette[out]
            pred_bgr = cv2.cvtColor(pred, cv2.COLOR_RGB2BGR)

            seg_msg = self.bridge.cv2_to_imgmsg(pred_bgr, "bgr8")
            seg_msg.header = msg.header  # Maintain original header
            self.seg_pub.publish(seg_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

if __name__ == '__main__':
    try:
        segmenter = BiSeNetV2Segmenter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Cleanup
        if torch.cuda.is_available():
            torch.cuda.empty_cache()